{- This file is part of "settle3D" software.

Copyright (C) 2009, 2010
          Helmholtz Centre Potsdam, GFZ German Research Centre for Geosciences.

"Settle3D" is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

"Settle3D" is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
"settle3D".  If not, see <http://www.gnu.org/licenses/>.


Author: Dmitrij Yu. Naumov

-}


module Simulation (
      State(..), makeState
    , stepSimulation
    , prependGrain
    , saveGrains, writeGrainsStatistics, writePovFiles
    , sandBoxWalls, groundPlane
    ) where

import Physics.Bullet
import Control.Concurrent.MVar (MVar, modifyMVar_, readMVar, swapMVar, newMVar)
import Foreign (nullPtr)
import Foreign.C.Types (CFloat)

import Control.Monad (forM_, when, zipWithM_)

import Grains (
          Grain(..)
        , size, volume
        , scalePrototype
        )

import Random (Random(randomIO, randomRIO))
import System.IO
import Text.Printf (printf)
import PovWriter (toPovray)

import Transformation

import qualified Config

type Triple a = (a, a, a)
type Point = Triple CFloat
type Tri = Triple Int


data State = State {
        dworld     ::  PlDynamicsWorldHandle
      , prototypes ::  [([Point], [Tri])]
      , grainsTrafos :: MVar [Transformation] -- Matrices of transformations
      , grainsMovingStep :: MVar [Int]  -- Last simulation step, when grain was moving
      , grains     ::  MVar [Grain]
      , bodies :: MVar [PlRigidBodyHandle]
      , simulationStep :: MVar Int  -- current step
}

prependGrain :: State -> Grain -> PlRigidBodyHandle -> IO ()
prependGrain state g b = do
    modifyMVar_ (grains state) (return . (g:))
    modifyMVar_ (bodies state) (return . (b:))
    t <- getGrainsTransformation b
    modifyMVar_ (grainsTrafos state) (return . (t:))
    i <- readMVar (simulationStep state)
    modifyMVar_ (grainsMovingStep state) (return . (i:))

-- Writes sizes and volumes of grains.
writeGrainsStatistics :: State -> FilePath -> IO ()
writeGrainsStatistics s f =
    readMVar (grains s) >>= writeFile f . unlines . map showSV
    where 
        showSV :: Grain -> String
        showSV g = show (size g) ++ " " ++ show (volume g)

-- Writes the grains in the current state with grainWriter.
saveGrains :: State -> (String -> [Point] -> [Tri] -> Transformation -> IO ()) -> IO ()
saveGrains s grainWriter =
    readMVar (grains s) >>= \gs ->
    readMVar (grainsTrafos s) >>= \trans ->
    forM_ (zip3 [(0::Int)..] gs trans) $ \(i, Grain (ps, ts), t) ->
        grainWriter (file i) ps ts t
        where file i = Config.outputDirectory ++ "/grain" ++ show i

writePovFiles :: State -> IO ()
writePovFiles s = do
    i <- readMVar (simulationStep s)
    gs <- readMVar (grains s)
    gts <- readMVar (grainsTrafos s)
    let f = Config.outputDirectory ++ "/grains" ++ printf "%06d" i ++ ".mesh"

    withFile f WriteMode (\fh ->
        zipWithM_ (\(Grain (ps, ts)) -> hPutStr fh . toPovray ps ts) gs gts)

getGrainsTransformation :: PlRigidBodyHandle -> IO Transformation
getGrainsTransformation b = do
    (ax,ay,az,_,bx,by,bz,_,cx,cy,cz,_,tx,ty,tz,_) <- plGetOpenGLMatrix b
    return $ array ((0,0), (3,2)) [
        ((0,0), ax), ((0,1), ay), ((0,2), az),
        ((1,0), bx), ((1,1), by), ((1,2), bz),
        ((2,0), cx), ((2,1), cy), ((2,2), cz),
        ((3,0), tx), ((3,1), ty), ((3,2), tz) ]
    
-------------------------------------------------------------------------------

-- Computes translational velocities of grains' two transformations.
grainsVelocities :: [Transformation] -> [Transformation] -> CFloat -> [CFloat]
grainsVelocities tsPrev tsNew dt = zipWith velocity tsPrev tsNew where
    velocity tPrev tNew = 1/dt * sum 
        (zipWith (abs . (-)) (getTranslation tPrev) (getTranslation tNew))

-- Vertical translation part of the transformation matrix
getTranslation :: Transformation -> [CFloat]
getTranslation t = [t!(3,0), t!(3,1), t!(3,2)]

-- Find height of grains given by their transformations.
-- Height is grains maximum vertical position. Only an approximate value is
-- needed, so we look the vertical translation part of transformation only. The
-- barycenters of the grains are in Origin.
grainsHeight :: [Transformation] -> CFloat
grainsHeight = foldl (flip (max . getHeight)) 0
    where
        -- Select vertical component of translation
        getHeight :: Transformation -> CFloat
        getHeight = flip (!) (3,1)

stepSimulation :: State -> Int -> IO Bool
stepSimulation s dt = do
    -- Compute next simulation timestep.
    -- dt in milliseconds.
    plStepSimulation (dworld s) $ fromIntegral dt
    modifyMVar_ (simulationStep s) $ return . (1+)
    currentStep <- readMVar (simulationStep s)

    -- Update grains' transformations:
    -- get new transformation matrices
    gtsNew <- mapM getGrainsTransformation =<< readMVar (bodies s)
    -- and replace instead of previous simulation step transformations.
    gts <- swapMVar (grainsTrafos s) gtsNew

    let totalGrains = length gtsNew
        -- Compute grains' velocities (translations).
        norms = grainsVelocities gts gtsNew $ fromIntegral dt

        -- Mark moving grains with True and static with False
        minMovingNorm = min 1e-3 $ max Config.movingThreshold (maximum norms)/10
        isMoving = map (> minMovingNorm) norms

        nMovingGrains = length $ filter (True ==) isMoving

        -- Select static grains' transformations from gtsNew list.
        staticGs = fst $ unzip $ filter (not . snd) (zip gtsNew isMoving)

        height = grainsHeight staticGs

    -- Update simulation time step of grainsMovingStep
    let updateGrainsMovingStep :: [Int] -> [Int]
        updateGrainsMovingStep oldValues = map
            (\(moving, time) -> if moving then currentStep else time)
            (zip isMoving oldValues)

    modifyMVar_ (grainsMovingStep s) (return . updateGrainsMovingStep)

    -- Freeze grains which are not moving for given number of simulation steps.
    bs <- readMVar (bodies s)
    gms <- readMVar (grainsMovingStep s)
    let freezeGs = fst $ unzip $ filter
            ((< currentStep - Config.freezeTimeSteps) . snd) (zip bs gms)
        nFrozenGs = length freezeGs

    mapM_ plMakeRigidBodyStatic freezeGs
    when (nFrozenGs > 0) $ print $
        show nFrozenGs ++ " grains frozen."

    
        
    let finished = totalGrains > Config.maxNumberGrains
            || height > Config.maxGrainsHeight

    when (not finished
        && nMovingGrains < Config.maxMovingGrains)
        $ createNewGrain s (height+Config.generateGrainsOffset + 2*Config.grainsSizeMean)

    when (Config.verbose) $ print $
        "Grains total/moving/maxV/height: " ++
        show totalGrains ++ "/" ++
        show nMovingGrains ++ "/" ++
        show (if null norms then 0.0 else maximum norms) ++ "/" ++
        show height ++ "."

    return (finished
        && nMovingGrains*1000 < totalGrains)


-------------------------------------------------------------------------------

makeState :: [([Point], [Tri])] -> IO State
makeState ps = do
    sdk <- plNewBulletSdk
    dw <- plCreateDynamicsWorld sdk

    -- set up bullet scene

    mapM_ (createBCube dw) groundPlane
    mapM_ (createBCube dw) sandBoxWalls

    gs <- newMVar []
    gts <- newMVar []
    gms <- newMVar []
    bs <- newMVar []
    step <- newMVar 0
    return State {
        dworld = dw
      , grains = gs
      , bodies = bs
      , grainsTrafos = gts
      , grainsMovingStep = gms
      , prototypes = ps
      , simulationStep = step
    }

-------------------------------------------------------------------------------

pointIsInsideCube :: ([CFloat], CFloat) -> [CFloat] -> Bool
pointIsInsideCube (p0, s) p = maximum diff < s
    where diff = zipWith (abs . (-)) p0 p

isSpaceGrainFree :: ([CFloat], CFloat) -> State -> IO Bool
isSpaceGrainFree space state
    = fmap (not . any (pointIsInsideCube space . getTranslation))
        (readMVar (grainsTrafos state))

createNewGrain :: State -> CFloat -> IO ()
createNewGrain state height = do
    -- Generate grains' scale and position
    s <- generateGrainSize
    let boxDim = Config.grainsGenerationBox
    x <- randomRIO (-boxDim, boxDim :: CFloat)
    z <- randomRIO (-boxDim, boxDim :: CFloat)

    spaceIsFree <- isSpaceGrainFree ([x, height, z], s*1.1) state
    -- skip grain generation to prevent grain overlap
    when (Config.verbose) $ if spaceIsFree
        then print "OK -- free space"
        else print "SKIP this one."

    when spaceIsFree $ createNewGrainAt state ((x, height, z), s)


createNewGrainAt :: State -> (Point, CFloat) -> IO ()
createNewGrainAt state ((x, height, z), s) = do
    -- Select prototype.
    let protos = prototypes state
    prototype <- randomRIO (0, length protos - 1)
    let p = protos !! prototype
    let (ps, ts) = scalePrototype p s

    -- Add grain to simulation environment.
    ch <- plNewConvexHullShape Foreign.nullPtr 0 0
    mapM_ (\(u,v,w) -> plAddVertex ch u v w) ps
    plSetScaling ch (1-0.04, 1-0.04, 1-0.04)
    -- New rigid body with given mass and shape.
    b <- plCreateRigidBody Foreign.nullPtr 1 ch
    plAddRigidBody (dworld state) b

    plSetPosition b (x, height, z)

    let g = Grain (ps, ts)

    when (Config.verbose) $ print $
        "grain's size/volume are " ++ show (size g) ++ "/" ++ show (volume g)

    prependGrain state g b

-- Univariate distribution -----------------------------------------------------

-- k is not used
-- l is not used
-- m is the return value
univariate :: (RealFloat a) => (a, a, a) -> a -> a
univariate (_, _, m) _ = m

-- Bivariate distribution -----------------------------------------------------

-- k is the proportion of l's, k \in [0,1),
-- l is lower value and
-- m is upper value.
bivariate :: (RealFloat a) => (a, a, a) -> a -> a
bivariate (k, l, m) y
    | y < k     = l
    | otherwise = m

-- Weibull distribution -------------------------------------------------------

-- The inverse cumulative density function of Weibull distribution takes three
-- parameters:
--  - k is the form parameter,
--  - l is the magnitude and
--  - m is the shift parameter.

icfdWeibull :: (RealFloat a) => (a, a, a) -> a -> a
icfdWeibull (k, l, m) y
    | y <= 0 = m
    | otherwise = m + (l - m) * abs (log y)**(1/k)


-- Usually the grain size distribution is given in mass-per cent and not in
-- numbers. Assume, there is a map from grain's size to its volume
-- volume(s) = c*s^3, where c is some shape-dependent coefficient.
-- The grain size distribution can be then given in grain counts: Instead of the
-- given paramaters K and L one shall use k = K/3 and l = volume(L).
generateGrainSize :: IO CFloat
generateGrainSize =
    let k = Config.grainsSizeSlope
        m = Config.grainsSizeMin
        l = Config.grainsSizeMean
        g = case Config.grainsSizeGenerator of
            "univariate" -> univariate
            "bivariate" -> bivariate
            "weibull" -> icfdWeibull
            _ -> error "Unknown random number distribution"
    in
    fmap (g (k, l, m)) randomIO

-------------------------------------------------------------------------------

createBCube :: PlDynamicsWorldHandle -> (Point, Point) -> IO ()
createBCube dw ((x,y,z), (w,h,d)) = do
  shape <- plNewBoxShape w h d
  b <- plCreateRigidBody Foreign.nullPtr 0 shape
  plAddRigidBody dw b
  plSetPosition b (x,y,z)

groundPlane = [((0,-1,0), (50.0,1.0,50.0))]

leftWall  = ((-7,0.5, 0), (1, 100, 7))
rightWall = (( 7,0.5, 0), (1, 100, 7))
backWall  = (( 0,0.5,-7), (7, 100, 1))
frontWall = (( 0,0.5, 7), (7, 100, 1))

sandBoxWalls = [leftWall, rightWall, backWall, frontWall]
