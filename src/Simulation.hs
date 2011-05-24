{- This file is part of "settle3D" software.

Copyright (C) 2009, 2010, 2011
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
    , createBCube
    , sandBoxWalls
    ) where

import Data.VectorSpace
import BulletFFI
import Control.Concurrent.MVar (MVar, modifyMVar_, readMVar, newMVar)

import Control.Monad (forM_, when, zipWithM_)

import qualified Grains as G (Grain, points, triangles, scale)

import System.Random (Random(randomIO, randomRIO))
import System.IO
import Text.Printf (printf)
import PovWriter (toPovray)

import Transformation

import qualified Config

type Triple a = (a, a, a)
type Point = Triple Double
type Tri = Triple Int


data State = State {
        dworld     ::  PlDynamicsWorldHandle
      , prototypes ::  [G.Grain]
      , grainsMovingStep :: MVar [Int]  -- Last simulation step, when grain was moving
      , grains     ::  MVar [G.Grain]
      , bodies :: MVar [PlRigidBodyHandle]
      , simulationStep :: MVar Int  -- current step
}

prependGrain :: State -> G.Grain -> PlRigidBodyHandle -> IO ()
prependGrain state g b = do
    modifyMVar_ (grains state) (return . (g:))
    modifyMVar_ (bodies state) (return . (b:))
    i <- readMVar (simulationStep state)
    modifyMVar_ (grainsMovingStep state) (return . (i:))

-- Writes sizes and volumes of grains.
writeGrainsStatistics :: State -> FilePath -> IO ()
writeGrainsStatistics s f =
    readMVar (grains s) >>= writeFile f . unlines . map show

-- Writes the grains in the current state with grainWriter.
saveGrains :: State -> (FilePath -> G.Grain -> Transformation -> IO ()) -> IO ()
saveGrains s grainWriter = do
    gs <- readMVar (grains s)
    trans <- readMVar (bodies s) >>= mapM getGrainsTransformation
    let files = map (\ i -> Config.outputDirectory ++ "/grain" ++ printf "%06d" i) [(0::Int)..]
    sequence_ (zipWith3 grainWriter files gs trans)

writePovFiles :: State -> IO ()
writePovFiles s = do
    i <- readMVar (simulationStep s)
    gs <- readMVar (grains s)
    gts <- mapM getGrainsTransformation =<< readMVar (bodies s)
    let f = Config.outputDirectory ++ "/grains" ++ printf "%06d" i ++ ".mesh"

    withFile f WriteMode (\fh ->
        zipWithM_ (\ g -> hPutStr fh . toPovray (G.points g) (G.triangles g)) gs gts)

getGrainsTransformation :: PlRigidBodyHandle -> IO Transformation
getGrainsTransformation b = do
    [ax,ay,az,_,bx,by,bz,_,cx,cy,cz,_,tx,ty,tz,_] <- plGetOpenGLMatrix b
    return $ array ((0,0), (3,2)) [
        ((0,0), ax), ((0,1), ay), ((0,2), az),
        ((1,0), bx), ((1,1), by), ((1,2), bz),
        ((2,0), cx), ((2,1), cy), ((2,2), cz),
        ((3,0), tx), ((3,1), ty), ((3,2), tz) ]
    
-------------------------------------------------------------------------------

-- Find height of a grain. Height is grains maximum vertical position. Only an
-- approximate value is needed, so center of mass positions are sufficient.
getGrainsHeight :: PlRigidBodyHandle -> IO Double
getGrainsHeight = fmap getVerticalPart . plGetPosition
    where
        getVerticalPart (_, y, _) = y

stepSimulation :: State -> IO Bool
stepSimulation s = do
    -- Compute next simulation timestep.
    plStepSimulation (dworld s)
    modifyMVar_ (simulationStep s) $ return . (1+)
    currentStep <- readMVar (simulationStep s)

    bs <- readMVar $ bodies s

    velocities <- mapM (fmap magnitude . plGetVelocity) bs
    let totalGrains = length bs
        -- Mark static grains with True and static with False
        isStatic = map (< Config.movingThreshold) velocities

        nMovingGrains = length $ filter (False ==) isStatic

        -- Select static grains.
        staticGs = filterBy isStatic bs
            where
                filterBy :: [Bool] -> [a] -> [a]
                filterBy tests = snd . unzip . filter fst . zip tests

    height <- fmap (foldl max 0) $ mapM getGrainsHeight staticGs

    -- Update simulation time step of grainsMovingStep
    let updateGrainsMovingStep :: [Int] -> [Int]
        updateGrainsMovingStep oldValues = map
            (\(static, time) -> if static then time else currentStep)
            (zip isStatic oldValues)

    modifyMVar_ (grainsMovingStep s) (return . updateGrainsMovingStep)

    -- Freeze grains which are not moving for given number of simulation steps.
    gms <- readMVar (grainsMovingStep s)
    let freezeGs = fst $ unzip $ filter
            ((< currentStep - Config.freezeTimeSteps) . snd) (zip bs gms)
        nFrozenGrains = length freezeGs

    mapM_ plMakeRigidBodyStatic freezeGs

        
    let finished = totalGrains >= Config.maxNumberGrains
            || height > Config.maxGrainsHeight

    when (not finished
        && nMovingGrains < Config.maxMovingGrains)
        $ createNewGrain s (height+Config.generateGrainsOffset + 2*Config.grainsSizeMean)

    when (Config.verbose) $ putStr $
        "total/moving/frozen/maxV/height:\t" ++
        show totalGrains ++ "\t" ++
        show nMovingGrains ++ "\t" ++
        show nFrozenGrains ++ "\t" ++
        show (if null velocities then 0.0 else maximum velocities) ++ "\t" ++
        show height ++ "\n"

    return (finished && nMovingGrains == 0)


-------------------------------------------------------------------------------

makeState :: [G.Grain] -> IO State
makeState ps = do
    let worldMin = (minimum . fst . unzip $ sandBoxWalls) ^-^ (1, 1, 1)
        worldMax = (maximum . snd . unzip $ sandBoxWalls) ^+^ (1, 1, 1)
    dw <- plCreateDynamicsWorld worldMin worldMax

    -- set up bullet scene

    mapM_ (createBCube dw) sandBoxWalls

    gs <- newMVar []
    gms <- newMVar []
    bs <- newMVar []
    step <- newMVar 0
    return State {
        dworld = dw
      , grains = gs
      , bodies = bs
      , grainsMovingStep = gms
      , prototypes = ps
      , simulationStep = step
    }

-------------------------------------------------------------------------------

isSpaceGrainFree :: (Point, Double) -> State -> IO Bool
isSpaceGrainFree (p0, radius) state =
    readMVar (bodies state) >>=
    fmap isFree . mapM plGetPosition
    where
        isFree :: [Point] -> Bool
        isFree = all pointIsOutsideSphere

        pointIsOutsideSphere :: Point -> Bool
        pointIsOutsideSphere = (radius <) . magnitude . (p0 ^-^)


createNewGrain :: State -> Double -> IO ()
createNewGrain state height = do
    -- Generate grains' scale and position
    s <- generateGrainSize
    let boxDim = Config.grainsGenerationBox
    x <- randomRIO (-boxDim, boxDim)
    z <- randomRIO (-boxDim, boxDim)

    -- skip grain generation to prevent grain overlap
    spaceIsFree <- isSpaceGrainFree ((x, height, z), s*1.1) state
    when spaceIsFree $ createNewGrainAt state ((x, height, z), s)


createNewGrainAt :: State -> (Point, Double) -> IO ()
createNewGrainAt state (pos, scale) = do
    -- Select prototype.
    let ps = prototypes state
    prototype <- randomRIO (0, length ps - 1)
    let g = G.scale scale $ ps !! prototype

    -- Add grain to simulation environment.
    b <- plPlaceRigidBody (plCreateConvexRigidBody $ G.points g) pos (dworld state)

    when (Config.verbose) $ putStr $
        "new grain size/volume " ++ show g ++ "\n"

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
generateGrainSize :: IO Double
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
createBCube dw (position, boxDimensions) =
    plPlaceRigidBody_ (plCreateBoxRigidBody boxDimensions) position dw

sandboxXZsize :: Double
sandboxXZsize = Config.grainsGenerationBox + Config.grainsSizeMean * 2

groundPlane, leftWall, rightWall, backWall, frontWall :: (Point, Point)

groundPlane = ((0, -1, 0), (2*sandboxXZsize+1, 1, 2*sandboxXZsize+1))
leftWall  = ((-sandboxXZsize,0.5, 0), (1, 2*Config.maxGrainsHeight, sandboxXZsize))
rightWall = (( sandboxXZsize,0.5, 0), (1, 2*Config.maxGrainsHeight, sandboxXZsize))
backWall  = (( 0,0.5,-sandboxXZsize), (sandboxXZsize, 2*Config.maxGrainsHeight, 1))
frontWall = (( 0,0.5, sandboxXZsize), (sandboxXZsize, 2*Config.maxGrainsHeight, 1))

sandBoxWalls :: [(Point, Point)]
sandBoxWalls = [groundPlane, leftWall, rightWall, backWall, frontWall]
