{- This file is part of "settleDyn" software.

Copyright (C) 2009, 2010, 2011
          Helmholtz Centre Potsdam, GFZ German Research Centre for Geosciences.

          (C) 2011
          Dmitrij Yu. Naumov

"settleDyn" is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

"settleDyn" is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
"settleDyn".  If not, see <http://www.gnu.org/licenses/>.


Author: Dmitrij Yu. Naumov

-}


module Simulation (
      State(..), makeState
    , stepSimulation
    , mapGrains, writeGrainsStatistics
    , createBCube
    , sandBoxWalls
    ) where

import BulletFFI
import Control.Concurrent.MVar (MVar, modifyMVar_, swapMVar, readMVar, newMVar)

import Control.Monad (when)

import Polyhedron (Polyhedron)
import qualified Polyhedron as P (points, scale, transform)

import System.Random (Random(randomIO, randomRIO))

import Geometry

import qualified Config

-- | A grain is a prototype with local scaling. Its pointer in the simulation is
-- also stored.
-- For 'freezing' grains its last moving time is saved.
data Grain = Grain { prototypeIndex :: Int
                   , localScale :: Double
                   , collisionObject :: PlRigidBodyHandle
                   , lastMovingTime :: Int
                   }

-- | Create scaled polyhedron from a grain using list of prototypes.
polyhedron :: [Polyhedron] -> Grain -> Polyhedron
polyhedron ps g = P.scale (localScale g) (ps !! (prototypeIndex g))

grainsInfo :: State -> Grain -> String
grainsInfo s = show . polyhedron (prototypes s)

data State = State { dworld     ::  PlDynamicsWorldHandle
                   , prototypes ::  [Polyhedron]
                   , simulationStep :: MVar Int  -- current step
                   , grains     ::  MVar [Grain]
                   }

-- Writes sizes and volumes of grains.
writeGrainsStatistics :: State -> FilePath -> IO ()
writeGrainsStatistics s f =
    readMVar (grains s) >>= writeFile f . unlines . map (grainsInfo s)

-- Calls given function on list of transformed polyhedrons.
mapGrains :: State -> ([Polyhedron] -> IO ()) -> IO ()
mapGrains s f =
    readMVar (grains s)
    >>= mapM (\ g -> do -- Make transformed polyhedron from grain.
        t <- getGrainsTransformation $ collisionObject g
        return $ P.transform t $ polyhedron (prototypes s) g)
    >>= f

getGrainsTransformation :: PlRigidBodyHandle -> IO Transformation
getGrainsTransformation b = fmap fromOpenGLMatrix $ plGetOpenGLMatrix b
    
-------------------------------------------------------------------------------

-- Find height of a grain. Height is grains maximum vertical position. Only an
-- approximate value is needed, so center of mass positions are sufficient.
getGrainsHeight :: Grain -> IO Double
getGrainsHeight = fmap getVerticalPart . plGetPosition . collisionObject
    where
        getVerticalPart (_, y, _) = y

computeGrainsHeight :: [(Grain, Bool)] -> IO Double
computeGrainsHeight =
    fmap (foldl max 0) . mapM getGrainsHeight . fst . unzip . filter snd

markStaticGrains :: [Grain] -> IO [(Grain, Bool)]
markStaticGrains gs = do
    velocities <- mapM (fmap magnitude . plGetVelocity . collisionObject) gs
        -- Mark static grains with True and static with False
    let isStatic = map (< Config.movingThreshold) velocities
    return $ zip gs isStatic

updateLastMovingTime :: [(Grain, Bool)] -> Int -> [Grain]
updateLastMovingTime gs time =
    map (\ (g, isStatic) ->
            if isStatic then g { lastMovingTime = time } else g
        ) gs

freezeOldStaticGrains :: Int -> [Grain] -> IO Int
freezeOldStaticGrains time gs = do
    mapM_ (plMakeRigidBodyStatic . collisionObject) grainsToFreeze
    return $ length grainsToFreeze
        where grainsToFreeze = filter ((time >) . lastMovingTime) gs

stepSimulation :: State -> IO Bool
stepSimulation s = do
    -- Compute next simulation timestep.
    plStepSimulation (dworld s)
    modifyMVar_ (simulationStep s) $ return . (1+)

    velocities <- mapM (fmap magnitude . plGetVelocity . collisionObject)
        =<< readMVar (grains s)
    staticGrains <- markStaticGrains =<< readMVar (grains s)

    height <- computeGrainsHeight staticGrains


    -- Update simulation time step of moving grains.
    currentStep <- readMVar (simulationStep s)
    gs <- swapMVar (grains s) $  updateLastMovingTime staticGrains currentStep

    -- Freeze grains which are not moving for given number of simulation steps.
    nFrozenGrains <-
        freezeOldStaticGrains (currentStep - Config.freezeTimeSteps) gs

    let totalGrains = length gs
        nMovingGrains = length $ fst $ unzip $ filter (not . snd) staticGrains

        finished = totalGrains >= Config.maxNumberGrains
            || height > Config.maxGrainsHeight

    when (not finished
        && nMovingGrains < Config.maxMovingGrains)
        $ createNewGrain s (height+Config.generateGrainsOffset + 2*Config.grainsSizeMean)

    when (Config.verbose) $ putStr $
        "total/moving/frozen/maxV/height:\t" ++
        show totalGrains ++ "\t" ++
        show nMovingGrains ++ "\t" ++
        show nFrozenGrains ++ "\t" ++
        show (foldl max 0 velocities) ++ "\t" ++
        show height ++ "\n"

    return (finished && nMovingGrains == 0)


-------------------------------------------------------------------------------

makeState :: [Polyhedron] -> IO State
makeState ps = do
    let worldMin = (minimum . fst . unzip $ sandBoxWalls) ^-^ (1, 1, 1)
        worldMax = (maximum . snd . unzip $ sandBoxWalls) ^+^ (1, 1, 1)
    dw <- plCreateDynamicsWorld worldMin worldMax

    -- set up bullet scene

    mapM_ (createBCube dw) sandBoxWalls

    gs <- newMVar []
    step <- newMVar 0
    return State {
        dworld = dw
      , grains = gs
      , prototypes = ps
      , simulationStep = step
    }

-------------------------------------------------------------------------------

isSpaceGrainFree :: (Point, Double) -> State -> IO Bool
isSpaceGrainFree sphere state =
    readMVar (grains state) >>=
    fmap isFree . mapM (plGetPosition . collisionObject)
    where
        isFree :: [Point] -> Bool
        isFree = all (pointIsOutsideSphere sphere)

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
    i <- randomRIO (0, length (prototypes state) - 1)

    -- Create and add collision object to simulation environment.
    let createBody = plCreateConvexRigidBody $
                        P.points $ P.scale scale ((prototypes state) !! i)
    body <- plPlaceRigidBody createBody pos (dworld state)

    time <- readMVar (simulationStep state)
    -- Create grain.
    let g = Grain { prototypeIndex = i
                  , localScale = scale
                  , collisionObject = body
                  , lastMovingTime = time
                  }
    -- Prepend grain to state with correct collision Object.
    modifyMVar_ (grains state) (return . (g:))


    when (Config.verbose) $ putStr $
        "new grain size/volume " ++ grainsInfo state g ++ "\n"


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
--  - l-m is the magnitude and
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
