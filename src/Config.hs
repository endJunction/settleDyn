{- This file is part of "settleDyn" software.

Copyright (C) 2009, 2010, 2011
          Helmholtz Centre Potsdam, GFZ German Research Centre for Geosciences.

          (C) 2011, 2012
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


module Config (
      freezeTimeSteps
    , generateGrainsOffset
    , grainsGenerationBox
    , grainsSizeMean
    , grainsSizeMin
    , grainsSizeSlope
    , grainsSizeGenerator
    , maxGrainsHeight
    , maxMovingGrains
    , maxNumberGrains
    , maxSimulationSteps
    , movingThreshold
    , outputDirectory
    , prototypeFiles
    , verbose
    , showHelp
    , exportEveryStep
    , exportLastStep
    , setOptions, getOptions
    , Options(..)
    ) where

import Data.IORef
import System.IO.Unsafe

data Options = Options {
      _freezeTimeSteps :: Int
    , _generateGrainsOffset :: Double
    , _grainsGenerationBox :: Double -- for random x,z position
    , _grainsSizeMean :: Double
    , _grainsSizeMin :: Double
    , _grainsSizeSlope :: Double
    , _grainsSizeGenerator :: String
    , _maxGrainsHeight :: Double
    , _maxMovingGrains :: Int
    , _maxNumberGrains :: Int
    , _maxSimulationSteps :: Int
    , _movingThreshold :: Double
    , _outputDirectory :: String
    , _prototypeFiles :: [FilePath]
    , _verbose :: Bool
    , _showHelp :: Bool
    , _exportEveryStep :: Bool
    , _exportLastStep :: Bool
    } deriving Show

getOptions :: IO Options
getOptions = readIORef options

setOptions :: Options -> IO ()
setOptions = writeIORef options

{-# NOINLINE options #-}
options :: IORef Options
options = unsafePerformIO $ newIORef defaultOptions

defaultOptions = Options {
      _freezeTimeSteps = 10000
    , _generateGrainsOffset = 4
    , _grainsGenerationBox = 5
    , _grainsSizeMean = 2
    , _grainsSizeMin = 1
    , _grainsSizeSlope = 3
    , _grainsSizeGenerator = "univariate"
    , _maxGrainsHeight = 10
    , _maxMovingGrains = 100
    , _maxNumberGrains = 16000  -- limited by bullet's broadphase algorithm
    , _maxSimulationSteps = 1000000
    , _movingThreshold = 1e-4
    , _outputDirectory = "out"
    , _prototypeFiles = []
    , _verbose = False
    , _showHelp = False
    , _exportEveryStep = False
    , _exportLastStep = False
    }

freezeTimeSteps = _freezeTimeSteps $ unsafePerformIO getOptions
generateGrainsOffset = _generateGrainsOffset $ unsafePerformIO getOptions
grainsGenerationBox = _grainsGenerationBox $ unsafePerformIO getOptions
grainsSizeMean = _grainsSizeMean $ unsafePerformIO getOptions
grainsSizeMin = _grainsSizeMin $ unsafePerformIO getOptions
grainsSizeSlope = _grainsSizeSlope $ unsafePerformIO getOptions
grainsSizeGenerator = _grainsSizeGenerator $ unsafePerformIO getOptions
maxGrainsHeight = _maxGrainsHeight $ unsafePerformIO getOptions
maxMovingGrains = _maxMovingGrains $ unsafePerformIO getOptions
maxNumberGrains = _maxNumberGrains $ unsafePerformIO getOptions
maxSimulationSteps = _maxSimulationSteps $ unsafePerformIO getOptions
movingThreshold = _movingThreshold $ unsafePerformIO getOptions
outputDirectory = _outputDirectory $ unsafePerformIO getOptions
prototypeFiles = _prototypeFiles $ unsafePerformIO getOptions
verbose = _verbose $ unsafePerformIO getOptions
showHelp = _showHelp $ unsafePerformIO getOptions
exportEveryStep = _exportEveryStep $ unsafePerformIO getOptions
exportLastStep = _exportLastStep $ unsafePerformIO getOptions
