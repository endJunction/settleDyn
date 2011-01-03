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
    , saveEveryStep
    , setOptions, getOptions
    , Options(..)
    ) where

import Data.IORef
import System.IO.Unsafe
import Foreign.C.Types (CFloat)

data Options = Options {
      _freezeTimeSteps :: Int
    , _generateGrainsOffset :: CFloat
    , _grainsGenerationBox :: CFloat -- for random x,z position
    , _grainsSizeMean :: CFloat
    , _grainsSizeMin :: CFloat
    , _grainsSizeSlope :: CFloat
    , _grainsSizeGenerator :: String
    , _maxGrainsHeight :: CFloat
    , _maxMovingGrains :: Int
    , _maxNumberGrains :: Int
    , _maxSimulationSteps :: Int
    , _movingThreshold :: CFloat
    , _outputDirectory :: String
    , _prototypeFiles :: [FilePath]
    , _verbose :: Bool
    , _showHelp :: Bool
    , _saveEveryStep :: Bool
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
    , _saveEveryStep = False
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
saveEveryStep = _saveEveryStep $ unsafePerformIO getOptions
