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


module CLI (
      parseOptions
    ) where

import System.Exit
import System.Console.GetOpt

import Config (Options(..))

optionProcessor :: [OptDescr (Options -> Options)]
optionProcessor =
    [
      Option "" ["offset"]
        (ReqArg (\s o -> o { _generateGrainsOffset = read s }) "Float")
        "Generate grains at current grains height plus this offset"
    , Option "b" ["bbox"]
        (ReqArg (\s o -> o { _grainsGenerationBox = read s }) "Float")
        "Generate grains randomly in the square [-b, b]^2, where b is this arguments value"
    , Option "l" ["mean"]
        (ReqArg (\s o -> o { _grainsSizeMean = read s }) "Float")
        "The mean of grains' size distribution"
    , Option "m" ["min"]
        (ReqArg (\s o -> o { _grainsSizeMin = read s }) "Float")
        "The minimum of grains' size"
    , Option "k" ["slope"]
        (ReqArg (\s o -> o { _grainsSizeSlope = read s}) "Float")
        "The shape parameter of the Weibull distribution for grains' sizes"
    , Option "h" ["height"]
        (ReqArg (\s o -> o { _maxGrainsHeight = read s}) "Float")
        "Maximum of grains' height. Stop grains generation after this height is reached."
    , Option "" ["movingGrains"]
        (ReqArg (\s o -> o { _maxMovingGrains = read s}) "Int")
        "Maximum number of moving grains when new grains may be generated."
    , Option "n" ["numberGrains"]
        (ReqArg (\s o -> o { _maxNumberGrains = read s }) "Int")
        "Maximum number of grains"
    , Option "s" ["steps"]
        (ReqArg (\s o -> o { _maxSimulationSteps = read s}) "Int")
        "Maximum of simulation steps to perform. Stop simulation when reached."
    , Option "t" ["threshold"]
        (ReqArg (\s o -> o { _movingThreshold = read s}) "Float")
        "Velocity threshold. Grains moving faster are counted as moving grains. Opposite to static grains"
    , Option "o" ["output"]
        (ReqArg (\s o -> o { _outputDirectory = s }) "Path")
        "Save results in this directory"
    , Option "p" ["prototype"]
        (ReqArg (\s o -> o { _prototypeFiles = s : _prototypeFiles o }) "File")
        "Use this prototype file grains generation. Multiple instance on command line are collected"
    , Option "" ["saveEvery"]
        (NoArg (\o -> o { _saveEveryStep = True }))
        "Save grains' positions every step. Default is false."
    ]

parseOptions :: Options -> [String] -> IO Options
parseOptions currentOptions args =
    case getOpt RequireOrder optionProcessor args of
        (opts, [], []) -> return $ foldl (flip ($)) currentOptions opts
        -- rests and errors are not allowed
        (_, rests, []) -> exitWithError "Unknow parameters:\n" rests
        (_, _, errs)   -> exitWithError [] errs
    where
        exitWithError info infos = putStrLn ("Error: " ++ info ++ unlines infos)
            >> exitWith (ExitFailure 1)
