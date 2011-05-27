{- This file is part of "settle3D" software.

Copyright (C) 2009, 2010, 2011
          Helmholtz Centre Potsdam, GFZ German Research Centre for Geosciences.

          (C) 2011
          Dmitrij Yu. Naumov

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
    , helpMessage
    ) where

import System.Exit
import System.Console.GetOpt

import Config (Options(..))

helpMessage :: String
helpMessage = usageInfo header optionProcessor
    where
    header = unlines $
        [ ""
        , "Settle3D---a sedimentation process simulator."
        , ""
        , "Copyright (C) 2009 - 2011 Dmitrij Yu. Naumov, GFZ-Potsdam."
        , "This program comes with ABSOLUTELY NO WARRANTY;  This is"
        , "free software, and you are welcome to redistribute it under"
        , "certain conditions;  For details see the LICENSE file."
        , ""
        , ""
        , "Usage: 'settle3D {--prototype=file, ...} [options]'"
        , ""
        , "Valid options are:"
        ]

optionProcessor :: [OptDescr (Options -> Options)]
optionProcessor =
    [
      Option "p" ["prototype"]
        (ReqArg (\s o -> o { _prototypeFiles = s : _prototypeFiles o }) "File")
        ("Use this prototype file grains generation.\n" ++
        "Multiple instance on command line are\n" ++
        "collected.")
    , Option "l" ["mean"]
        (ReqArg (\s o -> o { _grainsSizeMean = read s }) "Float")
        "The mean of grains' size distribution."
    , Option "m" ["min"]
        (ReqArg (\s o -> o { _grainsSizeMin = read s }) "Float")
        "The minimum of grains' size."
    , Option "k" ["slope"]
        (ReqArg (\s o -> o { _grainsSizeSlope = read s}) "Float")
        ("The shape parameter of the Weibull\n" ++
        "distribution for grains' sizes.")
    , Option "d" ["distribution"]
        (ReqArg (\s o -> o { _grainsSizeGenerator = s }) "{univariate|bivariate|weibull}")
        "Grain size distribution"
    , Option "o" ["output"]
        (ReqArg (\s o -> o { _outputDirectory = s }) "Path")
        "Save results in this directory."
    , Option "" ["offset"]
        (ReqArg (\s o -> o { _generateGrainsOffset = read s }) "Float")
        ("Generate grains at current grains height\n" ++
        "plus this offset.")
    , Option "h" ["height"]
        (ReqArg (\s o -> o { _maxGrainsHeight = read s}) "Float")
        ("Maximum of grains' height. Stop grain\n" ++
        "generation after this height is reached.")
    , Option "n" ["numberGrains"]
        (ReqArg (\s o -> o { _maxNumberGrains = read s }) "Int")
        "Maximum number of grains."
    , Option "" ["movingGrains"]
        (ReqArg (\s o -> o { _maxMovingGrains = read s}) "Int")
        ("Maximum number of moving grains when new\n" ++
        "grains may be generated.")
    , Option "t" ["threshold"]
        (ReqArg (\s o -> o { _movingThreshold = read s}) "Float")
        ("Velocity threshold. Grains moving faster than\n" ++
        "this threshold are counted as moving grains,\n" ++
        "opposite to settled grains.")
    , Option "f" ["freezeSteps"]
        (ReqArg (\s o -> o { _freezeTimeSteps = read s}) "Int")
        ("Number of simulation steps after which\n" ++
        "non-moving grains are frozen.")
    , Option "b" ["bbox"]
        (ReqArg (\s o -> o { _grainsGenerationBox = read s }) "Float")
        ("Generate grains randomly in the square\n" ++
        "[-b, b]^2, where b is this arguments value.")
    , Option "s" ["steps"]
        (ReqArg (\s o -> o { _maxSimulationSteps = read s}) "Int")
        ("Maximum of simulation steps to perform.\n" ++
        "Stop simulation when reached.")
    , Option "v" ["verbose"]
        (NoArg (\o -> o { _verbose = True}))
        "Print information every simulation step."
    , Option "" ["help"]
        (NoArg (\o -> o { _showHelp = True}))
        ""
    , Option "" ["saveEvery"]
        (NoArg (\o -> o { _saveEveryStep = True }))
        "Save grains' positions every step."
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
