{-

"Settle3D" is a sedimentation process simulator.  It is capable to generate
loose-sand--like structures from given particle forms and statistical
distributions.


Copyright (C) 2009, 2010, 2011
          Helmholtz Centre Potsdam, GFZ German Research Centre for Geosciences.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.


Author: Dmitrij Yu. Naumov

-}

import System.Exit (exitWith, ExitCode(..))
import Control.Monad (replicateM_, when)
import Control.Concurrent.MVar (readMVar)

import Polyhedron (Polyhedron, readPolyhedron, writePolyhedron, points, triangles)
import Simulation (State, makeState, stepSimulation, simulationStep, saveGrains, writeGrainsStatistics)

import qualified Config (setOptions, getOptions, maxSimulationSteps, outputDirectory, prototypeFiles, verbose, showHelp, saveEveryStep)
import qualified CLI

import System.Environment (getArgs)

import System.Directory (createDirectory, doesDirectoryExist)

import Text.Printf (printf)
import PovWriter (toPovray)
import Geometry (Transformation)

saveAndExit :: State -> IO ()
saveAndExit state = do
    saveGrains state grainWriter
    writeGrainsStatistics state $ Config.outputDirectory ++ "/statistics.txt"
    exitWith ExitSuccess

main :: IO ()
main = do

    c <- Config.getOptions
    getArgs >>= CLI.parseOptions c >>= Config.setOptions

    -- Show help message and exit
    when (Config.showHelp) $
        putStrLn (CLI.helpMessage)
            >> exitWith ExitSuccess

    when (Config.verbose) $
        Config.getOptions >>= print

    -- Read grain shapes from prototype files.
    when (null Config.prototypeFiles) $
        print "No prototype files were given. Exiting."
            >> exitWith (ExitFailure 2)
    grainPrototypes <- mapM readPolyhedron Config.prototypeFiles


    -- Check output directory and create when necessary.
    outputDirExists <- doesDirectoryExist Config.outputDirectory
    if outputDirExists
        then print "Output directory exists. Files may be overwritten."
        else createDirectory Config.outputDirectory
            >> print ("New output directory " ++ show Config.outputDirectory
                        ++ " created.")


    state <- makeState grainPrototypes

    replicateM_ Config.maxSimulationSteps $ do
        finished <- stepSimulation state
        when Config.saveEveryStep $ do
            step <- readMVar (simulationStep state)
            saveGrains state (povWriter step)
        when finished (saveAndExit state)

    -- Save results in any case.
    saveAndExit state

povWriter :: Int -> Int -> Polyhedron -> Transformation -> IO ()
povWriter stepI grainJ p = write file . toPovray (points p) (triangles p)
    where
        file = Config.outputDirectory ++ "/grains" ++ printf "%06d" stepI ++ ".mesh"
        write = if grainJ == 0 then writeFile else appendFile

grainWriter :: Int -> Polyhedron -> Transformation -> IO ()
grainWriter grainJ p = writePolyhedron file p
    where
        file = Config.outputDirectory ++ "/grain" ++ printf "%06d" grainJ
