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


module PovWriter (
      writePovFile
    , toPovray
    ) where


import Transformation

type Triple a = (a, a, a)

showT :: (Show a) => Triple a -> String
showT (x, y, z) = "<" ++ show x ++ ", " ++ show y ++ ", " ++ show z ++ ">"

showVector :: (Show a) => [a] -> String
showVector as = "<" ++ sV as ++ ">"
    where sV xs = sV' xs ""
          sV' :: (Show a) => [a] -> String -> String
          sV' [] s = s
          sV' [a] s = {-# SCC "sVsingle" #-} s ++ show a
          sV' (a:b:cs) s = {-# SCC "sVlist" #-} sV' (b:cs) (s `seq` s ++ (show a ++ ", "))

type Point a = Triple a
type Tri = Triple Int

toPovray :: (Fractional a) => [Point a] -> [Tri] -> Transformation -> String
toPovray ps ts m = unlines $ concat [
    ["mesh2 {"],
    map indent (listOf "vertex_vectors" ps showT),
    map indent (listOf "face_indices" ts showT),
    ["\tmatrix " ++ showVector (elems m),
     "}"]]

writePovFile :: (Fractional a) => String -> [Point a] -> [Tri] -> Transformation -> IO ()
writePovFile f ps ts = writeFile (f ++ ".pov") . toPovray ps ts

listOf :: Show a => String -> [a] -> (a -> String) -> [String]
listOf name vector printer = concat
    [ [name ++ " { " ++ show (length vector) ++ ","],
      map (indent . printer) vector,
      ["}"]
    ]

indent :: String -> String
indent = ("\t" ++)
