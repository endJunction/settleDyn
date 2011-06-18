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


module PovWriter (
      toPovray
    ) where


import Data.List (intercalate)
import Geometry

type Triple a = (a, a, a)

showTriple :: (Show a) => Triple a -> String
showTriple (x, y, z) = showVector [x,y,z]

showVector :: (Show a) => [a] -> String
showVector as = "<" ++ sV as ++ ">"
    where sV = intercalate ", " . map show

-- | Convert a polyhedron to mesh2 povray object.
polyToMesh2 :: [Point] -> [Triangle] -> [String]
polyToMesh2 ps ts = concat [
    ["mesh2 {"],
    listOfTriples "vertex_vectors" ps,
    listOfTriples "face_indices" ts,
    [indent "inside_vector z"],
    ["}"]]
    where listOfTriples name xs = map indent $ listOf name xs showTriple

-- | Convert a list of polyhedrons to an array of mesh2 povray objects. Also
-- writes length of array. Declare NPolys and Polys.
toPovray :: [([Point], [Triangle])] -> String
toPovray ps = unlines
    [ "#declare NPolys = " ++ show n ++ ";"
    , "#declare Polys = array[" ++ show n ++ "] {"
    , unlines $ povrayMeshes
    , "}"
    ]
    where
        n = length ps
        povrayMeshes :: [String]
        povrayMeshes = intercalate [","] $ map (uncurry polyToMesh2) ps

listOf :: Show a => String -> [a] -> (a -> String) -> [String]
listOf name vector printer = concat
    [ [name ++ " { " ++ show (length vector) ++ ","],
      map (indent . printer) vector,
      ["}"]
    ]

indent :: String -> String
indent = ("\t" ++)
