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


module Polyhedron (
      Polyhedron
    , points, triangles
    , readPolyhedron
    , getNormal
    , scale
    , size, volume
    , transform
    ) where

import Data.List (sort)
import Data.VectorSpace

import Geometry hiding (transform)
import qualified Geometry as Geom (transform)

import OffReader


data Polyhedron = Polyhedron { points :: [Point]
                             , triangles :: [Triangle]
                             }

instance Show Polyhedron where
    show g = show (size g) ++ " " ++ show (volume g)

transform :: Transformation -> Polyhedron -> Polyhedron
transform t p = p { points = map (Geom.transform t) (points p) }

scale :: Double -> Polyhedron -> Polyhedron
scale s p = p { points = map (s *^) (points p) }

readPolyhedron :: FilePath -> IO Polyhedron
readPolyhedron file =
    readOffFile file >>= \(ps, ts) ->
    return $ Polyhedron {
                points = scalePoints $ centerPoints $ map listToTriples ps,
                triangles = map listToTriples ts }
    where
        -- Move barycenter to Origin.
        centerPoints :: [Point] -> [Point]
        centerPoints ps = map (^-^ barycenter ps) ps

        -- Scale points, such that the resulting set has size 1.
        scalePoints :: [Point] -> [Point]
        scalePoints ps = map (^/ sizePoints ps) ps
        
        listToTriples :: [a] -> (a, a, a)
        listToTriples (x:y:z:_) = (,,) x y z
        listToTriples _ = error "Cannot convert list to triple."

-- Size of a polyhedron is defined by second shortest edge length of its bounding
-- box.
size :: Polyhedron -> Double
size = sizePoints . points

sizePoints :: [Point] -> Double
sizePoints [] = 0
sizePoints ps =
    case bbox ps of
        Nothing -> 0
        Just box -> flip (!!) 1 $ sort $ edgeLengths box
    where edgeLengths = toList . uncurry (flip (^-^))

volume :: Polyhedron -> Double
volume p = volumeSurface $ map (triangleCoordinates (points p)) $ triangles p

-- Given a convex triangulated surface with the origin in its interior, the
-- enclosed volume is the sum of tetrahedrons' volumes from origin to the
-- surface triangles.
volumeSurface :: [(Point, Point, Point)] -> Double
volumeSurface = foldl (flip $ (+) . volumeTetO) 0
