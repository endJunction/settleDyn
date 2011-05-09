{-# LANGUAGE TypeFamilies #-}

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


module Grains (
      Grain(..)
    , readGrainPrototype
    , Point, Triangle
    , getNormal
    , scalePrototype
    , size, volume
    ) where

import Data.List (sort)
import Data.VectorSpace
import Data.Cross (HasCross3(..))

import OffReader (readOffFile)

type Point = (Double, Double, Double)
type Triangle = (Int, Int, Int)

getNormal :: [Point] -> Triangle -> Point
getNormal ps (a, b, c) =
    let pA = ps !! a
        pB = ps !! b
        pC = ps !! c
    in normalized $ cross3 (pB ^-^ pA) (pC ^-^ pA)

data Grain = Grain ([Point], [Triangle])

scalePrototype :: ([Point], [Triangle]) -> Double -> ([Point], [Triangle])
scalePrototype (ps, ts) s = (map (s *^) ps, ts)

readGrainPrototype :: FilePath -> IO ([Point], [Triangle])
readGrainPrototype file =
    readOffFile file >>= \(ps, ts) ->
    return (scalePoints $ centerPoints $ map listToTriples ps
           , map listToTriples ts)
    where
        -- Move barycenter to Origin.
        centerPoints :: [Point] -> [Point]
        centerPoints ps = map (^-^ barycenter ps) ps

        barycenter :: [Point] -> Point
        barycenter ps = sumV ps ^/ n
            where n = fromIntegral $ length ps

        -- Scale points, such that the resulting set has size 1.
        scalePoints :: [Point] -> [Point]
        scalePoints ps = map (^/ sizePoints ps) ps
        
        listToTriples :: [a] -> (a, a, a)
        listToTriples (x:y:z:_) = (,,) x y z
        listToTriples _ = error "Cannot convert list to triple."

-- Size of a grain is defined by second shortest edge length of its bounding
-- box.
size :: Grain -> Double
size (Grain (ps, _)) = sizePoints ps

sizePoints :: [Point] -> Double
sizePoints = flip (!!) 1 . sort . edgeLengths
    where edgeLengths = toList . uncurry (flip (^-^)) . bbox

volume :: Grain -> Double
volume (Grain (ps, ts)) = volumeSurface $ map makeT ts
    where
        makeT (a, b, c) = (ps !! a, ps !! b, ps !! c)

-- Given a convex triangulated surface with the origin in its interior, the
-- enclosed volume is the sum of tetrahedrons' volumes from origin to the
-- surface triangles.
volumeSurface :: [(Point, Point, Point)] -> Double
volumeSurface = foldl (flip $ (+) . volumeTet) 0

-- To calculate a volume enclosed by a convex triangulated surface, we need a
-- volume of a single tetrahedron. Assume, one of tetrahedron's vertices lies in
-- origin, and denote the other by /pA/, /pB/ and /pC/. Then its volume is given
-- by:

volumeTet :: (Point, Point, Point) -> Double
volumeTet (a, b, c) = 1/6 * abs (a <.> (b `cross3` c))

-- Bbox -----------------------------------------------------------------------

type Bbox = (Point, Point)

bbox :: [Point] -> Bbox
bbox ps = (minAll ps, maxAll ps)
    where
      minAll, maxAll :: [Point] -> Point
      minAll = foldl1 minV
      maxAll = foldl1 maxV

-- Operations on points -------------------------------------------------------

toList :: Point -> [Double]
toList (x, y, z) = [x, y, z]

minV :: Point -> Point -> Point
minV (ax, ay, az) (bx, by, bz) = (min ax bx, min ay by, min az bz)
maxV :: Point -> Point -> Point
maxV (ax, ay, az) (bx, by, bz) = (max ax bx, max ay by, max az bz)

