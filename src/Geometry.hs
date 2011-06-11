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


module Geometry (
      Point
    , barycenter
    , bbox
    , pointIsOutsideSphere
    , toList
    , transform

    , Triangle
    , getNormal
    , triangleCoordinates
    , volumeTetO

    , Transformation
    , fromOpenGLMatrix
    , getRotTransMatrix

    , magnitude
    , (^+^), (^-^)
    ) where

import Data.Array.Unboxed

import Data.VectorSpace
import Data.Cross (HasCross3(..))

type Triple a = (a, a, a)
type Point = Triple Double
type Triangle = Triple Int
type Transformation = Array (Int, Int) Double

barycenter :: [Point] -> Point
barycenter ps = sumV ps ^/ n
    where n = fromIntegral $ length ps

-- | Bounding box of points.
bbox :: [Point] -> Maybe (Point, Point)
bbox [] = Nothing
bbox (p:ps) = Just (minAll ps, maxAll ps)
    where minAll, maxAll :: [Point] -> Point
          minAll = foldl minV p
          maxAll = foldl maxV p

          -- Component-wise minimum and maximum.
          minV, maxV :: Point -> Point -> Point
          minV (ax, ay, az) (bx, by, bz) = (min ax bx, min ay by, min az bz)
          maxV (ax, ay, az) (bx, by, bz) = (max ax bx, max ay by, max az bz)

-- | Check, if point is outside sphere. A sphere is given by point and radius.
pointIsOutsideSphere :: (Point, Double) -> Point -> Bool
pointIsOutsideSphere (p, r) = (r*r <) . magnitudeSq . (p ^-^)

-- | Convert a point to list.
toList :: Point -> [Double]
toList (x, y, z) = [x, y, z]

-- | Apply transformation to point.
transform :: Transformation -> Point -> Point
transform m (px, py, pz) =
    let [ax, ay, az, bx, by, bz, cx, cy, cz, tx, ty, tz] = getRotTransMatrix m
    in
        (ax*px + ay*py + az*pz + tx,
         bx*px + by*py + bz*pz + ty,
         cx*px + cy*py + cz*pz + tz)

-- | Compute unit normal vector of triangle.
getNormal :: [Point] -> Triangle -> Point
getNormal ps t = normalized $ cross3 (pB ^-^ pA) (pC ^-^ pA)
    where (pA, pB, pC) = triangleCoordinates ps t

-- | Extracts triangle's coordinates from list of points and indices.
triangleCoordinates :: [Point] -> Triangle -> Triple Point
triangleCoordinates ps (a, b, c) = (ps !! a, ps !! b, ps !! c)

-- | Volume of a tetrahedron with one of its points in Origin.
-- Volume is unsigned.
volumeTetO :: (Point, Point, Point) -> Double
volumeTetO (a, b, c) = 1/6 * abs (a <.> (b `cross3` c))

-- | Get rotation matrix with appended translation components. List of 9 + 3
-- components.
getRotTransMatrix :: Transformation -> [Double]
getRotTransMatrix = elems

-- | Convert an openGL matrix to transformation.
fromOpenGLMatrix :: [Double] -> Transformation
fromOpenGLMatrix [ax,ay,az,_,bx,by,bz,_,cx,cy,cz,_,tx,ty,tz,_] =
    array ((0,0), (3,2)) [
        ((0,0), ax), ((0,1), ay), ((0,2), az),
        ((1,0), bx), ((1,1), by), ((1,2), bz),
        ((2,0), cx), ((2,1), cy), ((2,2), cz),
        ((3,0), tx), ((3,1), ty), ((3,2), tz) ]

