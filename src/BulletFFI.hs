{- This file is part of "settleDyn" software.

Copyright (C) 2011, 2012
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


This code is based on Csaba Hruska's Haskell bullet-0.1.1 (2009) library code.

Author: Dmitrij Yu. Naumov

-}


{-# LANGUAGE ForeignFunctionInterface #-}

module BulletFFI (
    PlDynamicsWorldHandle,
    PlRigidBodyHandle,

    plCreateDynamicsWorld,
    plStepSimulation,

    readOffFile,
    plCreateConvexRigidBody,
    plCreateBoxRigidBody,
    plPlaceRigidBody, plPlaceRigidBody_,
    plMakeRigidBodyStatic,

    plGetOpenGLMatrix,
    plGetVelocity,
    plGetPosition,
) where

import Foreign
import Foreign.C.Types
import Data.List.Split (splitEvery)
import Foreign.C.String (CString(), withCString)

-- Pointers to structures.
data PlCollisionShape = PlCollisionShape
type PlCollisionShapeHandle = Ptr PlCollisionShape

data PlDynamicsWorld = PlDynamicsWorld
type PlDynamicsWorldHandle = Ptr PlDynamicsWorld

data PlRigidBody = PlRigidBody
type PlRigidBodyHandle = Ptr PlRigidBody

--
-- Creating bullet physics world.
--
plCreateDynamicsWorld :: (Real a) => Triple a -> Triple a -> IO PlDynamicsWorldHandle
plCreateDynamicsWorld worldMin worldMax =
    withTriple' worldMin (\ wMin ->
    withTriple' worldMax (\ wMax ->
    plCreateDynamicsWorld_ wMin wMax))
    where withTriple' = flip withTriple

foreign import ccall safe "plCreateDynamicsWorld" plCreateDynamicsWorld_
    :: PlVector3 -> PlVector3 -> IO PlDynamicsWorldHandle

--
-- Step simulation.
--
plStepSimulation :: PlDynamicsWorldHandle -> Double -> IO ()
plStepSimulation dw dt = plStepSimulation_ dw $ realToFrac dt

foreign import ccall safe "pl_StepSimulation" plStepSimulation_
    :: PlDynamicsWorldHandle -> CFloat -> IO ()


newtype PlVector3 = PlVector3 (Ptr CFloat)
type Triple a = (a, a, a)

withTriple :: Real a => (PlVector3 -> IO b) -> Triple a -> IO b
withTriple f v = withArray (tripleToList v) (f . PlVector3)
    where
    tripleToList :: Real a => Triple a -> [CFloat]
    tripleToList (a0, a1, a2) = map realToFrac [a0, a1, a2]

-- | Call function, where the last function argument is an array of given length
-- filled by called function.
callWithArray :: (Real a, Storable a, Fractional b) => Int -> (Ptr a -> IO ()) -> IO [b]
callWithArray l f = allocaArray l
    (\p -> f p >> fmap (map realToFrac) (peekArray l p))

listToTriple :: [a] -> Triple a
listToTriple as
    | length as == 3 = (as!!0, as!!1, as!!2)
    | otherwise = error $ "Try to convert list of length " ++ show (length as)
        ++ " to triple."

foreign import ccall safe "pl_SetMassProps" plSetMassProps_
    :: PlRigidBodyHandle -> CFloat -> Ptr CFloat -> IO ()
plMakeRigidBodyStatic :: PlRigidBodyHandle -> IO ()
plMakeRigidBodyStatic b = allocaArray 3 $
    plSetMassProps_ b 0

foreign import ccall safe "plGetOpenGLMatrix" plGetOpenGLMatrix_
    :: PlRigidBodyHandle -> Ptr CFloat -> IO ()
plGetOpenGLMatrix :: PlRigidBodyHandle -> IO [Double]
plGetOpenGLMatrix body = callWithArray 16 (plGetOpenGLMatrix_ body)

foreign import ccall safe "pl_GetVelocity" plGetVelocity_
    :: PlRigidBodyHandle -> Ptr CFloat -> IO ()
plGetVelocity :: PlRigidBodyHandle -> IO (Triple Double)
plGetVelocity body = fmap listToTriple $ callWithArray 3 (plGetVelocity_ body)

foreign import ccall safe "plGetPosition" plGetPosition_
    :: PlRigidBodyHandle -> Ptr CFloat -> IO ()
plGetPosition :: PlRigidBodyHandle -> IO (Triple Double)
plGetPosition body = fmap listToTriple $ callWithArray 3 (plGetPosition_ body)

--
-- Creating rigid bodies.
--

plPlaceRigidBody_ :: Real a => IO PlRigidBodyHandle -> Triple a -> PlDynamicsWorldHandle -> IO ()
plPlaceRigidBody_ createBody pos dw = plPlaceRigidBody createBody pos dw >> return ()

plPlaceRigidBody :: Real a => IO PlRigidBodyHandle -> Triple a -> PlDynamicsWorldHandle -> IO PlRigidBodyHandle
plPlaceRigidBody createBody pos dw = do
    b <- createBody
    plAddRigidBody dw b
    plSetPosition b pos
    return b

foreign import ccall safe "pl_AddRigidBody" plAddRigidBody
    :: PlDynamicsWorldHandle -> PlRigidBodyHandle -> IO ()

foreign import ccall safe "plSetPosition" plSetPosition_
    :: PlRigidBodyHandle -> PlVector3 -> IO ()
plSetPosition :: Real a => PlRigidBodyHandle -> Triple a -> IO ()
plSetPosition b = withTriple (plSetPosition_ b)

type PlUserDataHandle = Ptr ()
foreign import ccall safe "pl_CreateRigidBody" plCreateRigidBody
    :: PlUserDataHandle -> CFloat -> PlCollisionShapeHandle -> IO PlRigidBodyHandle

--
-- OFF file reader.
--
foreign import ccall safe "readOffFile" readOffFile_
    :: CString -> Ptr CInt -> Ptr (Ptr CFloat) -> Ptr Int -> Ptr (Ptr CInt) -> IO ()

readOffFile :: String -> IO ([[Double]], [[Int]])
readOffFile file = do
    alloca $ \nPoints -> do
    alloca $ \points -> do
    alloca $ \nTriangles -> do
    alloca $ \triangles -> do
    withCString file (\c_file -> readOffFile_ c_file nPoints points nTriangles triangles)
    np <- fmap fromIntegral $ peek nPoints
    nt <- fmap fromIntegral $ peek nTriangles
    psptr <- peek points
    tsptr <- peek triangles
    ps <- fmap (splitEvery 3 . map realToFrac) (peekArray np psptr)
    ts <- fmap (splitEvery 3 . map fromIntegral) (peekArray nt tsptr)
    return (ps, ts)

--
-- Convex rigid body from list of points.
--
plCreateConvexRigidBody :: Real a => [Triple a] -> IO PlRigidBodyHandle
plCreateConvexRigidBody ps = do
    ch <- plNewConvexHullShape Foreign.nullPtr 0 0
    mapM_ (plAddVertex ch) ps
    plSetScaling ch (1-0.04, 1-0.04, 1-0.04 :: CFloat)
    -- New rigid body with given mass and shape.
    plCreateRigidBody Foreign.nullPtr 1 ch

foreign import ccall safe "pl_NewConvexHullShape" plNewConvexHullShape
    :: Ptr CFloat -> Int -> Int -> IO PlCollisionShapeHandle

foreign import ccall safe "pl_AddVertex" plAddVertex_
    :: PlCollisionShapeHandle -> CFloat -> CFloat -> CFloat -> IO ()
plAddVertex :: Real a => PlCollisionShapeHandle -> Triple a -> IO ()
plAddVertex shape (x, y, z) =
    plAddVertex_ shape (realToFrac x) (realToFrac y) (realToFrac z)

foreign import ccall safe "pl_SetScaling" plSetScaling_
    :: PlCollisionShapeHandle -> PlVector3 -> IO ()
plSetScaling :: Real a => PlCollisionShapeHandle -> Triple a -> IO ()
plSetScaling shape = withTriple (plSetScaling_ shape)

--
-- Box
--
plCreateBoxRigidBody :: Real a => Triple a -> IO PlRigidBodyHandle
plCreateBoxRigidBody dims = plNewBoxShape dims >>= plCreateRigidBody Foreign.nullPtr 0

foreign import ccall safe "pl_NewBoxShape" plNewBoxShape_
    :: CFloat -> CFloat -> CFloat -> IO PlCollisionShapeHandle
plNewBoxShape:: Real a => Triple a -> IO PlCollisionShapeHandle
plNewBoxShape (w, h, d) = plNewBoxShape_ (realToFrac w) (realToFrac h) (realToFrac d)
