/*
Bullet Continuous Collision Detection and Physics Library C API modifications
Remove unused functions.
Copyright (c) 2011, 2012 Dmitrij Yu. Naumov

Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef TYPES_H
#define TYPES_H

#define PL_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifdef BT_USE_DOUBLE_PRECISION
typedef double	pl_Real;
#else
typedef float	pl_Real;
#endif

typedef pl_Real	pl_Vector3[3];
typedef pl_Real	pl_Quaternion[4];

#ifdef __cplusplus
extern "C" { 
#endif

/** 	Dynamics world, belonging to some physics SDK (C-API)*/
	PL_DECLARE_HANDLE(pl_DynamicsWorldHandle);

/** Rigid Body that can be part of a Dynamics World (C-API)*/	
	PL_DECLARE_HANDLE(pl_RigidBodyHandle);

/** 	Collision Shape/Geometry, property of a Rigid Body (C-API)*/
	PL_DECLARE_HANDLE(pl_CollisionShapeHandle);

#ifdef __cplusplus
}
#endif


#endif // TYPES_H

