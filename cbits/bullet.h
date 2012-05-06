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

#ifndef BULLET_C_API_H
#define BULLET_C_API_H

#include "types.h"

#ifdef __cplusplus
extern "C" { 
#endif

/* Dynamics World */

	extern pl_DynamicsWorldHandle pl_CreateDynamicsWorld(pl_Vector3 worldAabbMin, pl_Vector3 worldAabbMax);

	extern void           pl_DeleteDynamicsWorld(pl_DynamicsWorldHandle world);

	extern void	pl_StepSimulation(pl_DynamicsWorldHandle,	pl_Real	timeStep);

	extern void pl_AddRigidBody(pl_DynamicsWorldHandle world, pl_RigidBodyHandle object);

	extern void pl_RemoveRigidBody(pl_DynamicsWorldHandle world, pl_RigidBodyHandle object);


/* Rigid Body  */

	extern pl_RigidBodyHandle pl_CreateRigidBody(	void* user_data,  float mass, pl_CollisionShapeHandle cshape );

	extern void pl_DeleteRigidBody(pl_RigidBodyHandle body);



	/* get world transform */
	extern void	pl_GetOpenGLMatrix(pl_RigidBodyHandle object, pl_Real* matrix);
	extern void	pl_GetPosition(pl_RigidBodyHandle object,pl_Vector3 position);
	extern void pl_GetOrientation(pl_RigidBodyHandle object,pl_Quaternion orientation);
    void pl_GetVelocity(pl_RigidBodyHandle object, pl_Vector3 velocity);

	/* set world transform (position/orientation) */
	extern void pl_SetPosition(pl_RigidBodyHandle object, const pl_Vector3 position);
	extern void pl_SetOrientation(pl_RigidBodyHandle object, const pl_Quaternion orientation);
    extern void pl_SetMassProps(pl_RigidBodyHandle object, const pl_Real mass, const pl_Vector3 inertia);

#ifdef __cplusplus
}
#endif


#endif //BULLET_C_API_H

