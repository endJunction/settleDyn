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

	extern  plDynamicsWorldHandle plCreateDynamicsWorld(plVector3 worldAabbMin, plVector3 worldAabbMax);

	extern  void           plDeleteDynamicsWorld(plDynamicsWorldHandle world);

	extern	void	plStepSimulation(plDynamicsWorldHandle,	plReal	timeStep);

	extern  void plAddRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);

	extern  void plRemoveRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);


/* Rigid Body  */

	extern  plRigidBodyHandle plCreateRigidBody(	void* user_data,  float mass, plCollisionShapeHandle cshape );

	extern  void plDeleteRigidBody(plRigidBodyHandle body);



	/* get world transform */
	extern void	plGetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix);
	extern void	plGetPosition(plRigidBodyHandle object,plVector3 position);
	extern void plGetOrientation(plRigidBodyHandle object,plQuaternion orientation);
    extern void plGetVelocity(plRigidBodyHandle object, plVector3 velocity);

	/* set world transform (position/orientation) */
	extern  void plSetPosition(plRigidBodyHandle object, const plVector3 position);
	extern  void plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation);
    extern  void plSetMassProps(plRigidBodyHandle object, const plReal mass, const plVector3 inertia);

#ifdef __cplusplus
}
#endif


#endif //BULLET_C_API_H

