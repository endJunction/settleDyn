/*
Bullet Continuous Collision Detection and Physics Library C API modifications
Remove unused functions.
Copyright (c) 2011 Dmitrij Yu. Naumov

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

#define PL_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifdef BT_USE_DOUBLE_PRECISION
typedef double	plReal;
#else
typedef float	plReal;
#endif

typedef plReal	plVector3[3];
typedef plReal	plQuaternion[4];

#ifdef __cplusplus
extern "C" { 
#endif

/**	Particular physics SDK (C-API) */
	PL_DECLARE_HANDLE(plPhysicsSdkHandle);

/** 	Dynamics world, belonging to some physics SDK (C-API)*/
	PL_DECLARE_HANDLE(plDynamicsWorldHandle);

/** Rigid Body that can be part of a Dynamics World (C-API)*/	
	PL_DECLARE_HANDLE(plRigidBodyHandle);

/** 	Collision Shape/Geometry, property of a Rigid Body (C-API)*/
	PL_DECLARE_HANDLE(plCollisionShapeHandle);

/**
	Create and Delete a Physics SDK	
*/

	extern	plPhysicsSdkHandle	plNewBulletSdk(); //this could be also another sdk, like ODE, PhysX etc.
	extern	void		plDeletePhysicsSdk(plPhysicsSdkHandle	physicsSdk);


/* Dynamics World */

	extern  plDynamicsWorldHandle plCreateDynamicsWorld(plPhysicsSdkHandle physicsSdk);

	extern  void           plDeleteDynamicsWorld(plDynamicsWorldHandle world);

	extern	void	plStepSimulation(plDynamicsWorldHandle,	plReal	timeStep);

	extern  void plAddRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);

	extern  void plRemoveRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);


/* Rigid Body  */

	extern  plRigidBodyHandle plCreateRigidBody(	void* user_data,  float mass, plCollisionShapeHandle cshape );

	extern  void plDeleteRigidBody(plRigidBodyHandle body);


/* Collision Shape definition */

	extern  plCollisionShapeHandle plNewBoxShape(plReal x, plReal y, plReal z);

	extern  void plDeleteShape(plCollisionShapeHandle shape);

	/* Convex Meshes */
	extern  plCollisionShapeHandle plNewConvexHullShape();
	extern  void		plAddVertex(plCollisionShapeHandle convexHull, plReal x,plReal y,plReal z);

	extern  void plSetScaling(plCollisionShapeHandle shape, plVector3 scaling);

	/* get world transform */
	extern void	plGetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix);
	extern void	plGetPosition(plRigidBodyHandle object,plVector3 position);
	extern void plGetOrientation(plRigidBodyHandle object,plQuaternion orientation);

	/* set world transform (position/orientation) */
	extern  void plSetPosition(plRigidBodyHandle object, const plVector3 position);
	extern  void plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation);
    extern  void plSetMassProps(plRigidBodyHandle object, const plReal mass, const plVector3 inertia);

#ifdef __cplusplus
}
#endif


#endif //BULLET_C_API_H

