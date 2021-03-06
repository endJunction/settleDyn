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

#include "types.h"
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"	
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"

#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Dynamics World */
pl_DynamicsWorldHandle
pl_CreateDynamicsWorld(
    btVector3 worldAabbMin = btVector3(-100,-1,-100),
    btVector3 worldAabbMax = btVector3( 100, 100, 100))
{
    void* mem = btAlignedAlloc(sizeof(btDefaultCollisionConfiguration),16);
    btDefaultCollisionConfiguration* collisionConfiguration =
        new (mem)btDefaultCollisionConfiguration();

    mem = btAlignedAlloc(sizeof(btCollisionDispatcher),16);
    btDispatcher* dispatcher =
        new (mem)btCollisionDispatcher(collisionConfiguration);

    mem = btAlignedAlloc(sizeof(btAxisSweep3),16);
    btBroadphaseInterface* pairCache =
        new (mem)btAxisSweep3(worldAabbMin, worldAabbMax);

    mem = btAlignedAlloc(sizeof(btSequentialImpulseConstraintSolver),16);
    btConstraintSolver* constraintSolver =
        new(mem) btSequentialImpulseConstraintSolver();

    mem = btAlignedAlloc(sizeof(btDiscreteDynamicsWorld),16);
    btDiscreteDynamicsWorld* dynamicsWorld =
        new (mem)btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration);

    dynamicsWorld->getSolverInfo().m_splitImpulse=true;
    return (pl_DynamicsWorldHandle) dynamicsWorld;
}

void
pl_DeleteDynamicsWorld(pl_DynamicsWorldHandle world)
{
    btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);

    btAlignedFree(dynamicsWorld->getConstraintSolver());
    btAlignedFree(dynamicsWorld->getBroadphase());
    btAlignedFree(dynamicsWorld->getDispatcher());

    // DynamicsWorld does not safe collisionConfiguration object, only
    // stackAllocator. Not clear how to delete collisionConfiguration from
    // inside of this destructor.

    btAlignedFree(dynamicsWorld);
}

void	pl_StepSimulation(pl_DynamicsWorldHandle world,	pl_Real	timeStep)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	dynamicsWorld->stepSimulation(timeStep, 10, 1./1000);
}

void pl_AddRigidBody(pl_DynamicsWorldHandle world, pl_RigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);

	dynamicsWorld->addRigidBody(body);
}

void pl_RemoveRigidBody(pl_DynamicsWorldHandle world, pl_RigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);

	dynamicsWorld->removeRigidBody(body);
}

/* Rigid Body  */

pl_RigidBodyHandle pl_CreateRigidBody(	void* user_data,  float mass, pl_CollisionShapeHandle cshape )
{
	btTransform trans;
	trans.setIdentity();
	btVector3 localInertia(0,0,0);
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	btAssert(shape);
	if (mass)
	{
		shape->calculateLocalInertia(mass,localInertia);
	}
	void* mem = btAlignedAlloc(sizeof(btRigidBody),16);
	btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0,shape,localInertia);
	btRigidBody* body = new (mem)btRigidBody(rbci);
	body->setWorldTransform(trans);
	body->setUserPointer(user_data);
	return (pl_RigidBodyHandle) body;
}

void pl_DeleteRigidBody(pl_RigidBodyHandle cbody)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	btAlignedFree( body);
}

void pl_SetMassProps(pl_RigidBodyHandle object, const pl_Real mass, const pl_Vector3 inertia)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	body->setMassProps(mass, btVector3(inertia[0],inertia[1],inertia[2]));
}



void pl_SetPosition(pl_RigidBodyHandle object, const pl_Vector3 position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btVector3 pos(position[0],position[1],position[2]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setOrigin(pos);
	body->setWorldTransform(worldTrans);
}

void pl_SetOrientation(pl_RigidBodyHandle object, const pl_Quaternion orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btQuaternion orn(orientation[0],orientation[1],orientation[2],orientation[3]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setRotation(orn);
	body->setWorldTransform(worldTrans);
}

void	pl_GetOpenGLMatrix(pl_RigidBodyHandle object, pl_Real* matrix)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	body->getWorldTransform().getOpenGLMatrix(matrix);

}

void	pl_GetPosition(pl_RigidBodyHandle object,pl_Vector3 position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btVector3& pos = body->getWorldTransform().getOrigin();
	position[0] = pos.getX();
	position[1] = pos.getY();
	position[2] = pos.getZ();
}

void pl_GetOrientation(pl_RigidBodyHandle object,pl_Quaternion orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btQuaternion& orn = body->getWorldTransform().getRotation();
	orientation[0] = orn.getX();
	orientation[1] = orn.getY();
	orientation[2] = orn.getZ();
	orientation[3] = orn.getW();
}

void pl_GetVelocity(pl_RigidBodyHandle object, pl_Vector3 velocity)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btVector3& v = body->getLinearVelocity();
	velocity[0] = v.getX();
	velocity[1] = v.getY();
	velocity[2] = v.getZ();
}

#ifdef __cplusplus
}
#endif
