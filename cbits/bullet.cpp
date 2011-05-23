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

#include "bullet.h"
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"	
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"

#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

/* Dynamics World */
plDynamicsWorldHandle
plCreateDynamicsWorld(
    btVector3 worldAabbMin = btVector3(-1000,-1000,-1000),
    btVector3 worldAabbMax = btVector3( 1000, 1000, 1000))
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

    return (plDynamicsWorldHandle) dynamicsWorld;
}

void
plDeleteDynamicsWorld(plDynamicsWorldHandle world)
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

void	plStepSimulation(plDynamicsWorldHandle world,	plReal	timeStep)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	dynamicsWorld->stepSimulation(timeStep);
}

void plAddRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);

	dynamicsWorld->addRigidBody(body);
}

void plRemoveRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	btAssert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);

	dynamicsWorld->removeRigidBody(body);
}

/* Rigid Body  */

plRigidBodyHandle plCreateRigidBody(	void* user_data,  float mass, plCollisionShapeHandle cshape )
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
	return (plRigidBodyHandle) body;
}

void plDeleteRigidBody(plRigidBodyHandle cbody)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	btAssert(body);
	btAlignedFree( body);
}

void plSetMassProps(plRigidBodyHandle object, const plReal mass, const plVector3 inertia)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	body->setMassProps(mass, btVector3(inertia[0],inertia[1],inertia[2]));
}


/* Collision Shape definition */

plCollisionShapeHandle plNewBoxShape(plReal x, plReal y, plReal z)
{
	void* mem = btAlignedAlloc(sizeof(btBoxShape),16);
	return (plCollisionShapeHandle) new (mem)btBoxShape(btVector3(x,y,z));
}

/* Convex Meshes */
plCollisionShapeHandle plNewConvexHullShape()
{
	void* mem = btAlignedAlloc(sizeof(btConvexHullShape),16);
	return (plCollisionShapeHandle) new (mem)btConvexHullShape();
}


void		plAddVertex(plCollisionShapeHandle cshape, plReal x,plReal y,plReal z)
{
	btCollisionShape* colShape = reinterpret_cast<btCollisionShape*>( cshape);
	(void)colShape;
	btAssert(colShape->getShapeType()==CONVEX_HULL_SHAPE_PROXYTYPE);
	btConvexHullShape* convexHullShape = reinterpret_cast<btConvexHullShape*>( cshape);
	convexHullShape->addPoint(btVector3(x,y,z));

}

void plDeleteShape(plCollisionShapeHandle cshape)
{
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	btAssert(shape);
	btAlignedFree(shape);
}
void plSetScaling(plCollisionShapeHandle cshape, plVector3 cscaling)
{
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	btAssert(shape);
	btVector3 scaling(cscaling[0],cscaling[1],cscaling[2]);
	shape->setLocalScaling(scaling);	
}



void plSetPosition(plRigidBodyHandle object, const plVector3 position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btVector3 pos(position[0],position[1],position[2]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setOrigin(pos);
	body->setWorldTransform(worldTrans);
}

void plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btQuaternion orn(orientation[0],orientation[1],orientation[2],orientation[3]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setRotation(orn);
	body->setWorldTransform(worldTrans);
}

void	plGetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	body->getWorldTransform().getOpenGLMatrix(matrix);

}

void	plGetPosition(plRigidBodyHandle object,plVector3 position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btVector3& pos = body->getWorldTransform().getOrigin();
	position[0] = pos.getX();
	position[1] = pos.getY();
	position[2] = pos.getZ();
}

void plGetOrientation(plRigidBodyHandle object,plQuaternion orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btQuaternion& orn = body->getWorldTransform().getRotation();
	orientation[0] = orn.getX();
	orientation[1] = orn.getY();
	orientation[2] = orn.getZ();
	orientation[3] = orn.getW();
}

void plGetVelocity(plRigidBodyHandle object, plVector3 velocity)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btVector3& v = body->getLinearVelocity();
	velocity[0] = v.getX();
	velocity[1] = v.getY();
	velocity[2] = v.getZ();
}
