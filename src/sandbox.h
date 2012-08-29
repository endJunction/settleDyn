/* This file is part of "settleDyn" software.
 *
 * Copyright (C) 2012
 *           Dmitrij Yu. Naumov
 *
 * "settleDyn" is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * "settleDyn" is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * "settleDyn".  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Author: Dmitrij Yu. Naumov
 */

#ifndef SETTLEDYN_SANDBOX_H
#define SETTLEDYN_SANDBOX_H


#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletMultiThreaded/PosixThreadSupport.h>
#include <bullet/BulletMultiThreaded/PlatformDefinitions.h>
#include <bullet/BulletMultiThreaded/btParallelConstraintSolver.h>
#include <bullet/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h>

#include <bullet/BulletDynamics/Dynamics/btRigidBody.h>
#include <bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.h>

namespace SettleDyn {

const int numThreads = 2;

/// Sandbox is specialized bullet physics environment.
class
Sandbox : public btDiscreteDynamicsWorld {

    public:

    /// Size of the sandbox is [-xySize/2, xySize/2]^2 in x and y directions and
    /// 1000 units in vertical direction.
    Sandbox(btDispatcher* dispatcher,
            btBroadphaseInterface* broadphase,
            btConstraintSolver* solver,
            btCollisionConfiguration* collisionConfiguration)
        : btDiscreteDynamicsWorld(
                dispatcher,
                broadphase,
                solver,
                collisionConfiguration),
          _dispatcher(dispatcher),
          _broadphase(broadphase),
          _solver(solver),
          _collisionConfiguration(collisionConfiguration)
    {

        // Create ground plane and walls.
        staticBodies.push_back(createBody(new btStaticPlaneShape(btVector3( 0, 1,  0),  0)));
        staticBodies.push_back(createBody(new btStaticPlaneShape(btVector3( 1, 0,  0), -2.5)));
        staticBodies.push_back(createBody(new btStaticPlaneShape(btVector3(-1, 0,  0), -2.5)));
        staticBodies.push_back(createBody(new btStaticPlaneShape(btVector3( 0, 0,  1), -2.5)));
        staticBodies.push_back(createBody(new btStaticPlaneShape(btVector3( 0, 0, -1), -2.5)));
    }

    virtual ~Sandbox()
    {
        std::vector<btCollisionShape*> collisionShapes;
        for (btRigidBody* i : staticBodies)
        {
            collisionShapes.push_back(i->getCollisionShape());
            delete i;
        }

        for (btCollisionShape* i : collisionShapes)
        {
            delete i;
        }

        delete _collisionConfiguration;
        delete _dispatcher;
        delete _broadphase;
        delete _solver;

        // To delete threadSupportSolver from solverConstructionInfo created in
        // constructSandbox.
    }

    private:

    btRigidBody*
    createBody(btCollisionShape* shape, const float mass = 0)
    {
        btAssert(shape);

        btVector3 localInertia(0, 0, 0);
        if (mass > 1e-6)
        {
            shape->calculateLocalInertia(mass,localInertia);
        }

        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0, 0, 0));

        btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
        void* mem = btAlignedAlloc(sizeof(btRigidBody),16);
        btRigidBody* body = new (mem)btRigidBody(
            btRigidBody::btRigidBodyConstructionInfo(
                mass, motionState, shape, localInertia));

        this->addRigidBody(body);
        return body;
    }

    private:

    Sandbox(const Sandbox&);
    Sandbox& operator==(const Sandbox&);

    private:

    btDispatcher* _dispatcher;
    btBroadphaseInterface* _broadphase;
    btConstraintSolver* _solver;
    btCollisionConfiguration* _collisionConfiguration;

    std::vector<btRigidBody*> staticBodies;
};

Sandbox*
constructSandbox(const float xySize)
{
    //
    // Collision configuration.
    //
    btDefaultCollisionConfiguration* collisionConfiguration
        = new btDefaultCollisionConfiguration();

    //
    // Dispatcher.
    //
    btCollisionDispatcher* dispatcher
        = new btCollisionDispatcher(collisionConfiguration);

    //
    // Broadphase.
    //
    btAxisSweep3* broadphase = new btAxisSweep3(
        btVector3(-xySize/2-1, -xySize/2-1, -1),
        btVector3( xySize/2+1,  xySize/2+1, 1000));

    //
    // Solver.
    //
    btSequentialImpulseConstraintSolver* solver
        = new btSequentialImpulseConstraintSolver;
    //
    // Posix threads.
    //
    //PosixThreadSupport::ThreadConstructionInfo solverConstructionInfo(
    //    "solver", SolverThreadFunc, SolverlsMemoryFunc, numThreads);
    //PosixThreadSupport* threadSupportSolver
    //    = new PosixThreadSupport(solverConstructionInfo);
    //btParallelConstraintSolver* solver
    //    = new btParallelConstraintSolver(threadSupportSolver);


    //
    // Create and config world.
    //
    Sandbox* sandbox = new Sandbox(dispatcher, broadphase, solver,
            collisionConfiguration);

    sandbox->setGravity(btVector3(0, -9.81, 0));

    return sandbox;
}

}   // namespace SettleDyn

#endif  // SETTLEDYN_SANDBOX_H
