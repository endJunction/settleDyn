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


#include <algorithm>
#include <random>
#include <vector>
#include <time.h>

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

        this->setGravity(btVector3(0, -9.81, 0));
    }

    btCollisionShape*
    removeRigidBodySaveShape(btRigidBody* body)
    {
        btCollisionShape* shape = body->getCollisionShape();
        if (body->getMotionState())
            delete body->getMotionState();

        this->removeRigidBody(body);
        delete body;
        return shape;
    }

    virtual ~Sandbox()
    {
        // Do not remove collision shapes of the grains here, since the
        // collision shapes are managed by prototypes vector.
        std::vector<btCollisionShape*> shapes;
        for (btRigidBody* i : grains)
            removeRigidBodySaveShape(i);

        for (btRigidBody* i : staticBodies)
            shapes.push_back(removeRigidBodySaveShape(i));

        // Sort and unique before deletion.
        std::sort(shapes.begin(), shapes.end());
        shapes.erase(std::unique(shapes.begin(), shapes.end()), shapes.end());
        for (btCollisionShape* i : shapes)
            delete i;

        delete _solver;
        delete _broadphase;
        delete _dispatcher;
        delete _collisionConfiguration;

        // To delete threadSupportSolver from solverConstructionInfo created in
        // constructSandbox.
    }

    template <typename F>
    typename F::ReturnType
    getGrainsMaximum(const F f)
    {
        typedef std::vector<btRigidBody*>::const_iterator CI;

        CI g = std::max_element(grains.begin(), grains.end(), f);

        if (g == grains.end())
            return typename F::ReturnType();

        return f(*g);
    }

    void
    addGrain(btCollisionShape* prototype, const btScalar height = 100)
    {
        static std::mt19937 rng(time(NULL));
        static std::uniform_real_distribution<float> rnd_transl(-1, 1);
        static std::uniform_real_distribution<float> rnd_rot(0, 360);

        btRigidBody* grain = createBody(prototype, 1);
        grains.push_back(grain);

        const btVector3 translation(rnd_transl(rng), height, rnd_transl(rng));
        const btQuaternion rotation(rnd_rot(rng), rnd_rot(rng), rnd_rot(rng));
        grain->setWorldTransform(btTransform(rotation, translation));
    }

    std::vector<btRigidBody*>
    getGrains() const
    {
        return grains;
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
    std::vector<btRigidBody*> grains;
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
