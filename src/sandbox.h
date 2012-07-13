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

namespace SettleDyn {

const int numThreads = 2;

/// Sandbox contains bullet physics environment.
class
Sandbox {

    public:

    /// Size of the sandbox is [-xySize/2, xySize/2]^2 in x and y directions and
    /// 1000 units in vertical direction.
    Sandbox(const float xySize)
    {
        //
        // Posix threads.
        //
        PosixThreadSupport::ThreadConstructionInfo collisionConstructionInfo(
            "collision", processCollisionTask, createCollisionLocalStoreMemory,
            numThreads);
        _threadSupportCollision = new PosixThreadSupport(collisionConstructionInfo);

        PosixThreadSupport::ThreadConstructionInfo solverConstructionInfo(
            "solver", SolverThreadFunc, SolverlsMemoryFunc, numThreads);
        _threadSupportSolver = new PosixThreadSupport(solverConstructionInfo);

        //
        // Collision configuration.
        //
        _collisionConfiguration = new btDefaultCollisionConfiguration();

        //
        // Dispatcher.
        //
        _dispatcher = new btCollisionDispatcher(_collisionConfiguration);

        //
        // Broadphase.
        //
        _broadphase = new btAxisSweep3(
                            btVector3(-xySize/2-1, -xySize/2-1, -1),
                            btVector3( xySize/2+1,  xySize/2+1, 1000));

        //
        // Solver.
        //
        _solver = new btParallelConstraintSolver(_threadSupportSolver);


        //
        // Create and config world.
        //
        _world = new btDiscreteDynamicsWorld(_dispatcher, _broadphase, _solver,
                                             _collisionConfiguration);
        _world->setGravity(btVector3(0, 0, -9.81));

    }

    ~Sandbox()
    {
        delete _world;
        delete _solver;
        delete _broadphase;
        delete _dispatcher;
        delete _collisionConfiguration;

        delete _threadSupportCollision;
        delete _threadSupportSolver;
    }

    private:

    PosixThreadSupport* _threadSupportCollision;
    PosixThreadSupport* _threadSupportSolver;

    btCollisionConfiguration* _collisionConfiguration;
    btDispatcher* _dispatcher;
    btBroadphaseInterface* _broadphase;
    btConstraintSolver* _solver;
    btDiscreteDynamicsWorld* _world;

    btAlignedObjectArray<btCollisionShape*> _collisionShapes;
};

}   // namespace SettleDyn

#endif  // SETTLEDYN_SANDBOX_H
