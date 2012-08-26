/* "settleDyn" is a sedimentation process simulator.  It is capable to generate
 * loose-sand--like structures from given particle forms and statistical
 * distributions.
 * 
 * 
 * Copyright (C) 2012
 *           Dmitrij Yu. Naumov
 * 
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * 
 * Author: Dmitrij Yu. Naumov
 */

#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <string>

#include "grains.h"
#include "sandbox.h"

std::ostream&
operator<<(std::ostream& os, const btVector3& v)
{
    return os << "[" << v.x() << " " << v.y() << " " << v.z() << "]";
}

int
main(int argc, char* argv[])
{

    using namespace SettleDyn;

/* Load prototypes.
 *  A prototype is a triangulated closed surface (a polyhedron).
 *    + Use simpler bullet bodies e.g. sphere, cube, etc. for faster
 *      simulations.
 *  Prototypes are read from OFF-files.
 *  It has (sieve-)size, volume and is scalable.
 */
    std::vector<std::string> prototype_descriptions;
    prototype_descriptions.push_back("CUBE");

    const std::vector<btCollisionShape*> prototypes
        = createPrototypes(prototype_descriptions);

/*
 * Init random generators
 *   Weibull, uniform or bivariate for grain sizes.
 *   Integer for choosing from grain prototypes.
 */

/* Setup simulation:
 *  - Create sandbox.
 *      Sandbox consists of infinite horizontal plane and four vertical walls.
 *      Those are static objects.
 *  - Run simulation loop in own child process (which can be multithreaded).
 */
    Sandbox* sandbox = constructSandbox(10);

/* Interaction with running simulation.
 *  - Pause or finish simulation.
 *  - Change grain properties (friction, make static i.e. freeze).
 *  - Insert new grain at given position.
 */

    for (size_t timeStep = 0; timeStep < 100000; timeStep++)
    {
        if (timeStep % 1000 == 0)
            sandbox->addGrain(prototypes.front());
        sandbox->stepSimulation(0.01);
    }

/* Observing simulation.
 *  - Grain positions, grain linear and angular velocities.
 *  + use motion states to track positional changes.
 */

    delete sandbox;

    for (btCollisionShape* i : prototypes)
        delete i;

    return EXIT_SUCCESS;
}
