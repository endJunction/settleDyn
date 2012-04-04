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

#include <cstdlib>

#include "grains.h"
#include "sandbox.h"

int
main(int argc, char* argv[])
{

/* Load prototypes.
 *  A prototype is a triangulated closed surface (a polyhedron).
 *    + Use simpler bullet bodies e.g. sphere, cube, etc. for faster
 *      simulations.
 *  Prototypes are read from OFF-files.
 *  It has (sieve-)size, volume and is scalable.
 *
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

/* Interaction with running simulation.
 *  - Pause or finish simulation.
 *  - Change grain properties (friction, make static i.e. freeze).
 *  - Insert new grain at given position.
 */

/* Observing simulation.
 *  - Grain positions, grain linear and angular velocities.
 *  + use motion states to track positional changes.
 */

    return EXIT_SUCCESS;
}
