# settleDyn

settleDyn is a sedimentation process simulator.  It is capable of generating
loose sand-like structures from given particle forms and statistical
distributions.

This software is a complete rewrite of the Settle3D program written by Guido
Blöcher. See corresponding [paper] [BZ08] for further details.

[BZ08]: http://dx.doi.org/10.1016/j.cageo.2007.12.008 "Blöcher, G. and Zimmermann, G. 2008. Settle3D-A numerical generator for artificial porous media.  Comput. Geosci. 34, 12 (Dec. 2008), 1827-1842.  http://dx.doi.org/10.101/j.cageo.2007.12.008."

The complete sedimentation process including diagenesis involves many processes
on variing time and space scales. The first step in sandstone formation is
deposition of grains. This process can be simulated with current software.
Cementation and further compactification or any other diagenetic processes
cannot be simulated with settleDyn.

Simulation process starts with definition of grain shapes and grain size
distributions. The shapes are polyhedral descriptions read from files in
Geomview Object File Format ([.off] [G07]). For now, only convex shapes are allowed.

[G07]: http://www.geomview.org/docs/html/OFF.html "Geomview 1.9.4 Manual, 2008.  Section 4.2 Object File Formats"

After the simulation is finished, the grains in their current positions are
written to specified directory, again in the .off format.


## Requirements

This code is written in Haskell and uses the Bullet Game Physics Engine Library
for collision detection. In order to compile this program you will need:
 - Bullet Physics Library [Bullet](http://bulletphysics.org)
 - Haskell e.g. Glasgow Haskell Compiler (GHC)
   (http://hackage.haskell.org/platform/)
 - Various package dependencies given in the .cabal file.

## Feature development

In near future I shall rewrite same functionality in c++. This will make
compilation and modifications easier.

Possible new features are:
 - More statistical information
 - Other output format interesting especially for very large simulations.
 - Handling concave grains.
 - Randomization of grains' surfaces.
 - ...

Any suggestions are welcome.

## License and Copying

Author:

 - Dmitrij Yu. Naumov <settleDyn@naumov.de>.

Copyright:

 - 2009, 2010, 2011 - Helmholtz Centre Potsdam, GFZ German Research
Centre for Geosciences.
 - 2011, 2012 - Dmitrij Yu. Naumov

-----------------------------

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
