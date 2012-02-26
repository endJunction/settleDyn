#ifndef SHAPE_H
#define SHAPE_H

#include "types.h"

#ifdef __cplusplus
extern "C" { 
#endif

/* Collision Shape definition */

	extern  plCollisionShapeHandle plNewBoxShape(plReal x, plReal y, plReal z);

	extern  void plDeleteShape(plCollisionShapeHandle shape);

	/* Convex Meshes */
	extern  plCollisionShapeHandle plNewConvexHullShape();
	extern  void		plAddVertex(plCollisionShapeHandle convexHull, plReal x,plReal y,plReal z);

	extern  void plSetScaling(plCollisionShapeHandle shape, plVector3 scaling);

#ifdef __cplusplus
}
#endif


#endif // SHAPE_H
