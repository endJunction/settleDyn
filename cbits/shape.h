#ifndef SHAPE_H
#define SHAPE_H

#include "types.h"

#ifdef __cplusplus
extern "C" { 
#endif

/* Collision Shape definition */

	extern pl_CollisionShapeHandle pl_NewBoxShape(pl_Real x, pl_Real y, pl_Real z);

	extern void pl_DeleteShape(pl_CollisionShapeHandle shape);

	/* Convex Meshes */
	extern pl_CollisionShapeHandle pl_NewConvexHullShape();
	extern void		pl_AddVertex(pl_CollisionShapeHandle convexHull, pl_Real x,pl_Real y,pl_Real z);

	extern void pl_SetScaling(pl_CollisionShapeHandle shape, pl_Vector3 scaling);

    extern void readOffFile(const char* filename, int* nPoints, pl_Real** points,
        int* nTriangles, int** triangles);

#ifdef __cplusplus
}
#endif


#endif // SHAPE_H
