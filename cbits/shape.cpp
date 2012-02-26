
#include "shape.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"	
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"

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


