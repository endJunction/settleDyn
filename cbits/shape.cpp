
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

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

// Resulting vectors are always _triples_ of doubles (-> points) or integers (->
// triangles).
std::pair<std::vector<double>, std::vector<int> >
readOffFile(const std::string filename)
{
    std::vector<double> points;
    std::vector<int> triangles;

    std::ifstream file(filename.c_str());
    bool isBinary = false;

    std::string tmpString;
    // Read header.
    std::getline(file, tmpString);
    if (tmpString != std::string("OFF") && tmpString != std::string("OFF BINARY"))
        // Error. Return empty points and triangles.
        return std::make_pair(points, triangles);
    if (tmpString == std::string("OFF BINARY")) {
        // Reopen in binary mode and skip header.
        isBinary = true;
        const size_t pos = file.tellg();
        file.close();
        file.open(filename.c_str(), std::ios::in | std::ios::binary);
        file.seekg(pos);
    }

    // Dimensions.
    size_t nPoints, nTriangles, nEdges;
    file >> nPoints >> nTriangles >> nEdges;

    points.reserve(3*nPoints);
    for (size_t i = 0; i < 3*nPoints; i++) {
        double v;
        file >> v;
        points.push_back(v);
    }

    triangles.reserve(3*nTriangles);
    for (size_t i = 0; i < nTriangles; i++) {
        int v;
        file >> v;
        // Accept only triangles.
        if (v != 3) {
            triangles.clear();
            return std::make_pair(points, triangles);
        }
        for (int j = 0; j < 3; j++) {
            file >> v;
            triangles.push_back(v);
            if (isBinary) {
                // Skip color.
                file >> v;
                double c;
                for (int k = 0; k < v; k++)
                    file >> c;
            }
        }
    }

    return std::make_pair(points, triangles);
}

void
readOffFile(const char* filename, int* nPoints, plReal** points,
        int* nTriangles, int** triangles)
{
    const std::pair<std::vector<double>, std::vector<int> > poly
        = readOffFile(filename);

    *nPoints = poly.first.size();
    *points = new plReal[*nPoints];
    std::copy(poly.first.begin(), poly.first.end(), *points);

    *nTriangles = poly.second.size();
    *triangles = new int[*nTriangles];
    std::copy(poly.second.begin(), poly.second.end(), *triangles);
}

