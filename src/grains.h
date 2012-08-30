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

#ifndef SETTLEDYN_GRAINS_H
#define SETTLEDYN_GRAINS_H

#include <fstream>
#include <string>
#include <vector>

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletDynamics/Dynamics/btRigidBody.h>

namespace SettleDyn {

// Read .OFF file into triangle mesh. Only triangles are allowed in the surface
// description.
btTriangleMesh*
readOffFile(const std::string filename)
{
    std::vector<btVector3> vertices;

    std::ifstream file(filename.c_str());
    bool isBinary = false;

    std::string tmpString;
    // Read header.
    std::getline(file, tmpString);
    if (tmpString != std::string("OFF") && tmpString != std::string("OFF BINARY"))
        return 0;
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

    vertices.reserve(nPoints);
    for (size_t i = 0; i < 3*nPoints; i++) {
        double v[3];
        file >> v[0] >> v[1] >> v[2];
        vertices.push_back(btVector3(v[0], v[1], v[2]));
    }

    // 16bit, cartesian co-ordinates.
    btTriangleMesh* mesh = new btTriangleMesh(false, false);
    mesh->preallocateVertices(nPoints);
    //mesh->preallocateTriangles(nTriangles);

    int vi[3];  // vertex indices forming a triangle.
    for (size_t i = 0; i < nTriangles; i++) {
        int v;
        file >> v;
        // Accept only triangles.
        if (v != 3)
            return 0;

        for (int j = 0; j < 3; j++) {
            file >> vi[0] >> vi[1] >> vi[2];
            mesh->addTriangle(vertices[vi[0]], vertices[vi[1]], vertices[vi[2]]);
            if (isBinary) {
                // Skip color.
                file >> v;
                double c;
                for (int k = 0; k < v; k++)
                    file >> c;
            }
        }
    }

    return mesh;
}

std::vector<btCollisionShape*>
createPrototypes(const std::vector<std::string>& ps)
{
    // Create collision shapes from prototype descriptions. Simple shapes
    // are created according to the strings.
    std::vector<btCollisionShape*> shapes;

    for (const std::string p : ps)
    {
        // Skip unsupported shapes silently.
        if (p == "CUBE")
            shapes.push_back(new btBoxShape(btVector3(0.5, 0.5, 0.5)));
        if (p == "SPHERE")
            shapes.push_back(new btSphereShape(0.5));
    }

    return shapes;
}

//
// Comparator and getter objects for grain's properties.
//
// Works as comparator if two parameters are given and as getter if one
// parameter is given.
//

struct GrainsLinearVelocity
{
    typedef btScalar ReturnType;

    bool
    operator()(const btRigidBody* const a, const btRigidBody* const b) const
    {
        return a->getLinearVelocity().length()
                < b->getLinearVelocity().length();
    }

    btScalar
    operator()(const btRigidBody* const a) const
    {
        return a->getLinearVelocity().length();
    }
};

struct GrainsAngularVelocity
{
    typedef btScalar ReturnType;

    bool
    operator()(const btRigidBody* const a, const btRigidBody* const b) const
    {
        return a->getAngularVelocity().length()
                < b->getAngularVelocity().length();
    }

    btScalar
    operator()(const btRigidBody* const a) const
    {
        return a->getAngularVelocity().length();
    }
};

struct GrainsHeight
{
    typedef btScalar ReturnType;

    bool
    operator()(const btRigidBody* const a, const btRigidBody* const b) const
    {
        return a->getCenterOfMassPosition().y()
                < b->getCenterOfMassPosition().y();
    }

    btScalar
    operator()(const btRigidBody* const a) const
    {
        return a->getCenterOfMassPosition().y();
    }
};


}   // namespace SettleDyn

#endif  // SETTLEDYN_GRAINS_H
