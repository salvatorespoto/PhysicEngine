#include <vector>

#include "PhysicEngine/geometry/extrema.h"
#include "PhysicEngine/geometry/Vertex.h"
#include "PhysicEngine/geometry/Edge.h"
#include "PhysicEngine/geometry/Face.h"
#include "PhysicEngine/physics/RigidBody.h"


namespace PhysicEngine 
{

	PolyhedronBSPTree::PolyhedronBSPTree(const RigidBody& rb) : polyhedron(rb)
	{

		// Build the spherical dual of the polyhedron.
		// The spherical dual is made from spherical polygons defined on a sphere. 
		// Each vertex of a spherical poligon represent a normal, each arc connecting normals represent
		// an edge and each spherical poligon represent a vertex. 
		
		// Build edges of the spherical polygons dual
		std::vector<PolyhedronBSPTree::PolyhedronBSPNode> nodes;
		std::vector<Arc> arcs;
		for(int i = 0; i < polyhedron.Faces.size(); i++)
		{
			Face f1 = polyhedron.Faces[i];
			
			for(int j = i+1; j < polyhedron.Faces.size(); j++)
			{
				Face f2 = polyhedron.Faces[j];

				for(Edge e1 : f1.Edges) 
				{
					if (std::find_if(f2.Edges.begin(), f2.Edges.end(),
						[&e1](Edge e2)
						{
							return e1.I(0) == e2.I(1) && e1.I(1) == e2.I(0);
						}) != f2.Edges.end())
					{
						Arc arc;
						arc.i = i;
						arc.j = j;
						arcs.push_back(arc);
					}
				}
			}
		}

		// Compute dual spherical polygons
		std::vector<SphericalPoly> polygons(rb.Vertices.size());
		for (int i = 0; i < polygons.size(); i++) polygons[i].vId = i;
		for (int i=0; i<rb.Faces.size(); i++)
		{
			Face f = rb.Faces[i];
			for (int j = 0; j < f.size(); j++) 
			{
				polygons[f.I(j)].v.insert(i);
				polygons[f.I(j)].vId = f.I(j);
			}
		}

		// Recursively build the three
		PolyhedronBSPTree::PolyhedronBSPNode* root = BuildTree(arcs, polygons);
	}
	

	PolyhedronBSPTree::PolyhedronBSPNode* PolyhedronBSPTree::BuildTree(std::vector<Arc> arcs, std::vector<SphericalPoly> polygons)
	{
		PolyhedronBSPNode *n = new PolyhedronBSPNode();
		std::vector<Arc> leftA;
		std::vector<Arc> rightA;
		std::vector<SphericalPoly> leftS;
		std::vector<SphericalPoly> rightS;

		if(arcs.size() > 0) 
		{
			n->A = arcs.back();
			arcs.pop_back();
		} 
		else
		{
			n->S = polygons[0];
			n->isLeaf = true;
			return n;
		}
		
		glm::vec3 E = glm::cross(polyhedron.Faces[n->A.i].N(), polyhedron.Faces[n->A.j].N());
		
		for(Arc A : arcs) 
		{
			float di = glm::dot(E, polyhedron.Faces[A.i].N());
			float dj = glm::dot(E, polyhedron.Faces[A.j].N());

			if (di >= 0 && dj >= 0)
			{
				rightA.push_back(A);			
			}
			if (di <= 0 && dj <= 0)
			{
				leftA.push_back(A);
			}
			if(di * dj < 0)
			{
				rightA.push_back(A);
				leftA.push_back(A);
			}
		}

		for (SphericalPoly S : polygons)
		{
			bool onlyLeft = true;
			bool onlyRight = true;

			for(int k : S.v) 
			{
				if (glm::dot(E, polyhedron.Faces[k].N()) > 0)
				{
					onlyLeft &= false;
				}
				if (glm::dot(E, polyhedron.Faces[k].N()) < 0)
				{
					onlyRight &= false;
				}
			}

			if (!onlyLeft) rightS.push_back(S);
			if (!onlyRight) leftS.push_back(S);
		}

		if(!leftA.empty()) 
		{
			n->left = BuildTree(leftA, leftS);
		}
		else 
		{
			n->left = NULL;
		}
		if (!rightA.empty())
		{
			n->right = BuildTree(rightA, rightS);
		}
		else
		{
			n->right = NULL;
		}
		
		return n;
	}
}