#pragma once

#include <Discregrid/mesh/triangle_mesh.hpp>
#include "../common.hpp"

namespace std {
	template <> struct hash<Discregrid::Vector3r>
	{
		std::size_t operator()(Discregrid::Vector3r const& x) const
		{
			std::size_t seed = 0;
			std::hash<Discregrid::Real> hasher;
			seed ^= hasher(x[0]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
			seed ^= hasher(x[1]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
			seed ^= hasher(x[2]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
			return seed;
		}
	};

	template <> struct less<Discregrid::Vector3r>
	{
		bool operator()(Discregrid::Vector3r const& left, Discregrid::Vector3r const& right) const
		{
			for (auto i = 0u; i < 3u; ++i)
			{
				if (left(i) < right(i))
					return true;
				else if (left(i) > right(i))
					return false;
			}
			return false;
		}
	};
}
#include <Discregrid/utility/lru_cache.hpp>

#include <Discregrid/acceleration/bounding_sphere_hierarchy.hpp>


#include <array>
#include <vector>
#include <set>
#include <map>
#include <unordered_map>

#include <Eigen/Dense>

namespace Discregrid
{

enum class NearestEntity;
class TriangleMesh;
class Halfedge;
class MeshDistance
{

	struct Candidate
	{
		bool operator<(Candidate const& other) const { return b < other.b; }
		unsigned int node_index;
		Discregrid::Real b, w;
	};

public:

	MeshDistance(TriangleMesh const& mesh, bool precompute_normals = true);

	// Returns the shortest unsigned distance from a given point x to
	// the stored mesh.
	// Thread-safe function.
	Discregrid::Real distance(Discregrid::Vector3r const& x, Discregrid::Vector3r* nearest_point = nullptr,
		unsigned int* nearest_face = nullptr, NearestEntity* ne = nullptr) const;

	// Requires a closed two-manifold mesh as input data.
	// Thread-safe function.
	Discregrid::Real signedDistance(Discregrid::Vector3r const& x) const;
	Discregrid::Real signedDistanceCached(Discregrid::Vector3r const& x) const;

	Discregrid::Real unsignedDistance(Discregrid::Vector3r const& x) const;
	Discregrid::Real unsignedDistanceCached(Discregrid::Vector3r const& x) const;

private:

	Discregrid::Vector3r vertex_normal(unsigned int v) const;
	Discregrid::Vector3r edge_normal(Halfedge const& h) const;
	Discregrid::Vector3r face_normal(unsigned int f) const;

	void callback(unsigned int node_index, TriangleMeshBSH const& bsh,
		Discregrid::Vector3r const& x, 
		Discregrid::Real& dist) const;

	bool predicate(unsigned int node_index, TriangleMeshBSH const& bsh, 
		Discregrid::Vector3r const& x, Discregrid::Real& dist) const;

private:

	TriangleMesh const& m_mesh;
	TriangleMeshBSH m_bsh;

	using FunctionValueCache = LRUCache<Discregrid::Vector3r, Discregrid::Real>;
	mutable std::vector<TriangleMeshBSH::TraversalQueue> m_queues;
	mutable std::vector<unsigned int> m_nearest_face;
	mutable std::vector<FunctionValueCache> m_cache;
	mutable std::vector<FunctionValueCache> m_ucache;
	
	std::vector<Discregrid::Vector3r> m_face_normals;
	std::vector<Discregrid::Vector3r> m_vertex_normals;
	bool m_precomputed_normals;
};

}

