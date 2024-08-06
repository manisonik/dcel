#include "dcel-manifold.h"

using namespace dcel;

manifold::~manifold()
{
	for (auto& v : _vertices) {
		v->edge = nullptr;
	}

	for (auto& f : _faces) {
		f->head = nullptr;
	}
	for (auto& e : _edges) {
		e.second->pair = nullptr;
		e.second->next = nullptr;
		e.second->vert = nullptr;
		e.second->face = nullptr;
	}

	_edges.clear();
	_vertices.clear();
	_faces.clear();
}

vertex_ptr manifold::add_vertex(glm::vec3 pos)
{
	auto v = std::make_shared<vertex>();
	v->attr.pos = pos;
	v->edge = NULL;
	v->attr.normal = glm::vec3();
	_vertices.push_back(v);
	return v;
}

face_ptr manifold::add_face(const std::vector<vertex_ptr>& verts)
{
	face_ptr f = std::make_shared<face>();
	edge_ptr next = std::make_shared<edge>();
	edge_ptr prev = NULL;

	// Store the first vertex
	f->head = next;

	for (std::size_t i = 0; i < verts.size(); i++)
	{
		next->face = f;

		if (i == verts.size() - 1)
		{
			next->vert = verts[0];
			next->next = f->head;
			next->next->prev = next;
			next->prev = prev;
		}
		else
		{
			next->vert = verts[i + 1];
			next->next = std::make_shared<edge>();

			// Store edge
			//_edges.push_back(next->next);

			next->prev = prev;
		}

		// store prev
		prev = next;

		// Set the vertex edge
		verts[i]->edge = next;

		// store the next edge
		next = next->next;
	}

	// Pair faces
	pair_edges(f);

	// Compute face angles
	compute_face_angles(f);

	// Store
	_faces.push_back(f);

	return f;
}

void dcel::manifold::calculate_normals(face_ptr f)
{
	edge_ptr e = f->head;
	do {
		auto v0 = e->vert->attr.pos;
		auto v1 = e->next->vert->attr.pos;
		auto v2 = e->next->next->vert->attr.pos;
		e->vert->attr.normal = glm::triangleNormal(v0, v1, v2);
	} while (e != f->head);
}

void dcel::manifold::calculate_tangent_basis()
{
	for (face_ptr fa : _faces) {
		edge_ptr va = fa->head;
		do {
			glm::vec3 v0 = va->prev->vert->attr.pos;
			glm::vec3 v1 = va->vert->attr.pos;
			glm::vec3 v2 = va->next->vert->attr.pos;

			glm::vec2 uv0 = va->prev->vert->attr.uv;
			glm::vec2 uv1 = va->vert->attr.uv;
			glm::vec2 uv2 = va->next->vert->attr.uv;

			// Edges of the triangle : position delta
			glm::vec3 deltaPos1 = v1 - v0;
			glm::vec3 deltaPos2 = v2 - v0;

			// UV delta
			glm::vec2 deltaUV1 = uv1 - uv0;
			glm::vec2 deltaUV2 = uv2 - uv0;

			float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
			va->vert->attr.tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r;
			va->vert->attr.bitangent = (deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x) * r;
		} while (va != fa->head);
	}
}

void dcel::manifold::pair_edges(face_ptr fb)
{
	for (face_ptr fa : _faces) {
		edge_ptr va = fa->head;
		do {
			edge_ptr vb = fb->head;
			do {
				if (vb->vert->attr.pos == va->prev->vert->attr.pos &&
					vb->prev->vert->attr.pos == va->vert->attr.pos)
				{
					vb->pair = va;
					va->pair = vb;
				}
				vb = vb->next;
			} while (vb != fb->head);

			va = va->next;
		} while (va != fa->head);
	}
}

int dcel::manifold::print_build()
{
	int status = 0;

	printf("Starting commiting process: %i\n\n", status);

	printf("Calculating normals: %i\n\n", status);
	//calculate_normals();
	calculate_tangent_basis();

	// check if all paired
	for (face_ptr f : _faces) {
		printf("\n0x%p: Looping through face\n", f.get());
		printf("----------------------------------------\n");
		edge_ptr edge = f->head;
		do {
			if (edge->pair == NULL) {
				printf("0x%p: Edge not paired.\n", edge.get());
				status = 1;
			}
			else
			{
				glm::vec3 vaa = edge->prev->vert->attr.pos;
				glm::vec3 vab = edge->vert->attr.pos;
				glm::vec3 vba = edge->pair->prev->vert->attr.pos;
				glm::vec3 vbb = edge->pair->vert->attr.pos;

				std::string sa = "[" + glm::to_string(vaa) + "," + glm::to_string(vab) + "]";
				std::string sb = "[" + glm::to_string(vba) + "," + glm::to_string(vbb) + "]";;

				printf("Paired edges:\n0x%p %s\n0x%p %s\n", edge.get(), sa.c_str(), edge->pair.get(), sb.c_str());
			}

			edge = edge->next;
		} while (edge != f->head);
	}

	// Compute ABF++
	compute_abfpp();

	printf("\nFinished commit with status: %i\n", status);
	printf("----------------------------------------\n\n");

	return status;
}

int dcel::manifold::build()
{
	// Calculate angles and weights for ABF++
	initialize_angles_weights();

	// Calculate normals and tangent basis
	for (const auto& f : _faces) {
		for (const auto& e : *f) {
			glm::vec3 v0 = e->prev->vert->attr.pos;
			glm::vec3 v1 = e->vert->attr.pos;
			glm::vec3 v2 = e->next->vert->attr.pos;

			glm::vec2 uv0 = e->prev->vert->attr.uv;
			glm::vec2 uv1 = e->vert->attr.uv;
			glm::vec2 uv2 = e->next->vert->attr.uv;

			// Calculate triangle normal
			e->vert->attr.normal = glm::triangleNormal(v0, v1, v2);

			// Edges of the triangle : position delta
			glm::vec3 deltaPos1 = v1 - v0;
			glm::vec3 deltaPos2 = v2 - v0;

			// UV delta
			glm::vec2 deltaUV1 = uv1 - uv0;
			glm::vec2 deltaUV2 = uv2 - uv0;

			float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
			e->vert->attr.tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r;
			e->vert->attr.bitangent = (deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x) * r;
		}
	}

	return 0;
}

/// <summary>
/// Loop through the edges and generate indices for rendering
/// </summary>
/// <returns></returns>
std::vector<uint32_t> dcel::manifold::generate_edge_indices()
{
	std::vector<uint32_t> indices;
	for (face_ptr f : _faces)
	{
		if (f->vertexCount == 2)
		{
			edge_ptr e = f->head;
			indices.push_back(e->vert->idx);
			indices.push_back(e->next->vert->idx);
		}
		else 
		{
			edge_ptr e = f->head;
			do {
				indices.push_back(e->vert->idx);
				indices.push_back(e->next->vert->idx);
				e = e->next;
			} while (e != f->head);
		}
	}

	return indices;
}

void dcel::manifold::rotate_triangle_x_plane(glm::vec3& a, glm::vec3& b, glm::vec3& c)
{
	/*glm::vec3 v0 = a - b;
	glm::vec3 v1 = c - b;

	glm::vec3 v = glm::cross(v0, v1);
	float length = glm::length(v);

	// Directional cosine
	float alpha = std::acos(v.x / length);
	float beta = std::acos(v.y / length);
	float omega = std::acos(v.z / length);

	// Create rotation vector
	glm::mat4 rot = glm::eulerAngleXYZ(alpha, beta, omega);

	// Rotate
	a = rot * glm::vec4(a, 1.0f);
	b = rot * glm::vec4(b, 1.0f);
	c = rot * glm::vec4(c, 1.0f);

	v0 = a - b;
	v1 = c - b;

	float angle = std::atan2(v0.z - v1.z, v0.y - v1.y) * 180 / glm::pi<float>();*/
}

glm::vec3 dcel::manifold::plane_projection(const glm::vec3& target, const glm::vec3& normal)
{
	return target - ((glm::dot(target, normal) / glm::length2(normal)) * normal);
}

plane_ptr dcel::manifold::best_plane_from_points(const face_ptr& face)
{
	plane_ptr p = std::make_shared<plane>();
	
	// Get vertices
	std::vector<Eigen::Vector3f> c;
	edge_ptr e = face->head;
	do {
		c.push_back({ e->vert->attr.pos.x, e->vert->attr.pos.y, e->vert->attr.pos.z });
		e = e->next;
	} while (e != face->head);

	// copy coordinates to  matrix in Eigen format
	size_t num_atoms = c.size();
	Eigen::Matrix< Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
	for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

	// calculate centroid
	Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

	// subtract centroid
	coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

	// we only need the left-singular matrix here
	auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Vector3f plane_normal = svd.matrixU().rightCols(1);

	// Fill plane structure
	p->centroid.x = centroid.x();
	p->centroid.y = centroid.y();
	p->centroid.z = centroid.z();
	p->normal.x = plane_normal.x();
	p->normal.y = plane_normal.y();
	p->normal.z = plane_normal.z();

	return p;
}

/// <summary>
/// Triangulate the face and generate indices for rendering
/// Only works for faces that have 3 or more vertices
/// </summary>
/// <returns></returns>
std::vector<uint32_t> dcel::manifold::generate_face_indices()
{
	std::vector<uint32_t> indices;
	for (face_ptr f : _faces)
	{
		if (f->vertexCount < 3)
			continue;

		// Store possible triangles
		std::list<edge_ptr> tri;
		edge_ptr e = f->head;
		do {
			tri.push_back(e);
			e = e->next;
		} while (e != f->head);

		// The first vertex indicates the winding order
		auto normal = f->head->vert->attr.normal;

		// Ear clipping in 3d
		auto it = tri.begin();
		while(tri.size() > 3)
		{
			// Get a copy of the current iterator
			auto ik = it;

			// Start index of triangle
			auto ia = ik;

			// Second index of triangle
			if (++ik == tri.end()) ik = tri.begin(); auto ib = ik;

			// Third index of triangle
			if (++ik == tri.end()) ik = tri.begin(); auto ic = ik;

			// Starting point of point in triangle test
			if (++ik == tri.end()) ik = tri.begin(); auto ij = ik;

			// Construct the triangle
			glm::vec3 a = (*ia)->vert->attr.pos;
			glm::vec3 b = (*ib)->vert->attr.pos;
			glm::vec3 c = (*ic)->vert->attr.pos;

			// Condition #1 - Angle must be less than 180
			glm::vec3 v0 = a - b;
			glm::vec3 v1 = c - b;

			float angle = 180 + std::atan2(glm::dot(glm::cross(v0, v1), normal), glm::dot(v0, v1)) * 180 / glm::pi<float>();

			if (angle > 180) {
				if (++it == tri.end()) it = tri.begin(); continue;
			}

			// Condition #2 - No points can lie on triangle
			bool pit = false;
			while (ij != it)
			{
				glm::vec3 p = (*ij)->vert->attr.pos;
				pit = point_in_triangle(p, a, b, c);	
				if (++ij == tri.end()) ij = tri.begin();
				if (pit) break;
			}

			// If point found in triangle then we don't have an ear
			if (pit) {
				if (++it == tri.end()) it = tri.begin();
				continue;
			}

			// Store indices
			indices.push_back((*it)->vert->idx);
			indices.push_back((*ib)->vert->idx);
			indices.push_back((*ic)->vert->idx);

			// Remove
			tri.erase(ib);
		}

		// Store last triangle
		for (auto& t : tri) {
			indices.push_back(t->vert->idx);
		}
	}

	return indices;
}

/// <summary>
/// Generate vertices
/// </summary>
/// <returns></returns>
std::vector<vertex_attr> dcel::manifold::generate_vertices()
{
	std::vector<vertex_attr> verts;
	for (auto v : _vertices) {
		verts.push_back(v->attr);
	}

	return verts;
}

edge_ptr dcel::manifold::construct_edge(vertex_ptr vert, face_ptr face)
{
	// Make a new edge
	auto newEdge = std::make_shared<edge>();
	newEdge->face = face;

	// Set the head edge for this face
	if (not face->head) {
		face->head = newEdge;
	}

	// If vertex belongs to a different face already
	if (vert->face) {
		size_t newIdx = insert_vertex(vert->attr.pos);
		auto newVert = _vertices.at(newIdx);
		newVert->parent = vert;
		newVert->face = face;
	}

	// Set the vertex face
	vert->face = face;

	newEdge->vert = vert;
	vert->edges.push_back(newEdge);
	if (not vert->edge) {
		vert->edge = newEdge;
	}

	return newEdge;
}

/// <summary>
/// Insert a face from an ordered list of vertex indices
/// </summary>
/// <param name="v"></param>
/// <returns></returns>
std::size_t dcel::manifold::insert_face(const std::vector<std::size_t>& v)
{
	// Make a new face structure
	auto f = std::make_shared<face>();

	// Store vertex size
	size_t vertexSize = v.size();

	// Set the vertex count
	f->vertexCount = vertexSize;

	// Iterate over the vertex indices
	std::size_t prevIdx = 0;
	edge_ptr prevEdge;
	for (const auto& idx : v) {
		// Get the vertex by index
		vertex_ptr vert = _vertices.at(idx);
		auto newEdge = construct_edge(vert, f);

		// If there's a previous edge
		if (prevEdge) {
			// Update the previous edge's successor
			prevEdge->next = newEdge;
			vert->edges.push_back(prevEdge);

			// Try to find a pair for prev edge using this edge's index
			auto pair = find_edge(idx, prevIdx);
			if (pair) {
				if (pair->pair) {
					auto msg = "Resolved edge pair already paired. Edge (" +
						std::to_string(prevIdx) + ", " +
						std::to_string(idx) + ") is not 2-manifold.";
						throw std::runtime_error(msg.c_str());
				}
				prevEdge->pair = pair;
				pair->pair = prevEdge;
			}
		}

		// Store the edge
		newEdge->idx = _edges.size();
		_edges.emplace(idx, newEdge);

		// Update for the next iteration
		prevIdx = idx;
		prevEdge = newEdge;
	}

	// Link back to the beginning
	prevEdge->next = f->head;
	f->head->vert->edges.push_back(prevEdge);

	// Try to find a pair for final edge using this edge's index
	auto pair = find_edge(f->head->vert->idx, prevIdx);
	if (pair) {
		if (pair->pair) {
			auto msg = "Resolved edge pair already paired. Edge (" +
				std::to_string(prevIdx) + ", " +
				std::to_string(f->head->vert->idx) +
				") is not 2-manifold.";
			throw std::runtime_error(msg.c_str());
		}
		prevEdge->pair = pair;
		pair->pair = prevEdge;
	}

	// Sanity check: edge lengths
	for (const auto& e : *f) {
		if (glm::length(e->next->vert->attr.pos - e->vert->attr.pos) == 0.0) {
			auto msg = "Zero-length edge (" +
				std::to_string(e->vert->idx) + ", " +
				std::to_string(e->next->vert->idx) + ")";
			throw std::runtime_error(msg.c_str());
		}
	}

	// Compute angles for edges in face
	compute_face_angles(f);

	// Calculate vertex normals
	calculate_normals(f);

	// Give this face an idx and link the previous face with this one
	f->idx = _faces.size();
	if (not _faces.empty()) {
		_faces.back()->next = f;
	}
	_faces.emplace_back(f);

	return f->idx;
}

/// <summary>
/// Insert vertices into list
/// </summary>
/// <param name="pos"></param>
/// <returns></returns>
std::size_t dcel::manifold::insert_vertex(glm::vec3 pos)
{
	auto vert = std::make_shared<vertex>();
	vert->attr.pos = pos;
	vert->idx = _vertices.size();
	_vertices.push_back(vert);
	return vert->idx;
}

/// <summary>
/// 
/// </summary>
/// <returns></returns>
std::vector<vertex_ptr> dcel::manifold::vertices_interior()
{
	std::vector<vertex_ptr> ret;
	std::copy_if(_vertices.begin(), _vertices.end(), std::back_inserter(ret),
		[](auto x) {return not x->is_boundary(); });

	return ret;
}

/// <summary>
/// 
/// </summary>
/// <returns></returns>
std::vector<vertex_ptr> dcel::manifold::vertices_boundary()
{
	std::vector<vertex_ptr> ret;
	std::copy_if(
		_vertices.begin(), _vertices.end(), std::back_inserter(ret),
		[](auto x) { return x->is_boundary(); });
	return ret;
}

/// <summary>
/// 
/// </summary>
/// <returns></returns>
const std::vector<edge_ptr> dcel::manifold::edges()
{
	std::vector<edge_ptr> edges;
	for (const auto& f : _faces) {
		for (const auto& e : *f) {
			edges.emplace_back(e);
		}
	}
	return edges;
}

/// <summary>
/// Get the faces in insertion order
/// </summary>
/// <returns></returns>
const std::vector<face_ptr> dcel::manifold::faces()
{
	return _faces;
}

/// <summary>
/// Get the vertices in insertion order
/// </summary>
/// <returns></returns>
const std::vector<vertex_ptr> dcel::manifold::vertices()
{
	return _vertices;
}

/// <summary>
/// Get the number of edges
/// </summary>
/// <returns></returns>
const std::size_t dcel::manifold::num_edges()
{
	return _edges.size();
}

/// <summary>
/// Get the number of faces
/// </summary>
/// <returns></returns>
const std::size_t dcel::manifold::num_faces()
{
	return _faces.size();
}

/// <summary>
/// Get the number of vertices
/// </summary>
/// <returns></returns>
const std::size_t dcel::manifold::num_vertices()
{
	return _vertices.size();
}

/// <summary>
/// Get the number of interior vertices
/// </summary>
/// <returns></returns>
const std::size_t dcel::manifold::num_interior_vertices()
{
	return std::accumulate(
		_vertices.begin(), _vertices.end(), std::size_t{ 0 }, [](auto a, auto b) {
			return a + static_cast<std::size_t>(not b->is_boundary());
		});
}

/// <summary>
/// 
/// </summary>
/// <param name="face"></param>
/// <returns></returns>
bool dcel::manifold::coplanar(size_t idx)
{
	face_ptr f = _faces[idx];

	// Take the cross product of the first 3 vertices
	auto v0 = f->head->next->vert->attr.pos;
	auto v1 = f->head->vert->attr.pos;
	auto v2 = f->head->next->next->vert->attr.pos;
	auto n = glm::cross(v1 - v0, v2 - v0);

	// Check against additional vertices
	float r = 0;
	edge_ptr e = f->head->next->next->next;
	do {
		r = glm::dot(e->vert->attr.pos - v0, n);
		e = e->next;
	} while (e != f->head);

	return r == 0;
}

/// <summary>
/// 
/// </summary>
void dcel::manifold::compute_abfpp()
{
	Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver;
	using Triplet = Eigen::Triplet<float>;
	using SparseMatrix = Eigen::SparseMatrix<float>;
	using DenseVector = Eigen::Matrix<float, Eigen::Dynamic, 1>;

	std::size_t maxIters = 10;

	// Calculate angles and weights
	initialize_angles_weights();

	// 
	float grad = gradient();
	if (std::isnan(grad) or std::isinf(grad)) {
		// handle exception here
	}

	float gradDelta = std::numeric_limits<float>::infinity();
	std::size_t iters = 0;

	auto vIntCnt = num_interior_vertices();
	auto edgeCnt = num_edges();
	auto faceCnt = num_faces();

	while (grad > 0.001 and gradDelta > 0.001 and iters < maxIters)
	{
		if (std::isnan(grad) or std::isinf(grad)) {
			// handle exception here
		}

		// b1 = -alpha gradient
		std::vector<Triplet> triplets;
		std::size_t idx = 0;
		for (const auto& e : edges()) {
			triplets.emplace_back(idx, 0, -alpha_gradient(e));
			++idx;
		}
		SparseMatrix b1(edgeCnt, 1);
		b1.reserve(triplets.size());
		b1.setFromTriplets(triplets.begin(), triplets.end());

		// b2 = -lambda gradient
		triplets.clear();
		idx = 0;

		// lambda tri
		for (const auto& f : _faces) {
			triplets.emplace_back(idx, 0, -triangle_gradient(f));
			idx++;
		}

		// lambda plan and lambda len
		for (const auto& v : vertices_interior()) {
			triplets.emplace_back(idx, 0, -planar_gradient(v));
			triplets.emplace_back(vIntCnt + idx, 0, -length_gradient(v));
			idx++;
		}
		SparseMatrix b2(faceCnt + 2 * vIntCnt, 1);
		b2.reserve(triplets.size());
		b2.setFromTriplets(triplets.begin(), triplets.end());

		// vertex idx -> interior vertex idx permutation
		std::map<std::size_t, std::size_t> vIdx2vIntIdx;
		std::size_t newIdx = 0;
		for (const auto& v : vertices_interior()) {
			vIdx2vIntIdx[v->idx] = newIdx++;
		}

		// Compute J1 + J2
		triplets.clear();
		idx = 0;

		// Jacobian of the CTri constraints
		for (; idx < faceCnt; idx++) {
			triplets.emplace_back(idx, 3 * idx, 1);
			triplets.emplace_back(idx, 3 * idx + 1, 1);
			triplets.emplace_back(idx, 3 * idx + 2, 1);
		}

		for (const auto& v : vertices_interior()) {
			for (const auto& e0 : v->wheel()) {
				// Jacobian of the CPlan constraint
				triplets.emplace_back(idx, e0->idx, 1);

				// Jacobian of the CLen constraint
				auto e1 = e0->next;
				auto e2 = e1->next;
				auto d1 = length_gradient(v, e1);
				auto d2 = length_gradient(v, e2);
				triplets.emplace_back(vIntCnt + idx, e1->idx, d1);
				triplets.emplace_back(vIntCnt + idx, e2->idx, d2);
			}
			++idx;
		}

		// Construct the Sparse Matrix
		SparseMatrix J(faceCnt + 2 * vIntCnt, 3 * faceCnt);
		J.reserve(triplets.size());
		J.setFromTriplets(triplets.begin(), triplets.end());

		// Lambda = diag(2/w)
		// v.weight == 1/w, so LambdaInv is diag(2*weight)
		// We only need Lambda Inverse, so this is 1 / 2*weight
		triplets.clear();
		idx = 0;
		for (const auto& e : edges()) {
			triplets.emplace_back(idx, idx, 1.0f / (2.0f * e->weight));
			++idx;
		}
		SparseMatrix LambdaInv(edgeCnt, edgeCnt);
		LambdaInv.reserve(edgeCnt);
		LambdaInv.setFromTriplets(triplets.begin(), triplets.end());

		// solve Eq. 16
		auto bstar = J * LambdaInv * b1 - b2;
		auto JLiJt = J * LambdaInv * J.transpose();

		SparseMatrix LambdaStarInv = JLiJt.block(0, 0, faceCnt, faceCnt);
		for (int k = 0; k < LambdaStarInv.outerSize(); ++k) {
			for (typename SparseMatrix::InnerIterator it(LambdaStarInv, k);
				it; ++it) {
				it.valueRef() = 1.F / it.value();
			}
		}
		auto Jstar = JLiJt.block(faceCnt, 0, 2 * vIntCnt, faceCnt);
		auto JstarT = JLiJt.block(0, faceCnt, faceCnt, 2 * vIntCnt);
		auto Jstar2 = JLiJt.block(faceCnt, faceCnt, 2 * vIntCnt, 2 * vIntCnt);
		auto bstar1 = bstar.block(0, 0, faceCnt, 1);
		auto bstar2 = bstar.block(faceCnt, 0, 2 * vIntCnt, 1);

		// (J* Lam*^-1 J*^t - J**) delta_lambda_2 = J* Lam*^-1 b*_1 - b*_2
		SparseMatrix A = Jstar * LambdaStarInv * JstarT - Jstar2;
		SparseMatrix b = Jstar * LambdaStarInv * bstar1 - bstar2;
		A.makeCompressed();

		solver.compute(A);
		if (solver.info() != Eigen::ComputationInfo::Success) {
			//throw SolverException(solver.lastErrorMessage());
		}
		auto deltaLambda2 = solver.solve(b);
		if (solver.info() != Eigen::ComputationInfo::Success) {
			//throw SolverException(solver.lastErrorMessage());
		}

		// Compute Eq. 17 -> delta_lambda_1
		auto deltaLambda1 =
			LambdaStarInv * (bstar1 - JstarT * deltaLambda2);

		// Construct deltaLambda
		DenseVector deltaLambda(
			deltaLambda1.rows() + deltaLambda2.rows(), 1);
		deltaLambda << DenseVector(deltaLambda1), DenseVector(deltaLambda2);

		// Compute Eq. 10 -> delta_alpha
		DenseVector deltaAlpha =
			LambdaInv * (b1 - J.transpose() * deltaLambda);

		// lambda += delta_lambda
		for (auto& f : _faces) {
			f->lambda_tri += deltaLambda(f->idx, 0);
		}
		for (auto& v : vertices_interior()) {
			auto intIdx = vIdx2vIntIdx.at(v->idx);
			v->lambda_plan += deltaLambda(faceCnt + intIdx, 0);
			v->lambda_len += deltaLambda(faceCnt + vIntCnt + intIdx, 0);
		}

		// alpha += delta_alpha
		// Update sin and cos
		idx = 0;
		for (auto& e : edges()) {
			e->alpha += deltaAlpha(idx++, 0);
			e->alpha = std::min(std::max(e->alpha, 0.0f), glm::pi<float>());
			e->alpha_sin = std::sin(e->alpha);
			e->alpha_cos = std::cos(e->alpha);
		}

		// Recalculate gradient for next iteration
		auto newGrad = gradient();
		gradDelta = std::abs(newGrad - grad);
		grad = newGrad;
		iters++;
	}
}

/// <summary>
/// 
/// </summary>
void dcel::manifold::compute_lscm()
{
	Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver;
	using Triplet = Eigen::Triplet<float>;
	using SparseMatrix = Eigen::SparseMatrix<float>;
	using DenseMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

	// Pinned vertex selection
	// Get the end points of a boundary edge
	auto p0 = vertices_boundary()[0];
	auto e = p0->edge;
	do {
		if (not e->pair) {
			break;
		}
		e = e->pair->next;
	} while (e != p0->edge);
	if (e == p0->edge and e->pair) {
		//throw MeshException("Pinned vertex not on boundary");
	}
	auto p1 = e->next->vert;

	// Map selected edge to closest XY axis
	// Use sign to select direction
	auto pinVec = p1->attr.pos - p0->attr.pos;
	auto dist = glm::distance(p1->attr.pos, p0->attr.pos);
	pinVec /= dist;

	p0->attr.pos = { 0.0f, 0.0f, 0.0f };
	auto maxElem = 0; //std::max_element(pinVec.begin(), pinVec.end());
	auto maxAxis = 0; //std::distance(pinVec.begin(), maxElem);
	dist = 0; // std::copysign(dist, *maxElem);
	if (maxAxis == 0) {
		p1->attr.pos = { dist, 0.0f, 0.0f };
	}
	else {
		p1->attr.pos = { 0.0f, dist, 0.0f };
	}

	// For convenience
	auto numFaces = num_faces();
	auto numVerts = num_vertices();
	auto numFixed = 2;
	auto numFree = numVerts - numFixed;

	// Permutation for free vertices
	// This helps us find a vert's row in the solution matrix
	std::map<std::size_t, std::size_t> freeIdxTable;
	for (const auto& v : vertices()) {
		if (v == p0 or v == p1) {
			continue;
		}
		auto newIdx = freeIdxTable.size();
		freeIdxTable[v->idx] = newIdx;
	}

	// Setup pinned bFixed
	std::vector<Triplet> tripletsB;
	tripletsB.emplace_back(0, 0, p0->attr.pos[0]);
	tripletsB.emplace_back(1, 0, p0->attr.pos[1]);
	tripletsB.emplace_back(2, 0, p1->attr.pos[0]);
	tripletsB.emplace_back(3, 0, p1->attr.pos[1]);
	SparseMatrix bFixed(2 * numFixed, 1);
	bFixed.reserve(tripletsB.size());
	bFixed.setFromTriplets(tripletsB.begin(), tripletsB.end());

	// Setup variables matrix
	// Are only solving for free vertices, so push pins in special matrix
	std::vector<Triplet> tripletsA;
	tripletsB.clear();
	for (const auto& f : faces()) {
		auto e0 = f->head;
		auto e1 = e0->next;
		auto e2 = e1->next;
		auto sin0 = std::sin(e0->alpha);
		auto sin1 = std::sin(e1->alpha);
		auto sin2 = std::sin(e2->alpha);

		// Find the max sin idx
		std::vector<float> sins{ sin0, sin1, sin2 };
		auto sinMaxElem = std::max_element(sins.begin(), sins.end());
		auto sinMaxIdx = std::distance(sins.begin(), sinMaxElem);

		// Rotate the edge order of the face so last angle is largest
		if (sinMaxIdx == 0) {
			auto temp = e0;
			e0 = e1;
			e1 = e2;
			e2 = temp;
			sin0 = sins[1];
			sin1 = sins[2];
			sin2 = sins[0];
		}
		else if (sinMaxIdx == 1) {
			auto temp = e2;
			e2 = e1;
			e1 = e0;
			e0 = temp;
			sin0 = sins[2];
			sin1 = sins[0];
			sin2 = sins[1];
		}

		auto ratio = (sin2 == 0.0f) ? 1.0f : sin1 / sin2;
		auto cosine = std::cos(e0->alpha) * ratio;
		auto sine = sin0 * ratio;

		// If pin0 or pin1, put in fixedB matrix, else put in A
		auto row = 2 * f->idx;
		if (e0->vert == p0) {
			tripletsB.emplace_back(row, 0, cosine - 1.0f);
			tripletsB.emplace_back(row, 1, -sine);
			tripletsB.emplace_back(row + 1, 0, sine);
			tripletsB.emplace_back(row + 1, 1, cosine - 1.0f);
		}
		else if (e0->vert == p1) {
			tripletsB.emplace_back(row, 2, cosine - 1.0f);
			tripletsB.emplace_back(row, 3, -sine);
			tripletsB.emplace_back(row + 1, 2, sine);
			tripletsB.emplace_back(row + 1, 3, cosine - 1.0f);
		}
		else {
			auto freeIdx = freeIdxTable.at(e0->vert->idx);
			tripletsA.emplace_back(row, 2 * freeIdx, cosine - 1.0f);
			tripletsA.emplace_back(row, 2 * freeIdx + 1, -sine);
			tripletsA.emplace_back(row + 1, 2 * freeIdx, sine);
			tripletsA.emplace_back(row + 1, 2 * freeIdx + 1, cosine - 1.0f);
		}

		if (e1->vert == p0) {
			tripletsB.emplace_back(row, 0, -cosine);
			tripletsB.emplace_back(row, 1, sine);
			tripletsB.emplace_back(row + 1, 0, -sine);
			tripletsB.emplace_back(row + 1, 1, -cosine);
		}
		else if (e1->vert == p1) {
			tripletsB.emplace_back(row, 2, -cosine);
			tripletsB.emplace_back(row, 3, sine);
			tripletsB.emplace_back(row + 1, 2, -sine);
			tripletsB.emplace_back(row + 1, 3, -cosine);
		}
		else {
			auto freeIdx = freeIdxTable.at(e1->vert->idx);
			tripletsA.emplace_back(row, 2 * freeIdx, -cosine);
			tripletsA.emplace_back(row, 2 * freeIdx + 1, sine);
			tripletsA.emplace_back(row + 1, 2 * freeIdx, -sine);
			tripletsA.emplace_back(row + 1, 2 * freeIdx + 1, -cosine);
		}

		if (e2->vert == p0) {
			tripletsB.emplace_back(row, 0, 1.0f);
			tripletsB.emplace_back(row + 1, 1, 1.0f);
		}
		else if (e2->vert == p1) {
			tripletsB.emplace_back(row, 2, 1.0f);
			tripletsB.emplace_back(row + 1, 3, 1.0f);
		}
		else {
			auto freeIdx = freeIdxTable.at(e2->vert->idx);
			tripletsA.emplace_back(row, 2 * freeIdx, 1.0f);
			tripletsA.emplace_back(row + 1, 2 * freeIdx + 1, 1.0f);
		}
	}
	SparseMatrix A(2 * numFaces, 2 * numFree);
	A.reserve(tripletsA.size());
	A.setFromTriplets(tripletsA.begin(), tripletsA.end());

	SparseMatrix bFree(2 * numFaces, 2 * numFixed);
	bFree.reserve(tripletsB.size());
	bFree.setFromTriplets(tripletsB.begin(), tripletsB.end());

	// Calculate rhs from free and fixed matrices
	SparseMatrix b = bFree * bFixed * -1;

	// Setup AtA and solver
	SparseMatrix AtA = A.transpose() * A;
	AtA.makeCompressed();
	solver.compute(AtA);
	if (solver.info() != Eigen::ComputationInfo::Success) {
		//throw SolverException(solver.lastErrorMessage());
	}

	// Setup Atb
	SparseMatrix Atb = A.transpose() * b;

	// Solve AtAx = AtAb
	DenseMatrix x = solver.solve(Atb);

	// Assign solution to UV coordinates
	// Pins are already updated, so these are free vertices
	for (const auto& v : vertices()) {
		if (v == p0 or v == p1) {
			continue;
		}

		auto newIdx = 2 * freeIdxTable.at(v->idx);
		v->attr.pos[0] = x(newIdx, 0);
		v->attr.pos[1] = x(newIdx + 1, 0);
		v->attr.pos[2] = 0.0f;
	}
}

/// <summary>
/// Compute internal angles on a face
/// </summary>
/// <param name="face"></param>
void dcel::manifold::compute_face_angles(face_ptr face)
{
	for (auto& e : *face) {
		glm::vec3 ab = e->next->vert->attr.pos - e->vert->attr.pos;
		glm::vec3 ac = e->next->next->vert->attr.pos - e->vert->attr.pos;

		e->alpha = interior_angle(ab, ac);

		if (std::isnan(e->alpha) or std::isinf(e->alpha)) {
			// catch exception here
		}
	}
}

/// <summary>
/// 
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
float dcel::manifold::interior_angle(const glm::vec3& a, const glm::vec3& b)
{
	float res = std::acos(glm::dot(a, b) / (glm::length(a) * glm::length(b)));
	return res;
}

/// <summary>
/// Initialize the ABF angles and weights from the edge alpha values
/// [Sheffer and de Sturler 2001]
/// </summary>
void dcel::manifold::initialize_angles_weights()
{
	static constexpr float minAngle = glm::pi<float>() / 180.0f;
	static constexpr float maxAngle = glm::pi<float>() - minAngle;

	// Initialize and bound angle properties
	for (edge_ptr e : edges()) {
		e->alpha = e->beta = e->phi =
			std::min(std::max(e->alpha, minAngle), maxAngle);

		// Pre-calculations since they occur often
		e->alpha_sin = std::sin(e->alpha);
		e->alpha_cos = std::cos(e->alpha);

		// Initial choice for the weights. w = (phi)^-2
		e->weight = 1.0f / (e->phi * e->phi);
	}

	// Update weights for interior vertices
	for (auto& v : vertices_interior()) {
		auto wheel = v->wheel();
		auto angle_sum = std::accumulate(
			wheel.begin(), wheel.end(), 0.0f,
			[](auto a, auto b) { return a + b->beta; });

		for (auto& e : wheel) {
			e->phi *= 2 * glm::pi<float>() / angle_sum;
			e->weight = 1.0f / (e->phi * e->phi);
		}
	}
}

/// <summary>
/// Gradient 
/// </summary>
/// <returns></returns>
float dcel::manifold::gradient()
{
	float g = 0;
	for (const auto& f : _faces) {
		// AlphaGrad for all edges
		for (const auto& e : *f) {
			auto gAlpha = alpha_gradient(e);
			g += gAlpha * gAlpha;
		}
		// TriGrad for all faces
		auto gTri = triangle_gradient(f);
		g += gTri * gTri;
	}

	// PlanGrad and LenGrad for all interior vertices
	for (const auto& v : vertices_interior()) {
		auto gPlan = planar_gradient(v);
		g += gPlan * gPlan;

		auto gLen = length_gradient(v);
		g += gLen * gLen;
	}
	return g;
}

/// <summary>
/// One of the constraints used for triangle validatiy
/// CTri(t) = (a1)^t + (a2)^t + (a3)^t - Phi = 0
/// </summary>
/// <param name="f">Pointer to a face</param>
/// <returns></returns>
float dcel::manifold::triangle_gradient(const face_ptr& f)
{
	float g = -glm::pi<float>();
	for (const auto& e : *f) {
		g += e->alpha;
	}
	return g;
}

/// <summary>
/// One of the constraints used for planar validatiy
/// </summary>
/// <param name="v"></param>
/// <returns></returns>
float dcel::manifold::planar_gradient(const vertex_ptr& v)
{
	auto edges = v->wheel();
	float g = -2 * glm::pi<float>();
	for (const auto& e : v->wheel()) {
		g += e->alpha;
	}
	return g;
}

/// <summary>
/// This constraint ensures that edges shared by pairs of triangles have the same length.
/// </summary>
/// <param name="v"></param>
/// <returns></returns>
float dcel::manifold::length_gradient(const vertex_ptr& v)
{
	float p1 = 0.0f;
	float p2 = 0.0f;

	for (const auto& e : v->wheel()) {
		p1 *= e->next->alpha_sin;
		p2 *= e->next->next->alpha_sin;
	}
	return p1 - p2;
}

/// <summary>
/// 
/// </summary>
/// <param name="v"></param>
/// <param name="edge"></param>
/// <returns></returns>
float dcel::manifold::length_gradient(const vertex_ptr& v, const edge_ptr& edge)
{
	float p1 = 1.0f;
	float p2 = 1.0f;

	for (const auto& a : v->wheel()) {
		auto b = a->next;
		if (b == edge) {
			p1 *= b->alpha_cos;
			p2 = 0.0f;
		}
		else {
			p1 *= b->alpha_sin;
		}

		auto c = a->next->next;
		if (c == edge) {
			p1 = 0.0f;
			p2 *= c->alpha_cos;
		}
		else {
			p2 *= c->alpha_sin;
		}
	}

	return p1 - p2;
}

/// <summary>
/// 
/// </summary>
/// <param name="e"></param>
/// <returns></returns>
float dcel::manifold::alpha_gradient(const edge_ptr& e)
{
	auto g = (e->alpha - e->phi) * e->weight;
	g += e->face->lambda_tri;
	for (const auto& eg : *e->face) {
		// Skip boundary vertices
		if (eg->vert->is_boundary()) {
			continue;
		}
		if (eg == e) {
			g += eg->vert->lambda_plan;
		}
		else {
			auto d = length_gradient(eg->vert, e);
			d *= eg->vert->lambda_len;
			g += d;
		}
	}
	return g;
}

/// <summary>
/// 
/// </summary>
/// <param name="start"></param>
/// <param name="end"></param>
/// <returns></returns>
edge_ptr dcel::manifold::find_edge(std::size_t start, std::size_t end)
{
	// Get edges with this start index
	auto range = _edges.equal_range(start);

	// Loop over potential edges
	for (auto it = range.first; it != range.second; it++) {
		const auto& e = it->second;

		if (e->next and e->next->vert->idx == end) {
			return e;
		}
	}

	return nullptr;
}

std::string dcel::manifold::print_edges(face_ptr face)
{
	std::string s;

	edge_ptr edge = face->head;
	do {
		glm::vec3 p1 = edge->prev->vert->attr.pos;
		glm::vec3 p2 = edge->vert->attr.pos;

		s.append(glm::to_string(p1) + "\t" + glm::to_string(p2) + "\n");

		edge = edge->next;
	} while (edge != face->head);

	return s;
}

std::string dcel::manifold::print_vertices(face_ptr face)
{
	std::string s;
	edge_ptr edge = face->head;

	do {
		glm::vec3 p1 = edge->prev->vert->attr.pos;

		s.append(glm::to_string(p1) + "\n");

		edge = edge->next;
	} while (edge != face->head);

	return s;
}

bool dcel::manifold::point_in_triangle(const glm::vec3 p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
	// Compute vectors        
	glm::vec3 v0 = a - b;
	glm::vec3 v1 = c - b;
	glm::vec3 v2 = p - b;

	// Compute dot products
	float dot00 = glm::dot(v0, v0);
	float dot01 = glm::dot(v0, v1);
	float dot02 = glm::dot(v0, v2);
	float dot11 = glm::dot(v1, v1);
	float dot12 = glm::dot(v1, v2);

	// Compute barycentric coordinates
	float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return (u >= 0) && (v >= 0) && (u + v < 1);
}

bool dcel::manifold::is_ccw(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
	// calculate three axes
	glm::vec3 axis_x = glm::normalize(b - a);
	glm::vec3 axis_y = glm::normalize(c - a);
	glm::vec3 axis_z = glm::cross(axis_x, axis_y);

	// construct a transform matrix from our axes
	glm::mat3x3 object_transform;
	object_transform[0] = axis_x;
	object_transform[1] = axis_y;
	object_transform[2] = axis_z;

	// invert the matrix
	glm::mat3x3 object_to_world_transform = glm::inverse(object_transform);

	// transform the outward normal using the matrix
	glm::vec3 normal = object_to_world_transform * axis_z;

	return normal.z > 0.0f; // counter-clockwise winding
}

/* REFERENCES
SHEFFER, A. AND DE STURLER, E. 2001. Parameterization of faceted surfaces for meshing using angle based
	flattening.Engineering with Computers 17, 326–337.

 */