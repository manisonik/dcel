#pragma once
#include "dcel.h"

namespace dcel
{
	class manifold
	{
	public:
		manifold() = default;
		~manifold();

		using Pointer = std::shared_ptr<manifold>;

		vertex_ptr add_vertex(glm::vec3 pos);

		face_ptr add_face(const std::vector<vertex_ptr>& verts);

		void pair_edges(face_ptr fb);
		int print_build();
		int build();

		/* Generate edge indices from the manifold*/
		std::vector<uint32_t> generate_edge_indices();

		/* Rotate triangle onto z-plane */
		void rotate_triangle_x_plane(glm::vec3& a, glm::vec3& b, glm::vec3& c);

		/* Project point to plane */
		glm::vec3 plane_projection(const glm::vec3& target, const glm::vec3& normal);

		/* Determine the best-fit plane given a set points */
		plane_ptr best_plane_from_points(const face_ptr& face);

		/* Generate vertex indices from the manifold */
		std::vector<uint32_t> generate_face_indices();

		/* Generate vertices from the manifold*/
		std::vector<vertex_attr> generate_vertices();

		/* Contruct edge from vertex and face */
		edge_ptr construct_edge(vertex_ptr vert, face_ptr face);

		/* Calculate vertex normals from face */
		void calculate_normals(face_ptr f);

		/* Calculate tangent basis */
		void calculate_tangent_basis();

		/* Insert face */
		std::size_t insert_face(const std::vector<std::size_t>& v);

		/* Insert vertices */
		std::size_t insert_vertex(glm::vec3 pos);

		/* Get the list of interior vertices in insertion order */
		std::vector<vertex_ptr> vertices_interior();

		/* Get the list of boundary vertices in insertion order */
		std::vector<vertex_ptr> vertices_boundary();

		/* Get the list of edges in insertion order */
		const std::vector<edge_ptr> edges();

		/* Get the list of faces in insertion order */
		const std::vector<face_ptr> faces();

		/* Get the list of vertices in insertion order */
		const std::vector<vertex_ptr> vertices();

		/* Get the number of edges */
		const std::size_t num_edges();

		/* Get the number of faces */
		const std::size_t num_faces();

		/* Get the number of vertices */
		const std::size_t num_vertices();

		/* Get the number of interior vertices */
		const std::size_t num_interior_vertices();

		/* Check for coplanarity */
		bool coplanar(size_t idx);

		// Compute ABF++
		void compute_abfpp();

		// Compute LSCM
		void compute_lscm();

		/* Computer face angles */
		void compute_face_angles(face_ptr face);

		/* Compute interior angles */
		float interior_angle(const glm::vec3& a, const glm::vec3& b);

		/* Calculate the angle weights */
		void initialize_angles_weights();

		/* Gradient */
		float gradient();

		/* Triangle gradient */
		float triangle_gradient(const face_ptr& f);

		/* Planar gradient */
		float planar_gradient(const vertex_ptr& v);

		/* Length gradient */
		float length_gradient(const vertex_ptr& v);

		/* Length gradient */
		float length_gradient(const vertex_ptr& v, const edge_ptr& edge);

		/* Alpha gradient */
		float alpha_gradient(const edge_ptr& e);

		/* Find an existing edge with the provided points */
		edge_ptr find_edge(std::size_t start, std::size_t end);

		/* Print edges */
		std::string print_edges(face_ptr face);

		/* Print vertices*/
		std::string print_vertices(face_ptr face);

		/* Point in triangle test */
		bool point_in_triangle(const glm::vec3 p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

		/* Check winding order */
		bool is_ccw(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

	private:
		std::vector<vertex_ptr> _vertices;
		std::vector<face_ptr> _faces;
		std::multimap<std::size_t, edge_ptr> _edges;
	};
}
