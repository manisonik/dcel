#pragma once
#include "pch.h"

namespace dcel {
	// Forward classes
	class edge;
	class face;
	class vertex;
	class plane;

	// Vertex pointer type
	using vertex_ptr = std::shared_ptr<vertex>;

	// Edge pointer type
	using edge_ptr = std::shared_ptr<edge>;

	// Face pointer type
	using face_ptr = std::shared_ptr<face>;

	// Plane pointer type
	using plane_ptr = std::shared_ptr<plane>;

	template <bool Const = false>
	class face_iterator {
	public:
		/* Difference type */
		using difference_type = std::size_t;
		/** Value type */
		using value_type = edge_ptr;
		/* Pointer type */
		using pointer =
			std::conditional_t<Const, value_type const*, value_type*>;
		/* Reference type */
		using reference =
			std::conditional_t<Const, value_type const&, value_type&>;
		/*Iterator category */
		using iterator_category = std::input_iterator_tag;

		/* Default constructor == End iterator */
		face_iterator() = default;

		/** Construct from head of triangle and current edge */
		explicit face_iterator(const edge_ptr& head, const edge_ptr& current)
			: _head{ head }, _current{ current }
		{
		}

		/* Dereference operator */
		template <bool Const_ = Const>
		std::enable_if_t<Const_, reference> operator*() const
		{
			return _current;
		}

		/* Dereference operator */
		template <bool Const_ = Const>
		std::enable_if_t<not Const_, reference> operator*()
		{
			return _current;
		}

		/* Equality operator */
		auto operator==(const face_iterator& other) const -> bool
		{
			return _current == other._current;
		}

		/* Inequality operator */
		auto operator!=(const face_iterator& other) const -> bool
		{
			return !(*this == other);
		}

		/* Increment operator */
		auto operator++() -> face_iterator&
		{
			// Already at end
			if (_current == nullptr) {
				return *this;
			}

			// Get the next edge
			_current = _current->next;

			// If back at head, done iterating
			if (_current == _head) {
				_current = nullptr;
			}

			return *this;
		}

	private:
		edge_ptr _head;
		edge_ptr _current;
	};

	class vertex_attr
	{
	public:
		glm::vec3 pos;					// vertex position
		glm::vec3 normal;				// vertex normal
		glm::vec2 uv;					// texture coordinates
		glm::vec2 tangent;				// tangent
		glm::vec2 bitangent;			// bitanget
	};

	class vertex
	{
	public:
		const bool is_boundary();

		/* Set of all faces that share this interior node */
		const std::vector<edge_ptr> wheel();

		/* Edges in the wheel connected to the interior node */
		const std::vector<edge_ptr> spoke();

		vertex_attr attr;				// vertex attributes
		face_ptr face;					// face the vertex belongs to
		vertex_ptr parent;				// parent node
		edge_ptr edge;					// one of the half-edges emantating from the vertex
		size_t idx;						// insertion index
		std::vector<edge_ptr> edges;	// list of edges with this vertex endpoint

		// ABF++
		float lambda_plan;
		float lambda_len;
	};

	class edge
	{
	public:
		const bool is_boundary();

		vertex_ptr vert; // vertex at the end of the half-edge 
		edge_ptr pair;   // oppositely oriented adjacent half-edge
		face_ptr face;   // face the half-edge borders
		edge_ptr next;   // next half-edge around the face
		edge_ptr prev;   // prev half-edge around the face
		size_t idx;		 // insertion index

		// ABF++
		float alpha;
		float alpha_sin;
		float alpha_cos;
		float phi;
		float beta;
		float weight;
	};

	class face
	{
	public:
		/* Face edge iterator type */
		using iterator = face_iterator<false>;

		/* Face edge const iterator type */
		using const_iterator = face_iterator<true>;
		
		/* Returns an iterator over the edges of the face */
		iterator begin() { return iterator{ head, head }; }

		/* Returns the end iterator */
		iterator end() { return iterator(); }

		/* Returns an const iterator over the edges of the face */
		const_iterator cbegin() const { return const_iterator{ head, head }; }

		/* Returns the const end iterator */
		const_iterator cend() const { return const_iterator(); }

		/* Triangulate the face */
		void triangulate(std::function<void(glm::vec3, glm::vec3, glm::vec3)> func);

		face_ptr next;			// next face
		edge_ptr head;			// one of the half-edges bordering the face
		glm::vec3 faceNormal;   // face normal
		size_t idx;				// insertion index
		size_t vertexCount;     // vertex count

		// ABF++
		float lambda_tri;
	};

	class plane
	{
	public:
		glm::vec3 normal;
		glm::vec3 centroid;
	};
}