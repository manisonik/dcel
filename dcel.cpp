#include "pch.h"
#include "dcel.h"

using namespace dcel;

const bool dcel::vertex::is_boundary()
{
	edge_ptr e = edge;
	do {
		if (not e->pair) {
			return true;
		}
		e = e->pair->next;
	} while (e != edge);

    return false;
}

const std::vector<edge_ptr> dcel::vertex::wheel()
{
    std::vector<edge_ptr> ret;
    auto e = edge;
    do {
        if (not e->pair) {
           // throw exception here
        }
        ret.push_back(e);
        e = e->pair->next;
    } while (e != edge);
    return ret;
}

const std::vector<edge_ptr> dcel::vertex::spoke()
{
    return std::vector<edge_ptr>();
}

const bool dcel::edge::is_boundary()
{
	return pair == nullptr;
}

void face::triangulate(std::function<void(glm::vec3, glm::vec3, glm::vec3)> func)
{
	edge_ptr e = head;
	do {
		edge_ptr e1 = e->next;
		 func(e->vert->attr.pos, e1->vert->attr.pos, e1->next->vert->attr.pos);
		e = e1->next;
	} while (e != head);
}
