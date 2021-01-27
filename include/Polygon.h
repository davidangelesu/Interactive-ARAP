#include <vector>

// Code to check if a given point lies inside a given polygon 
// Based on https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
namespace GUI {

	class Polygon {
	public:
		bool isInside(int x, int y);
		inline void addVertex(int x, int y) { m_vertices.push_back(Point{ x,y }); };
		inline void clearVertices() { m_vertices.clear(); };
	private:
		std::vector<Point> m_vertices;

	};

	struct Point { int x; int y; };
};