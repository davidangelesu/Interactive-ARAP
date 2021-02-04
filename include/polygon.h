#ifndef POLYGON_H
#define POLYGON_H

#include <vector>

// Code to check if a given point lies inside a given polygon 
// Based on https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
namespace GUI {
	struct Point { double x; double y; };
	class Polygon {
	public:
		/**
		 * @brief  Evaluates if point is inside the Polygon
		 * @param x x-Coordinate
		 * @param y y-Coordinate
		 * @return if point is inside Polygon
		*/
		bool isInside(double x, double y);
		/**
		 * @brief Adds a vertex to the list of vertices that define the polygon
		 * @param x x-Coordinate
		 * @param y y-Coordinate
		*/
		inline void addVertex(double x, double y) { m_vertices.push_back(Point{ x,y }); };
		/**
		 * @brief Clears the list of vertices
		*/
		inline void clearVertices() { m_vertices.clear(); };
	private:
		std::vector<Point> m_vertices;

	};

	
};


#endif // !POLYGON_H