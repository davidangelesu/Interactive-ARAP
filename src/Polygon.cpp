#include "polygon.h"
#include<algorithm> 

/* Code for the Logic of Is Inside based on https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/  */


// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std:: max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
        (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0; // colinear 
    return (val > 0) ? 1 : 2; // clock or counterclock wise 
}

// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case 
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases 
}

// Returns true if the point p lies inside the polygon with n vertices 
bool Polygon::isInside( int x, int y)
{
    // There must be at least 3 vertices in polygon 
    if (m_vertices.size() < 3) return false;

    Point p_current{ x,y };

    // Create a point for line segment from p_current to infinite 
    Point p_extreme = { 10000, p_current.y };

    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0;
    do
    {
        int next = (i + 1) % m_vertices.size();

        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon at i' to 'polygon at next vertex' 
        Point p_1{ m_vertices[i].x,m_vertices[i].y };
        Point p_2{ m_vertices[next].x,m_vertices[next].y };
        if (doIntersect(p_1, p_2, p_current, p_extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(p_1, p_current, p_2) == 0)
                return onSegment(p_2, p_current, p_2);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise 
    return (count % 2 == 1); 
}