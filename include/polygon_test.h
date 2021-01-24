#include <iostream> 
#include <vector>
#include <tuple>
// Code to check if a given point lies inside a given polygon 
// Based on https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
struct Point { int x; int y };
bool isInside(std::vector<std::tuple<int,int>> polygon, int x, int y);