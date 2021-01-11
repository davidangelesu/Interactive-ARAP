#ifndef COT_EV
#define COT_EV
#include <Eigen/Core>
/// <summary>
/// Computes the cotagent of the angle located at vertex opposite to side c.
/// </summary>
/// <param name="a">Length of side a</param>
/// <param name="b">Length of side b</param>
/// <param name="c">Length of side c</param>
/// <returns>Cotagent of vertex opposite to side c</returns>
double cotagent(double a,double b, double c);

/// <summary>
/// Computes the cotagent of the angle located at vertex p3.
/// </summary>
/// <param name="p1">Coordinates of a</param>
/// <param name="p2">Coordinates of b</param>
/// <param name="p3">Coordinates of c</param>
/// <returns>Cotagent of angle at p3</returns>
double cotagent(Eigen::Vector3d & p1,Eigen::Vector3d & p2,Eigen::Vector3d & p3);

#endif