#ifndef COT_EV
#define COT_EV
#include <Eigen/Core>

/**
 * Computes the cotangent of the angle located at vertex opposite to side c.
 * @param a Length of side a
 * @param b Length of side b
 * @param c Length of side c
 * @return Cotagent of vertex opposite to side c
 */
double cotagent(double a,double b, double c);

/**
 * Computes the cotagent of the angle located at vertex p3.
 * @param p1 Coordinates of a
 * @param p2 Coordinates of b
 * @param p3 Coordinates of c
 * @return Cotagent of angle at p3
 */
double cotagent(Eigen::Vector3d & p1,Eigen::Vector3d & p2,Eigen::Vector3d & p3);

#endif