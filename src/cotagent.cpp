#include "../include/cotagent.h"
#include <Eigen/Core>

double cotagent(double a, double b, double c) {
	double numerator = pow(a, 2) + pow(b, 2) - pow(c, 2);

	double s = (a + b + c) / 2;
	double denominator = 4 * sqrt(s * (s - a) * (s - b) * (s - c));
	return numerator/denominator;
};

double cotagent(Eigen::Vector3d & p1, Eigen::Vector3d & p2, Eigen::Vector3d & p3) {
	double a = (p2 - p3).norm();
	double b = (p1 - p3).norm();
	double c = (p1 - p2).norm();
	return cotagent(a, b, c);
}