#ifndef COMMON_MATH_H_
#define COMMON_MATH_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace GlobalPlan
{

template <typename FloatType>
FloatType GetDistPlaneToPoint(const Eigen::Matrix<FloatType, 4, 1> &plane,
                              const Eigen::Matrix<FloatType, 3, 1> &point)
{
    FloatType x = point.x();
    FloatType y = point.y();
    FloatType z = point.z();
    FloatType a = plane[0];
    FloatType b = plane[1];
    FloatType c = plane[2];
    FloatType d = plane[3];
    return std::abs(a * x + b * y + c * z + d) / std::sqrt(a * a + b * b + c * c);
}

template <typename FloatType>
Eigen::Matrix<FloatType, 4, 1> FindPlane(const Eigen::Matrix<FloatType, 3, 1> point, const Eigen::Matrix<FloatType, 3, 1> normal_vector)
{
    FloatType x = point.x();
    FloatType y = point.y();
    FloatType z = point.z();
    FloatType a = normal_vector.x();
    FloatType b = normal_vector.y();
    FloatType c = normal_vector.z();

    return Eigen::Matrix<FloatType, 4, 1>(a, b, c, -(a * x + b * y + c * z));
}

template <typename FloatType>
FloatType GetZOnSurface(const Eigen::Matrix<FloatType, 4, 4> &Tinv, const FloatType x, const FloatType y, const FloatType variance = 6)
{
    FloatType a = Tinv(0, 0);
    FloatType b = Tinv(0, 1);
    FloatType c = Tinv(0, 2);
    FloatType d = Tinv(0, 3);

    FloatType e = Tinv(1, 0);
    FloatType f = Tinv(1, 1);
    FloatType g = Tinv(1, 2);
    FloatType h = Tinv(1, 3);

    FloatType i = Tinv(2, 0);
    FloatType j = Tinv(2, 1);
    FloatType k = Tinv(2, 2);
    FloatType l = Tinv(2, 3);

    //x' = a*x+b*y+c*z+d
    //y' = e*x+f*y+g*z+h
    //z' = i*x+j*y+k*z+l
    //x'^2 + y'^2 + z'^2 = 1
    //we need slove it!
    //(a*x+b*y+c*z+d)^2 + (e*x+f*y+g*z+h)^2 + (i*x+j*y+k*z+l)^2 = 1

    FloatType m = a * x + b * y + d;
    FloatType n = e * x + f * y + h;
    FloatType o = i * x + j * y + l;

    FloatType p = (2 * c * m + 2 * g * n + 2 * k * o);
    FloatType q = (c * c + g * g + k * k) * (m * m + n * n + o * o - variance);
    FloatType delta = p * p - 4 * q;

    if (delta < 0)
        return NAN;

    //we only use the big one;

    if (c * c + g * g + k * k == 0)
        return NAN;

    FloatType z = (sqrt(delta) / 2 - c * m - g * n - k * o) / (c * c + g * g + k * k);
    return z;
}
static Eigen::Matrix4d MakeTinv(const Eigen::Matrix3d &matrixU, const Eigen::Vector3d &singularValues, const Eigen::Vector3d &centroid)
{
    Eigen::Matrix3d R(matrixU);
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2) = R.col(0).cross(R.col(1));
    R.col(2).normalize();
    R.col(0) = R.col(1).cross(R.col(2));
    R.col(0).normalize();
    Eigen::Matrix3d S;
    S << sqrt(singularValues.x()), 0, 0,
        0, sqrt(singularValues.y()), 0,
        0, 0, sqrt(singularValues.z());

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R * S;
    T(0, 3) = centroid(0);
    T(1, 3) = centroid(1);
    T(2, 3) = centroid(2);
    Eigen::Matrix4d Tinv = T.inverse();
    return Tinv;
}
} // namespace GlobalPlan

#endif
