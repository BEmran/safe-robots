#include "core/utils/math.hpp"

#include <eigen3/Eigen/Core>

typedef Eigen::Vector3d EigVec3;
typedef Eigen::Matrix3Xd EigMat3;
typedef Eigen::Rotation2D<core::math::TYPE> EigRot2;
typedef Eigen::Quaternion<core::math::TYPE> EigQuat;
typedef Eigen::Transform<core::math::TYPE, 3, Eigen::Affine> EigTransform;

class Vec : public core::math::Vec3
{
    private:
    data
}