#ifndef VECTOR_ANGLE_HPP
#define VECTOR_ANGLE_HPP

#include <ignition/math/Vector3.hh>
#include <cmath>
#include <algorithm>

namespace nubot {
namespace math {

// Constants
constexpr double PI = M_PI;

// Manual clamp implementation if not available
template <typename T>
inline T clamp(T value, T min, T max) {
    return std::max(min, std::min(max, value));
}

// Function to get cosine of angle between two vectors
inline double get_cos_angle(const ignition::math::Vector3d& vector1, 
                             const ignition::math::Vector3d& target_vector2) 
{
    // Normalize vectors and compute dot product
    auto vector1_norm = vector1.Normalized();
    auto vector2_norm = target_vector2.Normalized();
    return vector1_norm.Dot(vector2_norm);
}

// Function to get sine of angle between two vectors
inline double get_sin_angle(const ignition::math::Vector3d& reference_vector, 
                             const ignition::math::Vector3d& target_vector) 
{
    // Normalize vectors and compute cross product
    auto reference_norm = reference_vector.Normalized();
    auto target_norm = target_vector.Normalized();
    auto cross_vector = reference_norm.Cross(target_norm);
    return (cross_vector.Z() >= 0 ? 1 : -1) * cross_vector.Length();
}

// Get angle in range [-PI, PI]
inline double get_angle_PI(const ignition::math::Vector3d& reference_vector, 
                            const ignition::math::Vector3d& target_vector) 
{
    double cos_angle = get_cos_angle(reference_vector, target_vector);
    double sin_angle = get_sin_angle(reference_vector, target_vector);
    
    // Manual clamping
    cos_angle = std::max(-1.0, std::min(cos_angle, 1.0));
    
    double angle = std::acos(cos_angle);
    return (sin_angle >= 0 ? 1 : -1) * angle;
}

// Get angle in range [0, 2*PI]
inline double get_angle_2PI(const ignition::math::Vector3d& reference_vector, 
                             const ignition::math::Vector3d& target_vector) 
{
    double cos_angle = get_cos_angle(reference_vector, target_vector);
    double sin_angle = get_sin_angle(reference_vector, target_vector);
    
    // Manual clamping
    cos_angle = std::max(-1.0, std::min(cos_angle, 1.0));
    
    double angle = std::acos(cos_angle);
    return (sin_angle > 0 ? angle : (2 * PI - angle));
}

} // namespace math
} // namespace nubot

#endif // VECTOR_ANGLE_HPP