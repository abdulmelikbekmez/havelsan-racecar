#pragma once
#include <cmath>

struct Vector
{

    float x, y, z;

    Vector(float x, float y, float z) : x{x}, y{y}, z{z}
    {
    }

    Vector operator+(const Vector &other) const
    {
        return Vector{other.x + x, other.y + y, other.z + z};
    }

    Vector operator-(const Vector &other) const
    {
        return Vector{other.x - x, other.y - y, other.z - z};
    }

    Vector operator*(const Vector &other) const
    {
        return Vector{other.x * x, other.y * y, other.z * z};
    }

    Vector operator/(const Vector &other) const
    {
        return Vector{other.x / x, other.y / y, other.z / z};
    }

    float length() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    float angle() const
    {
        return atan2(y, x);
    }

    float angle_to(const Vector other) const
    {
        return other.angle() - angle();
    }

    float dot(const Vector other) const
    {
        return x * other.x + y * other.y;
    }
};
