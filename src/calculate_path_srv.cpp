#include "ros/ros.h"
#include <iostream>
#include "stajyer/CalculatePath.h"
#include "new_vector.hpp"

using namespace stajyer;

bool calculatePath(CalculatePathRequest &req, CalculatePathResponse &res)
{
    return true;
}

int main()
{
    Vector a{15.2, 2, 3};
    Vector b{12, 2, 3};
    Vector c = a + b;
    std::cout << c.x << std::endl;
    std::cout << c.y << std::endl;
    std::cout << c.z << std::endl;
    return 0;
}
