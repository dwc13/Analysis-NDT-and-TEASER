#include <iostream>
#include <string>
#include "normal_distributions_transform.h"

int main (int argc, char** argv)
{
    std::string algorithm = argv[1];
    if (algorithm == "ndt")
    {
        std::cout << "Running Normal Distribution Transform!\n";
        NDT::ndt();
    }
    else
        std::cout << "Invalid Usage. Example: ./main.cpp ndt";
    return (0);
}
