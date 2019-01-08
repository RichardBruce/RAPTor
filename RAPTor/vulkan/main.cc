#include <iostream>
#include <stdexcept>
#include <functional>
#include <cstdlib>

#include "vulkan.h"


int main() 
{
    raptor_vulkan::Vulkan vulkan;

    try 
    {
        vulkan.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}