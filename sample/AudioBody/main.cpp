#include <iostream>
#include <sstream>

#include "app.h"

int main( int argc, char* argv[] )
{
    try{
        Kinect kinect;
        kinect.run();
    } catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }

    return 0;
}