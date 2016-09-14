cmake_minimum_required( VERSION 3.6 )

# Create Project
project( Sample )
add_executable( AudioBeam app.h app.cpp main.cpp util.h )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "AudioBeam" )

# Find Package
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
find_package( KinectSDK2 REQUIRED )

if( KinectSDK2_FOUND )
  # Additional Include Directories
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )

  # Additional Library Directories
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( AudioBeam ${KinectSDK2_LIBRARIES} )
endif()