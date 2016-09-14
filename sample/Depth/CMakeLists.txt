cmake_minimum_required( VERSION 3.6 )

# Create Project
project( Sample )
add_executable( Depth app.h app.cpp main.cpp util.h )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "Depth" )

# Find Package
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
find_package( KinectSDK2 REQUIRED )

set( OpenCV_DIR "C:/Program Files/opencv/build" )
option( OpenCV_STATIC OFF )
find_package( OpenCV REQUIRED )

# Set Static Link Runtime Library
if( OpenCV_STATIC )
  foreach( flag_var
           CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
           CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
           CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
           CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO )
    if( ${flag_var} MATCHES "/MD" )
      string( REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}" )
    endif()
  endforeach()
endif()

if( KinectSDK2_FOUND AND OpenCV_FOUND )
  # Additional Include Directories
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # Additional Library Directories
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )
  link_directories( ${OpenCV_LIB_DIR} )

  # Additional Dependencies
  target_link_libraries( Depth ${KinectSDK2_LIBRARIES} )
  target_link_libraries( Depth ${OpenCV_LIBS} )
endif()