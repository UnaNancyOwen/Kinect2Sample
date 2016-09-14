cmake_minimum_required( VERSION 3.6 )

# Create Project
project( Sample )
add_executable( Speech app.h app.cpp main.cpp util.h KinectAudioStream.h KinectAudioStream.cpp )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "Speech" )

# Find Package
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
find_package( KinectSDK2 REQUIRED )

find_package( SpeechPlatformSDK REQUIRED )

if( KinectSDK2_FOUND AND SpeechPlatformSDK_FOUND )
  # Additional Include Directories
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
  include_directories( ${SpeechPlatformSDK_INCLUDE_DIRS} )

  # Additional Library Directories
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )
  link_directories( ${SpeechPlatformSDK_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( Speech ${KinectSDK2_LIBRARIES} )
  target_link_libraries( Speech ${SpeechPlatformSDK_LIBRARIES} )
endif()