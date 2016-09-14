cmake_minimum_required( VERSION 3.6 )

# Create Project
project( Sample )

# Set Binary Output Directory
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )

# Sample Sub-Directories Name  
set( SAMPLES Color Depth Infrared BodyIndex Body JointSmooth MultiSource CoordinateMapper Face HDFace Fusion Gesture Speech AudioBeam AudioBody ChromaKey FaceClip )

# Sample Build Option
foreach( SAMPLE ${SAMPLES} )
  option( BUILD_${SAMPLE} "Build ${SAMPLE} Sample Project" ON )
endforeach()

# Sample Add Sub-Directories
foreach( SAMPLE ${SAMPLES} )
  if( BUILD_${SAMPLE} )
    add_subdirectory( ${SAMPLE} )
  endif()
endforeach()

# Adjust ( Copy Run-Time Files )
if( BUILD_Speech )
  file( COPY ${CMAKE_SOURCE_DIR}/Speech/Grammar_enUS.grxml DESTINATION ${CMAKE_BINARY_DIR}/bin )
  file( COPY ${CMAKE_SOURCE_DIR}/Speech/Grammar_jaJP.grxml DESTINATION ${CMAKE_BINARY_DIR}/bin )
endif()

if( BUILD_Gesture )
  file( COPY ${CMAKE_SOURCE_DIR}/Gesture/SampleDatabase.gbd DESTINATION ${CMAKE_BINARY_DIR}/bin )
endif()