#.rst:
# FindSpeechPlatformSDK
# ---------------------
#
# Find Speech Platform SDK include dirs, library dirs, libraries
#
# Use this module by invoking find_package with the form::
#
#    find_package( SpeechPlatformSDK [REQUIRED] )
#
# Results for users are reported in following variables::
#
#    SpeechPlatformSDK_FOUND         - Return "TRUE" when Speech Platform SDK found. Otherwise, Return "FALSE".
#    SpeechPlatformSDK_INCLUDE_DIRS  - Speech Platform SDK include directories. ($(SpeechPlatformSDK_DIR)/Include)
#    SpeechPlatformSDK_LIBRARY_DIRS  - Speech Platform SDK library directories. ($(SpeechPlatformSDK_DIR)/Lib)
#    SpeechPlatformSDK_LIBRARIES     - Speech Platform SDK library files. ($(SpeechPlatformSDK_LIBRARY_DIRS)/sapi.lib)
#
# CMake entries::
#
#    SpeechPlatformSDK_DIR           - Speech Platform SDK root directory. (Default $(ProgramFiles)/Microsoft SDKs/Speech/v11.0 or $(ProgramW6432)/Microsoft SDKs/Speech/v11.0)
#
# Example to find Speech Platform SDK::
#
#    cmake_minimum_required( VERSION 2.8 )
#
#    project( project )
#    add_executable( project main.cpp )
#
#    # Find package using this module.
#    find_package( SpeechPlatformSDK REQUIRED )
#
#    if( SpeechPlatformSDK_FOUND )
#      # [C/C++]>[General]>[Additional Include Directories]
#      include_directories( ${SpeechPlatformSDK_INCLUDE_DIRS} )
#
#      # [Linker]>[General]>[Additional Library Directories]
#      link_directories( ${SpeechPlatformSDK_LIBRARY_DIRS} )
#
#      # [Linker]>[Input]>[Additional Dependencies]
#      target_link_libraries( project ${SpeechPlatformSDK_LIBRARIES} )
#    endif()
#
# =============================================================================
#
# Copyright (c) 2016 Tsukasa SUGIURA
# Distributed under the MIT License.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# =============================================================================

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(SpeechPlatformSDK_FOUND FALSE)
    unset(_DIR)
  endif()
endmacro()

# Check Files Macro
macro(CHECK_FILES _FILES _DIR)
  set(_MISSING_FILES)
  foreach(_FILE ${${_FILES}})
    if(NOT EXISTS "${_FILE}")
      get_filename_component(_FILE ${_FILE} NAME)
      set(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
    endif()
  endforeach()
  if(_MISSING_FILES)
    message(WARNING "In directory \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
    set(SpeechPlatformSDK_FOUND FALSE)
    unset(_FILES)
  endif()
endmacro()

# Program Files
set(PROGRAMFILES)
if(NOT CMAKE_CL_64)
  set(PROGRAMFILES $ENV{ProgramFiles})
else()
  set(PROGRAMFILES $ENV{ProgramW6432})
endif()

##### Find Speech Platform SDK #####

# Found
set(SpeechPlatformSDK_FOUND TRUE)

# Root Directoty
set(SpeechPlatformSDK_DIR)
if(SpeechPlatformSDK_FOUND)
  set(SpeechPlatformSDK_DIR ${PROGRAMFILES}/Microsoft\ SDKs/Speech/v11.0 CACHE PATH "Speech Platform SDK Install Path." FORCE)
  check_dir(SpeechPlatformSDK_DIR)
endif()

# Include Directories
set(SpeechPlatformSDK_INCLUDE_DIRS)
if(SpeechPlatformSDK_FOUND)
  set(SpeechPlatformSDK_INCLUDE_DIRS ${SpeechPlatformSDK_DIR}/Include)
  check_dir(SpeechPlatformSDK_INCLUDE_DIRS)
endif()

# Library Directories
set(SpeechPlatformSDK_LIBRARY_DIRS)
if(SpeechPlatformSDK_FOUND)
  set(SpeechPlatformSDK_LIBRARY_DIRS ${SpeechPlatformSDK_DIR}/Lib)
  check_dir(SpeechPlatformSDK_LIBRARY_DIRS)
endif()

# Dependencies
set(SpeechPlatformSDK_LIBRARIES)
if(SpeechPlatformSDK_FOUND)
  set(SpeechPlatformSDK_LIBRARIES ${SpeechPlatformSDK_LIBRARY_DIRS}/sapi.lib)
  check_files(SpeechPlatformSDK_LIBRARIES SpeechPlatformSDK_LIBRARY_DIRS)
endif()

message(STATUS "SpeechPlatformSDK_FOUND : ${SpeechPlatformSDK_FOUND}")