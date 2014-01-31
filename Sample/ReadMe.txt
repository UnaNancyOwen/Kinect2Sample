Kinect for Windows SDK v2 Sample Program ReadMe.txt


■Structure
Structure of this Sample Program is as follows.

    Kinect2Sample
    │
    │  // Sample Program
    ├─Sample
    │  │
    │  ├─Sample.sln
    │  │
    │  ├─Color
    │  │  ├─Color.vcxproj
    │  │  └─Color.cpp
    │  │
    │  ├─Depth
    │  │  ├─Depth.vcxproj
    │  │  └─Depth.cpp
    │  │
    │  ├─BodyIndex
    │  │  ├─BodyIndex.vcxproj
    │  │  └─BodyIndex.cpp
    │  │
    │  ├─Body
    │  │  ├─Body.vcxproj
    │  │  └─Body.cpp
    │  │
    │  ├─Infrared
    │  │  ├─Infrared.vcxproj
    │  │  └─Infrared.cpp
    │  │
    │  ├─Event
    │  │  ├─Event.vcxproj
    │  │  └─Event.cpp
    │  │
    │  ├─AudioBeam
    │  │  ├─AudioBeam.vcxproj
    │  │  └─AudioBeam.cpp
    │  │
    │  ├─Speech
    │  │  ├─Speech.vcxproj
    │  │  ├─Speech.cpp
    │  │  ├─KinectAudioStream.h
    │  │  ├─KinectAudioStream.cpp
    │  │  └─SpeechRecognition.grxml
    │  │
    │  ├─AudioBody
    │  │  ├─AudioBody.vcxproj
    │  │  └─AudioBody.cpp
    │  │
    │  ├─Gesture
    │  │  ├─Gesture.vcxproj
    │  │  └─Gesture.cpp
    │  │
    │  ├─Face
    │  │  ├─Face.vcxproj
    │  │  └─Face.cpp
    │  │
    │  ├─LongExposureInfrared
    │  │  ├─LongExposureInfrared.vcxproj
    │  │  └─LongExposureInfrared.cpp
    │  │
    │  ├─CoodinateMapper
    │  │  ├─CoodinateMapper.vcxproj
    │  │  └─CoodinateMapper.cpp
    │  │
    │  ├─Fusion
    │  │  ├─Fusion.vcxproj
    │  │  ├─Fusion.cpp
    │  │  ├─KinectFusionHelper.h
    │  │  └─KinectFusionHelper.cpp
    │  │
    │  ├─HDFace
    │  │  ├─HDFace.vcxproj
    │  │  └─HDFace.cpp
    │  │
    │  ├─Controls
    │  │  ├─Controls.vcxproj
    │  │  └─Controls.cpp
    │  │
    │  │  // Property Sheet
    │  ├─Sample.props
    │  │
    │  │  // License
    │  ├─License.txt
    │  │
    │  │  // Read Me
    │  └─ReadMe.txt
    │
    └─README.md


■About Sample Program
This Sample Program is setting to be able to build in the following environments. 
Please rewrite the appropriate setting depending on the environment.

    Visual Studio Express 2013 for Windows Desktop
    Kinect for Windows SDK v2.0
    OpenCV 2.4.10
    Speech Platform SDK 11
    Kinect for Windows SDK v2.0 Language Packs (en-US)


■Setting for OpenCV
Please UnZip to the following Location.

    Location:
    C:\Program Files\opencv

Please set the following Environment Variables.

    Variable Name:	Variable Value:
    OPENCV_DIR		C:\Program Files\opencv\
    OPENCV_VER		2410
    Path		;%OPENCV_DIR%\build\x64\vc12\bin

For more information, see this page.

    Summary?Blog - How to build the Development Environment of OpenCV
    http://unanancyowen.com/?p=97


■Build Insider
Kinect for Windows v2 Introduction - for C++ Programmers Article Series
http://www.buildinsider.net/small/kinectv2cpp

    1. To Compare Kinect v1 and Kinect v2
    http://www.buildinsider.net/small/kinectv2cpp/01

    2. Kinect v2 Programming for C++ - Color part
    http://www.buildinsider.net/small/kinectv2cpp/02

    3. Kinect v2 Programming for C++ - Depth part
    http://www.buildinsider.net/small/kinectv2cpp/03

    4. Kinect v2 Programming for C++ - BodyIndex part
    http://www.buildinsider.net/small/kinectv2cpp/04

    5. Kinect v2 Programming for C++ - Body part
    http://www.buildinsider.net/small/kinectv2cpp/05

    6. Kinect Studio v2
    http://www.buildinsider.net/small/kinectv2cpp/06

    7. Kinect v2 Programming for C++ - AudioBeam part
    http://www.buildinsider.net/small/kinectv2cpp/07


■About License
License of this Sample Program suppose MIT License.
Please refer to License.txt.


■Disclaimer
"This is preliminary software and/or hardware and APIs are preliminary and subject to change."

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIM
ED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DI
RECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLU
DING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF 
USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEO
RY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGL
IGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN I
F ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


■Change History
2014/02/01 Created Sample Program
2014/02/01 Add Color
2014/02/14 Add Depth
2014/02/28 Add BodyIndex
2014/03/07 Add Body
2014/03/14 Add Infrared
2014/04/18 Add Event
2014/06/25 Add AudioBeam
2014/07/24 Add Speech
2014/08/19 Add AudioBody
2014/08/20 Add Gesture
2014/08/20 Add Face
2014/08/21 Add LongExposureInfrared
2014/08/21 Add CoodinateMapper
2014/08/21 Add Fusion
2014/08/26 Add HDFace
2015/02/10 Add Controls