How to Build Sample Program
===================

How to Install Tools and Libraries
--------------------------------------
* Visual Studio 2015  
    In Visual Studio 2015, Visual C++ is not installed by default. When installing, be sure to choose Custom installation and then choose the C++ components you require.  
    Or, if Visual Studio is already installed, choose [File]>[New]>[Project]>[C++] and you will be prompted to install the necessary components.  

* Kinect for Windows SDK v2.0  
    Please download installer, and install follow the instructions of installer.  

* OpenCV 3.1.0  
    Please download pre-built package, and unzip the self-extracting file.  
    Then, Please placed OpenCV folder in any directory. ( e.g. <code>C:\Program Files\opencv</code> )  

* CMake 3.6.1  
    Please download installer, and install follow the instructions of installer.  

* Speech Platform SDK 11 ( and Kinect for Windows SDK v2.0 Language Packs (en-US) )  
    Please download installer for target platform, and install follow the instructions of installer.  

How to Generate Project of Samples using CMake
--------------------------------------------------------
1. Run CMake GUI

2. Fill These Fields
    * **Where is the source code**  
        This area is directory containing CMakeLists.txt file.  
        If you want to build all samples, please fill path of sample directory. ( e.g. <code>C:/Kinect2Sample/sample</code> )  
        If you want to build any one sample, please fill path of any one directory. ( e.g.  <code>C:/Kinect2Sample/sample/Color</code> )  

    * **Where to build the binaries**  
        This area is directory Visual Studio project files will be generated.  
        By convention, Fill the path that added the <code>\/build</code> to  above path. ( e.g. <code>C:/Kinect2Sample/sample/build</code> )  

3. Click Configure Button  
    You will be prompted for compiler and target platform.  
    Please specify compiler and target platform to use. ( e.g. <code>Visual Studio 14 2015 Win64</code> )  
    Then, click finish button.  

4. Fill Configuration Fields  
    It will be entered almost automatically.  
    Please check configuration settings.  
    Then, click configure button.  
    * **KINECTSDK\_DIR** ... The directory path of Kinect for Windows SDK v2.0 ( e.g. <code>C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409</code> )  
    * **OPENCV\_DIR** ... The directory path that to search CMake configuration file for OpenCV. ( e.g. <code>C:/Program Files/opencv/build</code> )  

5. Click Generate Button  
    If there is no errors, the Visual Studio project files will be generated into the "Where to build the binaries" directory.  

6. Set Environment Variable  
    You might need to add OpenCV binary directory path to environment variable "PATH" to be able to run applications. (e.g. <code>C:\Program Filesopencv\build\x64\vc14\bin</code> )  
    The path that have to be added to environment variable will be displayed in output area of CMake GUI.  
    This path is depend on Compiler, Target Platform and OpenCV Directory.  

How to Build and Start Samples
------------------------------------
1. Open Visual Studio Solution File ( e.g. <code>..\build\Sample.sln</code> )  

2. Set Solution Configurations  
    Select Release from the Solution Configuration drop-down list, which is on the Standard toolbar.  ( e.g. <code>Release</code> )  
    Release build will be enabled optimization by compiler.  

3. Build Solution ( or Project )  
    On the Build menu, Click "Build Solution".  
    Or, In Solution Explorer, select the desired build target project within your solution. Then Click "Build Project".  

4. Set Startup Project  
    In Solution Explorer, select the desired startup project within your solution.  
    On the Project menu, choose "Set as StartUp Project".  

5. Start Without Debugging  
    On the Debug menu, choose "Start Without Debugging".  
