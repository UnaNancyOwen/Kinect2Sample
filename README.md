
Kinect for Windows SDK v2 Sample Program
============================

This repository is Sample Program of Kinect for Windows SDK v2 written in Native C++.  

Environment
--------------
* Visual Studio Community 2015 <sup>*1</sup>
* Kinect for Windows SDK v2.0
* Kinect for Windows SDK v2.0 Language Packs (en-US)
* Speech Platform SDK 11
* OpenCV 3.1.0 <sup>*2</sup>
* CMake 3.6.1 <sup>*3</sup>

<sup>&#042;1 This sample program need Visual Studio Community (or upper version), because depends on ATL.</sup>  
<sup>&#042;2 Pre-built OpenCV that is distributed by official team does not include library for Win32 (x86) target platform. If you want to build sample program for Win32 (x86) target platform, You need build OpenCV yourself. Similarly, If it does not include library for your target compiler, You need build OpenCV yourself.</sup>  
<sup>&#042;3 You need generate project of this sample program using CMake. Please read [this document](HOWTOBUILD.md) about how to generate project using CMake.</sup>  

License
---------
Copyright &copy; 2016 Tsukasa SUGIURA  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php "MIT License | Open Source Initiative").

Contact
---------
* Tsukasa Sugiura  
    * <t.sugiura0204@gmail.com>  
    * <https://twitter.com/UnaNancyOwen>  
    * <http://UnaNancyOwen.com>  

Reference
------------
* KINECT for Windows SDK programming - Kinect for Windows v2 sensor supported version | Shuwa System Co.,Ltd.  
  <http://www.shuwasystem.co.jp/products/7980html/4395.html>

* Kinect for Windows SDK 2.0 | MSDN Library  
  <https://msdn.microsoft.com/en-us/library/dn799271.aspx>