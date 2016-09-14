//------------------------------------------------------------------------------
// <copyright file="KinectFusionHelper.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "NuiKinectFusionApi.h"
#include <vector>
#include <string>

/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
void SetIdentityMatrix(Matrix4 &mat);

/// <summary>
/// Extract translation values from the 4x4 Matrix4 transformation in M41,M42,M43
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <param name="translation">Array of 3 floating point values for translation.</param>
void ExtractVector3Translation(const Matrix4 &transform, _Out_cap_c_(3) float *translation);

/// <summary>
/// Extract translation Vector3 from the 4x4 Matrix transformation in M41,M42,M43
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <returns>Returns a Vector3 containing the translation.</returns>
Vector3 ExtractVector3Translation(const Matrix4 &transform);

/// <summary>
/// Extract 3x3 rotation from the 4x4 Matrix and return in new Matrix4
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <returns>Returns a Matrix4 containing the rotation.</returns>
Matrix4 Extract3x3Rotation(const Matrix4 &transform);

/// <summary>
/// Extract 3x3 rotation matrix from the Matrix4 4x4 transformation:
/// Then convert to Euler angles.
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <param name="rotation">Array of 3 floating point values for euler angles.</param>
void ExtractRot2Euler(const Matrix4 &transform, _Out_cap_c_(3) float *rotation);

/// <summary>
/// Test whether the camera moved too far between sequential frames by looking at starting and end transformation matrix.
/// We assume that if the camera moves or rotates beyond a reasonable threshold, that we have lost track.
/// Note that on lower end machines, if the processing frame rate decreases below 30Hz, this limit will potentially have
/// to be increased as frames will be dropped and hence there will be a greater motion between successive frames.
/// </summary>
/// <param name="T_initial">The transform matrix from the previous frame.</param>
/// <param name="T_final">The transform matrix from the current frame.</param>
/// <param name="maxTrans">The maximum translation in meters we expect per x,y,z component between frames under normal motion.</param>
/// <param name="maxRotDegrees">The maximum rotation in degrees we expect about the x,y,z axes between frames under normal motion.</param>
/// <returns>true if camera transformation is greater than the threshold, otherwise false</returns>
bool CameraTransformFailed(const Matrix4 &T_initial, const Matrix4 &T_final, float maxTrans, float maxRotDegrees);

/// <summary>
/// Invert the 3x3 Rotation Matrix Component of a 4x4 matrix
/// </summary>
/// <param name="rot">The rotation matrix to invert.</param>
void InvertRotation(Matrix4 &rot);

/// <summary>
/// Negate the 3x3 Rotation Matrix Component of a 4x4 matrix
/// </summary>
/// <param name="rot">The rotation matrix to negate.</param>
void NegateRotation(Matrix4 &rot);

/// <summary>
/// Rotate a vector with the 3x3 Rotation Matrix Component of a 4x4 matrix
/// </summary>
/// <param name="vec">The Vector3 to rotate.</param>
/// <param name="rot">Rotation matrix.</param>
Vector3 RotateVector(const Vector3 &vec, const Matrix4 &rot);

/// <summary>
/// Invert Matrix4 Pose either from WorldToCameraTransform (view) matrix to CameraToWorldTransform pose matrix (world/SE3) or vice versa
/// </summary>
/// <param name="transform">The camera pose transform matrix.</param>
/// <returns>Returns a Matrix4 containing the inverted camera pose.</returns>
Matrix4 InvertMatrix4Pose(const Matrix4 &transform);

/// <summary>
/// Write Binary .STL mesh file
/// see http://en.wikipedia.org/wiki/STL_(file_format) for STL format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteBinarySTLMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ = true);

/// <summary>
/// Write ASCII Wavefront .OBJ mesh file
/// See http://en.wikipedia.org/wiki/Wavefront_.obj_file for .OBJ format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiObjMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ = true);

/// <summary>
/// Write ASCII .PLY file
/// See http://paulbourke.net/dataformats/ply/ for .PLY format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <param name="outputColor">Set this true to write out the surface color to the file when it has been captured.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiPlyMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ = true, bool outputColor = false);

/// <summary>
/// Write ASCII Wavefront .OBJ file with bitmap texture and material file
/// See http://en.wikipedia.org/wiki/Wavefront_.obj_file for .OBJ format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <param name="pTexture">The Kinect Fusion color texture image.</param>
/// <param name="texcoords">Three Vector3 texture coordinates per mesh triangle, normalized by the image size.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT WriteTexturedeAsciiObjMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ, NUI_FUSION_IMAGE_FRAME *pTexture, const std::vector<Vector3> &texcoords);

/// <summary>
/// Returns whether this is running as a 32 or 64bit application.
/// </summary>
/// <returns>TRUE indicates a 64bit app.</returns>
BOOL Is64BitApp();

/// <summary>
/// Write 32bit BMP image file
/// </summary>
/// <param name="pszFile">The full path and filename of the file to save.</param>
/// <param name="imageBytes">A pointer to the image bytes to save.</param>
/// <param name="width">The width of the image to save.</param>
/// <param name="height">The width of the image to save.</param>
/// <returns>indicates success or failure</returns>
HRESULT SaveBMPFile(LPCWSTR pszFile, const byte *pImageBytes, unsigned int width, unsigned int height);

/// <summary>
/// Copy an image with identical sizes and parameters.
/// </summary>
/// <param name="pSrc">A pointer to the source image.</param>
/// <param name="pDest">A pointer to the destination image.</param>
/// <returns>indicates success or failure</returns>
HRESULT CopyImageFrame(const NUI_FUSION_IMAGE_FRAME *pSrc, const NUI_FUSION_IMAGE_FRAME *pDest);

/// <summary>
/// Horizontally mirror a 32bit (color/float) image in-place.
/// </summary>
/// <param name="pImage">A pointer to the image to mirror.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT HorizontalMirror32bitImageInPlace(const NUI_FUSION_IMAGE_FRAME *pImage);

/// <summary>
/// Horizontally mirror a 32bit (color/float) image.
/// </summary>
/// <param name="pSrcImage">A pointer to the image to mirror.</param>
/// <param name="pDestImage">A pointer to the destination mirrored image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT HorizontalMirror32bitImage(const NUI_FUSION_IMAGE_FRAME *pSrcImage, const NUI_FUSION_IMAGE_FRAME *pDestImage);

/// <summary>
/// Color the residual/delta image from the AlignDepthFloatToReconstruction call
/// </summary>
/// <param name="pFloatDeltaFromReference">A pointer to the source pFloatDeltaFromReference image.</param>
/// <param name="pShadedDeltaFromReference">A pointer to the destination ShadedDeltaFromReference image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT ColorResiduals(const NUI_FUSION_IMAGE_FRAME *pFloatDeltaFromReference, const NUI_FUSION_IMAGE_FRAME *pShadedDeltaFromReference);

/// <summary>
/// Statistics calculated for a FloatDeltaFromReference Image after the 
/// AlignDepthFloatToReconstruction and CalculateResidualStatistics calls.
/// </summary>
struct DeltaFromReferenceImageStatistics
{
    unsigned int totalPixels;
    unsigned int zeroPixels;
    unsigned int validPixels;
    unsigned int invalidDepthOutsideVolumePixels;
    float totalValidPixelsDistance;
};

/// <summary>
/// Calculate statistics on the residual/delta image from the AlignDepthFloatToReconstruction call.
/// </summary>
/// <param name="pFloatDeltaFromReference">A pointer to the source FloatDeltaFromReference image.</param>
/// <param name="stats">A pointer to a DeltaFromReferenceImageStatistics struct to fill with the statistics.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT CalculateResidualStatistics(const NUI_FUSION_IMAGE_FRAME *pFloatDeltaFromReference, DeltaFromReferenceImageStatistics *stats);

/// <summary>
/// Down sample color, depth float or point cloud frame with nearest neighbor
/// </summary>
/// <param name="src">The source color, depth float or pointcloud image.</param>
/// <param name="dest">The destination down sampled color, depth float or pointcloud image.</param>
/// <param name="factor">The down sample factor (1=just copy, 2=x/2,y/2, 4=x/4,y/4).</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT DownsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor);

/// <summary>
/// Up sample color or depth float (32 bits/pixel) frame with nearest neighbor - replicates pixels
/// </summary>
/// <param name="src">The source color image.</param>
/// <param name="dest">The destination up-sampled color image.</param>
/// <param name="factor">The up sample factor (1=just copy, 2=x*2,y*2, 4=x*4,y*4).</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT UpsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor);

/// <summary>
/// Down sample color frame with nearest neighbor to the depth frame resolution
/// </summary>
/// <param name="src">The source color image.</param>
/// <param name="dest">The destination down sampled  image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT DownsampleColorFrameToDepthResolution(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest);

/// <summary>
/// Convert int to string
/// </summary>
/// <param name="theValue">The int value to convert.</param>
/// <returns>Returns a string containing the int value.</returns>
inline std::string to_string(int theValue)
{
    char buffer[65];

    errno_t err = _itoa_s(theValue, buffer, ARRAYSIZE(buffer), 10);

    if (0 != err)
    {
        return std::string("");
    }

    return std::string(buffer);
}

/// <summary>
/// Convert float to string
/// </summary>
/// <param name="theValue">The float value to convert.</param>
/// <returns>Returns a string containing the float value.</returns>
inline std::string to_string(float theValue)
{
    char buffer[_CVTBUFSIZE];

    errno_t err = _gcvt_s(buffer, _CVTBUFSIZE, theValue, 6);

    if (0 != err)
    {
        return std::string("");
    }

    return std::string(buffer);
}

/// <summary>
/// Clamp a value if outside two given thresholds
/// </summary>
/// <param name="x">The value to clamp.</param>
/// <param name="a">The minimum inclusive threshold.</param>
/// <param name="b">The maximum inclusive threshold.</param>
/// <returns>Returns the clamped value.</returns>
template <typename T>
inline T clamp(const T& x, const T& a, const T& b)
{
    if (x < a)
        return a;
    else if (x > b)
        return b;
    else
        return x;
}

/// <summary>
/// Load an 24bit RGB color from a packed int pixel image and return as float values
/// </summary>
/// <param name="colorImage">The int image array.</param>
/// <param name="x">The x coordinate of the pixel to return.</param>
/// <param name="y">The y coordinate of the pixel to return.</param>
/// <param name="imageWidth">The width of the image in pixels.</param>
/// <returns>Returns a Vector3 containing the rgb color components.</returns>
inline Vector3 load_color(const unsigned int *colorImage, int x, int y, int imageWidth)
{
    Vector3 rgb;

    unsigned int packedValue = colorImage[(y * imageWidth) + x];

    rgb.x = static_cast<float>(packedValue & 255);          // r
    rgb.y = static_cast<float>((packedValue >> 8) & 255);   // g
    rgb.z = static_cast<float>((packedValue >> 16) & 255);  // b

    return rgb;
}

/// <summary>
/// Linearly interpolate (Lerp) between two values.
/// </summary>
/// <param name="z">The first value.</param>
/// <param name="b">The second value.</param>
/// <param name="f">The amount to interpolate between the values.</param>
/// <returns>Returns the interpolated value.</returns>
template <typename T, typename Tf>
inline auto lerp(const T& a, const T& b, Tf f) -> decltype(a + f * (b - a))
{
    return a + f * (b - a);
}

/// <summary>
/// Linearly interpolate (Lerp) between two Vector3-based RGB color pixels.
/// </summary>
/// <param name="z">The first color pixel.</param>
/// <param name="b">The second color pixel.</param>
/// <param name="f">The amount to interpolate between the values.</param>
/// <returns>Returns the interpolated value.</returns>
inline Vector3 lerp_color(const Vector3 &a, const Vector3 &b, float f)
{
    Vector3 rgb;

    rgb.x = lerp(a.x, b.x, f);  // r
    rgb.y = lerp(a.y, b.y, f);  // g
    rgb.z = lerp(a.z, b.z, f);  // b

    return rgb;
}

/// <summary>
/// Bilinear sample an RGB int image
/// </summary>
/// <param name="colorImage">The int image array.</param>
/// <param name="x">The x coordinate of the pixel to return.</param>
/// <param name="y">The y coordinate of the pixel to return.</param>
/// <param name="imageWidth">The width of the image in pixels.</param>
/// <param name="imageHeight">The height of the image in pixels.</param>
/// <returns>Returns a packed int containing the rgb color components.</returns>
inline unsigned int bilinear_sample(const unsigned int *colorImage, float x, float y, int imageWidth, int imageHeight)
{
    const float half = 0.5f;

    const unsigned int xi = static_cast<unsigned int>(x - half);
    const unsigned int yi = static_cast<unsigned int>(y - half);

    const float xf = x - half - static_cast<float>(xi);
    const float yf = y - half - static_cast<float>(yi);

    const unsigned int posax = clamp<unsigned int>(xi, 0, imageWidth - 1);
    const unsigned int posay = clamp<unsigned int>(yi, 0, imageHeight - 1);

    const unsigned int posbx = clamp<unsigned int>(xi+1, 0, imageWidth - 1);
    const unsigned int posby = clamp<unsigned int>(yi+1, 0, imageHeight - 1);

    // Load the corners
    Vector3 d00 = load_color(colorImage, posax, posay, imageWidth);
    Vector3 d01 = load_color(colorImage, posax, posby, imageWidth);
    Vector3 d10 = load_color(colorImage, posbx, posay, imageWidth);
    Vector3 d11 = load_color(colorImage, posbx, posby, imageWidth);

    // Interpolate over x
    auto dx0 = lerp_color(d00, d10, xf);
    auto dx1 = lerp_color(d01, d11, xf);

    // Interpolate over y
    auto dxy = lerp_color(dx0, dx1, yf);

    return (255 << 24 | static_cast<unsigned int>(dxy.z) << 16 | static_cast<unsigned int>(dxy.y) << 8 | static_cast<unsigned int>(dxy.x) );  // always full alpha
}

/// <summary>
/// Calculate the squared difference between two Vector3 vertices
/// </summary>
/// <param name="v1">The first vertex.</param>
/// <param name="v2">The second vertex.</param>
/// <returns>Returns the squared difference.</returns>
inline float squared_difference(const Vector3 &v1, const Vector3 &v2)
{
    float dx = v1.x-v2.x;
    float dy = v1.y-v2.y;
    float dz = v1.z-v2.z;
    return (dx*dx) + (dy*dy) + (dz*dz);
}

/// <summary>
/// Calculate the distance between two Vector3 vertices
/// </summary>
/// <param name="v1">The first vertex.</param>
/// <param name="v2">The second vertex.</param>
/// <returns>Returns the distance.</returns>
inline float distance(const Vector3 &v1, const Vector3 &v2)
{
    return sqrtf(squared_difference(v1, v2));
}

/// <summary>
/// Calculate the normalized dot product between two vectors.
/// Must be normalized input Vector3s.
/// Output: 1 if parallel, same dir, 0 if 90 degrees, -1 if parallel, looking opposite direction 
/// </summary>
/// <param name="v1">The first vector.</param>
/// <param name="v2">The second vector.</param>
/// <returns>Returns the dot product.</returns>
inline float dot_normalized(const Vector3 &v1, const Vector3 &v2)
{
    return (v1.x*v2.x) + (v1.y*v2.y) + (v1.z*v2.z);
}

/// <summary>
/// Transform a vertex in the world coordinate system by the worldToCamera pose,
/// into the camera coordinate system.
/// </summary>
/// <param name="v1">The vertex to transform.</param>
/// <param name="worldToCamera">The worldToCamera pose.</param>
/// <returns>Returns the transformed vertex.</returns>
inline Vector3 transform(const Vector3 &v1, const Matrix4 &worldToCamera)
{
    // Transform the point from the global frame into the local camera frame.
    Vector3 R;

    R.x = worldToCamera.M41 + (worldToCamera.M11 * v1.x) + (worldToCamera.M21 * v1.y) + (worldToCamera.M31 * v1.z);
    R.y = worldToCamera.M42 + (worldToCamera.M12 * v1.x) + (worldToCamera.M22 * v1.y) + (worldToCamera.M32 * v1.z);
    R.z = worldToCamera.M43 + (worldToCamera.M13 * v1.x) + (worldToCamera.M23 * v1.y) + (worldToCamera.M33 * v1.z);

    return R;
}

/// <summary>
/// Project a 3D vertex in the world coordinate system into a 2D camera image,
/// given its known intrinsic parameters and camera pose.
/// </summary>
/// <param name="v1">The vertex to transform.</param>
/// <param name="flx">The focal length in the x axis, in pixels.</param>
/// <param name="fly">The focal length in the y axis, in pixels.</param>
/// <param name="ppx">The principal point of the image in the x axis, in pixels.</param>
/// <param name="ppy">The principal point of the image in the y axis, in pixels.</param>
/// <param name="worldToCamera">The worldToCamera pose.</param>
/// <returns>Returns the 3D vertex transformed into a pixel in the 2D camera image.</returns>
inline Vector3 fast_project(const Vector3 &v1, float flx, float fly, float ppx, float ppy, const Matrix4 &worldToCamera) 
{
    // Transform from world to camera coordinate system
    Vector3 R = transform(v1, worldToCamera);

    Vector3 uv;
    uv.x = R.x / R.z;
    uv.y = R.y / R.z;

    // Project from camera plane in world to image
    uv.x = ppx +  flx * uv.x;
    uv.y = ppy +  fly * uv.y;

    uv.z = R.z;
    return uv;
}
