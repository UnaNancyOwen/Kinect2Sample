//------------------------------------------------------------------------------
// <copyright file="KinectFusionHelper.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// System includes
//#include "stdafx.h"
#include <atlbase.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <locale>
#include <codecvt>

#pragma warning(push)
#pragma warning(disable:6255)
#pragma warning(disable:6263)
#pragma warning(disable:4995)
#include "ppl.h"
#pragma warning(pop)

// Project includes
#include "KinectFusionHelper.h"

/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
void SetIdentityMatrix(Matrix4 &mat)
{
    mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
    mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
    mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
    mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

/// <summary>
/// Extract translation Vector3 from the Matrix4 4x4 transformation in M41,M42,M43
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <param name="translation">Array of 3 floating point values for translation.</param>
void ExtractVector3Translation(const Matrix4 &transform, _Out_cap_c_(3) float *translation)
{
    translation[0] = transform.M41;
    translation[1] = transform.M42;
    translation[2] = transform.M43;
}

/// <summary>
/// Extract translation Vector3 from the 4x4 Matrix in M41,M42,M43
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <returns>Returns a Vector3 containing the translation.</returns>
Vector3 ExtractVector3Translation(const Matrix4 &transform)
{
    Vector3 translation;
    translation.x = transform.M41;
    translation.y = transform.M42;
    translation.z = transform.M43;
    return translation;
}

/// <summary>
/// Extract 3x3 rotation from the 4x4 Matrix and return in new Matrix4
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <returns>Returns a Matrix4 containing the rotation.</returns>
Matrix4 Extract3x3Rotation(const Matrix4 &transform)
{
    Matrix4 rotation;

    rotation.M11 = transform.M11;
    rotation.M12 = transform.M12;
    rotation.M13 = transform.M13;
    rotation.M14 = 0;

    rotation.M21 = transform.M21;
    rotation.M22 = transform.M22;
    rotation.M23 = transform.M23;
    rotation.M24 = 0;

    rotation.M31 = transform.M31;
    rotation.M32 = transform.M32;
    rotation.M33 = transform.M33;
    rotation.M34 = 0;

    rotation.M41 = 0;
    rotation.M42 = 0;
    rotation.M43 = 0;
    rotation.M44 = 1;

    return rotation;
}

/// <summary>
/// Extract 3x3 rotation matrix from the Matrix4 4x4 transformation:
/// Then convert to Euler angles.
/// </summary>
/// <param name="transform">The transform matrix.</param>
/// <param name="rotation">Array of 3 floating point values for euler angles.</param>
void ExtractRot2Euler(const Matrix4 &transform, _Out_cap_c_(3) float *rotation)
{
    float phi = atan2f(transform.M23, transform.M33);
    float theta = asinf(-transform.M13);
    float psi = atan2f(transform.M12, transform.M11);

    rotation[0] = phi;    // This is rotation about x,y,z, or pitch, yaw, roll respectively
    rotation[1] = theta;
    rotation[2] = psi;
}

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
bool CameraTransformFailed(const Matrix4 &T_initial, const Matrix4 &T_final, float maxTrans, float maxRotDegrees)
{
    // Check if the transform is too far out to be reasonable 
    float deltaTrans = maxTrans;
    float angDeg = maxRotDegrees;
    float deltaRot = (angDeg * (float)M_PI) / 180.0f;

    // Calculate the deltas
    float eulerInitial[3];
    float eulerFinal[3];

    ExtractRot2Euler(T_initial, eulerInitial);
    ExtractRot2Euler(T_final, eulerFinal);

    float transInitial[3];
    float transFinal[3];

    ExtractVector3Translation(T_initial, transInitial);
    ExtractVector3Translation(T_final, transFinal);

    bool failRot = false;
    bool failTrans = false;

    float rDeltas[3];
    float tDeltas[3];

    static const float pi = static_cast<float>(M_PI);

    for (int i = 0; i < 3; i++)
    {
        // Handle when one angle is near PI, and the other is near -PI.
        if (eulerInitial[i] >= (pi - deltaRot) && eulerFinal[i] < (deltaRot - pi))
        {
            eulerInitial[i] -= pi * 2;
        }
        else if (eulerFinal[i] >= (pi - deltaRot) && eulerInitial[i] < (deltaRot - pi))
        {
            eulerFinal[i] -= pi * 2;
        }

        rDeltas[i] = eulerInitial[i] - eulerFinal[i];
        tDeltas[i] = transInitial[i] - transFinal[i];

        if (fabs(rDeltas[i]) > deltaRot)
        {
            failRot = true;
            break;
        }
        if (fabs(tDeltas[i]) > deltaTrans)
        {
            failTrans = true;
            break;
        }
    }

    return failRot || failTrans;
}

/// <summary>
/// Invert/Transpose the 3x3 Rotation Matrix Component of a 4x4 matrix
/// </summary>
/// <param name="rot">The rotation matrix to invert.</param>
void InvertRotation(Matrix4 &rot)
{
    // Invert equivalent to a transpose for 3x3 rotation rotrices when orthogonal
    float tmp = rot.M12;
    rot.M12 = rot.M21;
    rot.M21 = tmp;

    tmp = rot.M13;
    rot.M13 = rot.M31;
    rot.M31 = tmp;

    tmp = rot.M23;
    rot.M23 = rot.M32;
    rot.M32 = tmp;
}

/// <summary>
/// Negate the 3x3 Rotation Matrix Component of a 4x4 matrix
/// </summary>
/// <param name="rot">The rotation matrix to negate.</param>
void NegateRotation(Matrix4 &rot)
{
    rot.M11 = -rot.M11;
    rot.M12 = -rot.M12;
    rot.M13 = -rot.M13;

    rot.M21 = -rot.M21;
    rot.M22 = -rot.M22;
    rot.M23 = -rot.M23;

    rot.M31 = -rot.M31;
    rot.M32 = -rot.M32;
    rot.M33 = -rot.M33;
}

/// <summary>
/// Rotate a vector with the 3x3 Rotation Matrix Component of a 4x4 matrix
/// </summary>
/// <param name="vec">The Vector3 to rotate.</param>
/// <param name="rot">Rotation matrix.</param>
Vector3 RotateVector(const Vector3 &vec, const Matrix4 & rot)
{
    // we only use the rotation component here
    Vector3 result;

    result.x = (rot.M11 * vec.x) + (rot.M21 * vec.y) + (rot.M31 * vec.z);
    result.y = (rot.M12 * vec.x) + (rot.M22 * vec.y) + (rot.M32 * vec.z);
    result.z = (rot.M13 * vec.x) + (rot.M23 * vec.y) + (rot.M33 * vec.z);

    return result;
}
/// <summary>
/// Invert Matrix4 Pose either from WorldToCameraTransform (view) matrix to CameraToWorldTransform camera pose matrix (world/SE3) or vice versa
/// </summary>
/// <param name="transform">The camera pose transform matrix.</param>
/// <returns>Returns a Matrix4 containing the inverted camera pose.</returns>
Matrix4 InvertMatrix4Pose(const Matrix4 &transform)
{
    // Given the SE3 world transform transform T = [R|t], the inverse view transform matrix is simply:
    // T^-1 = [R^T | -R^T . t ]
    // This also works the opposite way to get the world transform, given the view transform matrix.
    Matrix4 rotation = Extract3x3Rotation(transform);

    Matrix4 invRotation = rotation;
    InvertRotation(invRotation);  // invert(transpose) 3x3 rotation

    Matrix4 negRotation = invRotation;
    NegateRotation(negRotation);  // negate 3x3 rotation

    Vector3 translation = ExtractVector3Translation(transform);
    Vector3 invTranslation = RotateVector(translation, negRotation);

    // Add the translation back in
    invRotation.M41 = invTranslation.x;
    invRotation.M42 = invTranslation.y;
    invRotation.M43 = invTranslation.z;

    return invRotation;
}

/// <summary>
/// Write Binary .STL file
/// see http://en.wikipedia.org/wiki/STL_(file_format) for STL format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteBinarySTLMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ)
{
    HRESULT hr = S_OK;

    if (NULL == mesh)
    {
        return E_INVALIDARG;
    }

    unsigned int numVertices = mesh->VertexCount();
    unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
    unsigned int numTriangles = numVertices / 3;

    if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 || numVertices != numTriangleIndices)
    {
        return E_INVALIDARG;
    }

    const Vector3 *vertices = NULL;
    hr = mesh->GetVertices(&vertices);
    if (FAILED(hr))
    {
        return hr;
    }

    const Vector3 *normals = NULL;
    hr = mesh->GetNormals(&normals);
    if (FAILED(hr))
    {
        return hr;
    }

    const int *triangleIndices = NULL;
    hr = mesh->GetTriangleIndices(&triangleIndices);
    if (FAILED(hr))
    {
        return hr;
    }

    // Open File
    std::string filename  = std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(lpOleFileName);
    FILE *meshFile = NULL;
    errno_t err = fopen_s(&meshFile, filename.c_str(), "wb");

    // Could not open file for writing - return
    if (0 != err || NULL == meshFile)
    {
        return E_ACCESSDENIED;
    }

    // Write the header line
    const unsigned char header[80] = {0};   // initialize all values to 0
    fwrite(&header, sizeof(unsigned char), ARRAYSIZE(header), meshFile);

    // Write number of triangles
    fwrite(&numTriangles, sizeof(int), 1, meshFile);

    // Sequentially write the normal, 3 vertices of the triangle and attribute, for each triangle
    for (unsigned int t=0; t < numTriangles; ++t)
    {
        Vector3 normal = normals[t*3];

        if (flipYZ)
        {
            normal.y = -normal.y;
            normal.z = -normal.z;
        }

        // Write normal
        fwrite(&normal, sizeof(float), 3, meshFile);

        // Write vertices
        for (unsigned int v=0; v<3; v++)
        {
            Vector3 vertex = vertices[(t*3) + v];

            if (flipYZ)
            {
                vertex.y = -vertex.y;
                vertex.z = -vertex.z;
            }

            fwrite(&vertex, sizeof(float), 3, meshFile);
        }

        unsigned short attribute = 0;
        fwrite(&attribute, sizeof(unsigned short), 1, meshFile);
    }

    fflush(meshFile);
    fclose(meshFile);

    return hr;
}

/// <summary>
/// Write ASCII Wavefront .OBJ file
/// See http://en.wikipedia.org/wiki/Wavefront_.obj_file for .OBJ format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiObjMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ)
{
    HRESULT hr = S_OK;

    if (NULL == mesh)
    {
        return E_INVALIDARG;
    }

    unsigned int numVertices = mesh->VertexCount();
    unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
    unsigned int numTriangles = numVertices / 3;

    if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 || numVertices != numTriangleIndices)
    {
        return E_INVALIDARG;
    }

    const Vector3 *vertices = NULL;
    hr = mesh->GetVertices(&vertices);
    if (FAILED(hr))
    {
        return hr;
    }

    const Vector3 *normals = NULL;
    hr = mesh->GetNormals(&normals);
    if (FAILED(hr))
    {
        return hr;
    }

    const int *triangleIndices = NULL;
    hr = mesh->GetTriangleIndices(&triangleIndices);
    if (FAILED(hr))
    {
        return hr;
    }

    // Open File
    std::string filename  = std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(lpOleFileName);
    FILE *meshFile = NULL;
    errno_t err = fopen_s(&meshFile, filename.c_str(), "wt");

    // Could not open file for writing - return
    if (0 != err || NULL == meshFile)
    {
        return E_ACCESSDENIED;
    }

    // Write the header line
    std::string header = "#\n# OBJ file created by Microsoft Kinect Fusion\n#\n";
    fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

    const unsigned int bufSize = MAX_PATH*3;
    char outStr[bufSize];
    int written = 0;

    if (flipYZ)
    {
        // Sequentially write the 3 vertices of the triangle, for each triangle
        for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "v %f %f %f\nv %f %f %f\nv %f %f %f\n", 
                vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z, 
                vertices[vertexIndex+1].x, -vertices[vertexIndex+1].y, -vertices[vertexIndex+1].z, 
                vertices[vertexIndex+2].x, -vertices[vertexIndex+2].y, -vertices[vertexIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }

        // Sequentially write the 3 normals of the triangle, for each triangle
        for (unsigned int t=0, normalIndex=0; t < numTriangles; ++t, normalIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "n %f %f %f\nn %f %f %f\nn %f %f %f\n", 
                normals[normalIndex].x, -normals[normalIndex].y, -normals[normalIndex].z, 
                normals[normalIndex+1].x, -normals[normalIndex+1].y, -normals[normalIndex+1].z, 
                normals[normalIndex+2].x, -normals[normalIndex+2].y, -normals[normalIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }
    }
    else
    {
        // Sequentially write the 3 vertices of the triangle, for each triangle
        for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "v %f %f %f\nv %f %f %f\nv %f %f %f\n", 
                vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z, 
                vertices[vertexIndex+1].x, vertices[vertexIndex+1].y, vertices[vertexIndex+1].z, 
                vertices[vertexIndex+2].x, vertices[vertexIndex+2].y, vertices[vertexIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }

        // Sequentially write the 3 normals of the triangle, for each triangle
        for (unsigned int t=0, normalIndex=0; t < numTriangles; ++t, normalIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "n %f %f %f\nn %f %f %f\nn %f %f %f\n", 
                normals[normalIndex].x, normals[normalIndex].y, normals[normalIndex].z, 
                normals[normalIndex+1].x, normals[normalIndex+1].y, normals[normalIndex+1].z, 
                normals[normalIndex+2].x, normals[normalIndex+2].y, normals[normalIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }
    }

    // Sequentially write the 3 vertex indices of the triangle face, for each triangle
    // Note this is typically 1-indexed in an OBJ file when using absolute referencing!
    for (unsigned int t=0, baseIndex=1; t < numTriangles; ++t, baseIndex += 3) // Start at baseIndex=1 for the 1-based indexing.
    {
        written = sprintf_s(outStr, bufSize, "f %u//%u %u//%u %u//%u\n", 
            baseIndex, baseIndex, baseIndex+1, baseIndex+1, baseIndex+2, baseIndex+2);
        fwrite(outStr, sizeof(char), written, meshFile);
    }

    // Note: we do not have texcoords to store, if we did, we would put the index of the texcoords between the vertex and normal indices (i.e. between the two slashes //) in the string above
    fflush(meshFile);
    fclose(meshFile);

    return hr;
}

/// <summary>
/// Write ASCII .PLY file
/// See http://paulbourke.net/dataformats/ply/ for .PLY format
/// </summary>
/// <param name="mesh">The Kinect Fusion mesh object.</param>
/// <param name="lpOleFileName">The full path and filename of the file to save.</param>
/// <param name="flipYZ">Flag to determine whether the Y and Z values are flipped on save.</param>
/// <param name="outputColor">Set this true to write out the surface color to the file when it has been captured.</param>
/// <returns>indicates success or failure</returns>
HRESULT WriteAsciiPlyMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ, bool outputColor)
{
    HRESULT hr = S_OK;

    if (NULL == mesh)
    {
        return E_INVALIDARG;
    }

    unsigned int numVertices = mesh->VertexCount();
    unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
    unsigned int numTriangles = numVertices / 3;
    unsigned int numColors = mesh->ColorCount();

    if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 
        || numVertices != numTriangleIndices || (outputColor && numVertices != numColors))
    {
        return E_INVALIDARG;
    }

    const Vector3 *vertices = NULL;
    hr = mesh->GetVertices(&vertices);
    if (FAILED(hr))
    {
        return hr;
    }

    const int *triangleIndices = NULL;
    hr = mesh->GetTriangleIndices(&triangleIndices);
    if (FAILED(hr))
    {
        return hr;
    }

    const int *colors = NULL;
    if (outputColor)
    {
        hr = mesh->GetColors(&colors);
        if (FAILED(hr))
        {
            return hr;
        }
    }

    // Open File
    std::string filename  = std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(lpOleFileName);
    FILE *meshFile = NULL;
    errno_t err = fopen_s(&meshFile, filename.c_str(), "wt");

    // Could not open file for writing - return
    if (0 != err || NULL == meshFile)
    {
        return E_ACCESSDENIED;
    }

    // Write the header line
    std::string header = "ply\nformat ascii 1.0\ncomment file created by Microsoft Kinect Fusion\n";
    fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

    const unsigned int bufSize = MAX_PATH*3;
    char outStr[bufSize];
    int written = 0;

    if (outputColor)
    {
        // Elements are: x,y,z, r,g,b
        written = sprintf_s(outStr, bufSize, "element vertex %u\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n", numVertices);
        fwrite(outStr, sizeof(char), written, meshFile);
    }
    else
    {
        // Elements are: x,y,z
        written = sprintf_s(outStr, bufSize, "element vertex %u\nproperty float x\nproperty float y\nproperty float z\n", numVertices);
        fwrite(outStr, sizeof(char), written, meshFile);
    }

    written = sprintf_s(outStr, bufSize, "element face %u\nproperty list uchar int vertex_index\nend_header\n", numTriangles);
    fwrite(outStr, sizeof(char), written, meshFile);

    if (flipYZ)
    {
        if (outputColor)
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
            {
                unsigned int color0 = colors[vertexIndex];
                unsigned int color1 = colors[vertexIndex+1];
                unsigned int color2 = colors[vertexIndex+2];

                written = sprintf_s(outStr, bufSize, "%f %f %f %u %u %u\n%f %f %f %u %u %u\n%f %f %f %u %u %u\n", 
                    vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z, 
                    ((color0 >> 16) & 255), ((color0 >> 8) & 255), (color0 & 255), 
                    vertices[vertexIndex+1].x, -vertices[vertexIndex+1].y, -vertices[vertexIndex+1].z,
                    ((color1 >> 16) & 255), ((color1 >> 8) & 255), (color1 & 255), 
                    vertices[vertexIndex+2].x, -vertices[vertexIndex+2].y, -vertices[vertexIndex+2].z,
                    ((color2 >> 16) & 255), ((color2 >> 8) & 255), (color2 & 255));

                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
        else
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
            {
                written = sprintf_s(outStr, bufSize, "%f %f %f\n%f %f %f\n%f %f %f\n", 
                    vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z, 
                    vertices[vertexIndex+1].x, -vertices[vertexIndex+1].y, -vertices[vertexIndex+1].z, 
                    vertices[vertexIndex+2].x, -vertices[vertexIndex+2].y, -vertices[vertexIndex+2].z);
                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
    }
    else
    {
        if (outputColor)
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
            {
                unsigned int color0 = colors[vertexIndex];
                unsigned int color1 = colors[vertexIndex+1];
                unsigned int color2 = colors[vertexIndex+2];

                written = sprintf_s(outStr, bufSize, "%f %f %f %u %u %u\n%f %f %f %u %u %u\n%f %f %f %u %u %u\n", 
                    vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z, 
                    ((color0 >> 16) & 255), ((color0 >> 8) & 255), (color0 & 255), 
                    vertices[vertexIndex+1].x, vertices[vertexIndex+1].y, vertices[vertexIndex+1].z,
                    ((color1 >> 16) & 255), ((color1 >> 8) & 255), (color1 & 255), 
                    vertices[vertexIndex+2].x, vertices[vertexIndex+2].y, vertices[vertexIndex+2].z,
                    ((color2 >> 16) & 255), ((color2 >> 8) & 255), (color2 & 255));

                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
        else
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
            {
                written = sprintf_s(outStr, bufSize, "%f %f %f\n%f %f %f\n%f %f %f\n", 
                    vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z, 
                    vertices[vertexIndex+1].x, vertices[vertexIndex+1].y, vertices[vertexIndex+1].z,
                    vertices[vertexIndex+2].x, vertices[vertexIndex+2].y, vertices[vertexIndex+2].z);
                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
    }

    // Sequentially write the 3 vertex indices of the triangle face, for each triangle (0-referenced in PLY)
    for (unsigned int t=0, baseIndex=0; t < numTriangles; ++t, baseIndex += 3)
    {
        written = sprintf_s(outStr, bufSize, "3 %u %u %u\n", baseIndex, baseIndex+1, baseIndex+2);
        fwrite(outStr, sizeof(char), written, meshFile);
    }

    fflush(meshFile);
    fclose(meshFile);

    return hr;
}

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
HRESULT WriteTexturedeAsciiObjMeshFile(INuiFusionColorMesh *mesh, LPOLESTR lpOleFileName, bool flipYZ, NUI_FUSION_IMAGE_FRAME *pTexture, const std::vector<Vector3> &texcoords)
{
    HRESULT hr = S_OK;

    if (nullptr == mesh || nullptr == pTexture)
    {
        return E_INVALIDARG;
    }

    unsigned int numVertices = mesh->VertexCount();
    unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
    unsigned int numTriangles = numVertices / 3;

    if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3 || numVertices != numTriangleIndices)
    {
        return E_INVALIDARG;
    }

    const Vector3 *vertices = NULL;
    hr = mesh->GetVertices(&vertices);
    if (FAILED(hr))
    {
        return hr;
    }

    const Vector3 *normals = NULL;
    hr = mesh->GetNormals(&normals);
    if (FAILED(hr))
    {
        return hr;
    }

    const int *triangleIndices = NULL;
    hr = mesh->GetTriangleIndices(&triangleIndices);
    if (FAILED(hr))
    {
        return hr;
    }

    // Open File
    std::string filename  = std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(lpOleFileName);
    FILE *meshFile = NULL;
    errno_t err = fopen_s(&meshFile, filename.c_str(), "wt");

    // Could not open file for writing - return
    if (0 != err || NULL == meshFile)
    {
        return E_ACCESSDENIED;
    }

    // Split the name and extension
    std::string mtlfilename = filename + ".mtl";

    // Open the material file
    FILE *mtlFile = NULL;
    err = fopen_s(&mtlFile, mtlfilename.c_str(), "wt");

    // Could not open file for writing - return
    if (0 != err || NULL == mtlFile)
    {
        if (meshFile)
        {
            fclose(meshFile);
        }
        return E_ACCESSDENIED;
    }

    // Write the header line
    std::string header = "#\n# OBJ file created by Microsoft Kinect Fusion\n#\n";
    fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

    // Split to extract path
    std::string::size_type pos = filename.find_last_of("\\", filename.length());
    std::string filenamePath = filename.substr(0,pos+1);
    std::string filenameRelative = filename.substr(pos+1);

    // Write that we have an accompanying material file
    std::string mtlfile = "mtllib " + filenameRelative + ".mtl\n";
    fwrite(mtlfile.c_str(), sizeof(char), mtlfile.length(), meshFile);

    const unsigned int bufSize = MAX_PATH*3;
    char outStr[bufSize];
    int written = 0;

    if (flipYZ)
    {
        // Sequentially write the 3 vertices of the triangle, for each triangle
        for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "v %f %f %f\nv %f %f %f\nv %f %f %f\n", 
                vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z, 
                vertices[vertexIndex+1].x, -vertices[vertexIndex+1].y, -vertices[vertexIndex+1].z, 
                vertices[vertexIndex+2].x, -vertices[vertexIndex+2].y, -vertices[vertexIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }

        // Sequentially write the 3 normals of the triangle, for each triangle
        for (unsigned int t=0, normalIndex=0; t < numTriangles; ++t, normalIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "n %f %f %f\nn %f %f %f\nn %f %f %f\n", 
                normals[normalIndex].x, -normals[normalIndex].y, -normals[normalIndex].z, 
                normals[normalIndex+1].x, -normals[normalIndex+1].y, -normals[normalIndex+1].z, 
                normals[normalIndex+2].x, -normals[normalIndex+2].y, -normals[normalIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }
    }
    else
    {
        // Sequentially write the 3 vertices of the triangle, for each triangle
        for (unsigned int t=0, vertexIndex=0; t < numTriangles; ++t, vertexIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "v %f %f %f\nv %f %f %f\nv %f %f %f\n", 
                vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z, 
                vertices[vertexIndex+1].x, vertices[vertexIndex+1].y, vertices[vertexIndex+1].z, 
                vertices[vertexIndex+2].x, vertices[vertexIndex+2].y, vertices[vertexIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }

        // Sequentially write the 3 normals of the triangle, for each triangle
        for (unsigned int t=0, normalIndex=0; t < numTriangles; ++t, normalIndex += 3)
        {
            written = sprintf_s(outStr, bufSize, "n %f %f %f\nn %f %f %f\nn %f %f %f\n", 
                normals[normalIndex].x, normals[normalIndex].y, normals[normalIndex].z, 
                normals[normalIndex+1].x, normals[normalIndex+1].y, normals[normalIndex+1].z, 
                normals[normalIndex+2].x, normals[normalIndex+2].y, normals[normalIndex+2].z);
            fwrite(outStr, sizeof(char), written, meshFile);
        }
    }

    // Sequentially write the 3 texture coordinates of the triangle, for each triangle
    for (unsigned int t=0, texcoordIndex=0; t < numTriangles; ++t, texcoordIndex += 3)
    {
        written = sprintf_s(outStr, bufSize, "vt %f %f\nvt %f %f\nvt %f %f\n", 
            texcoords[texcoordIndex].x, texcoords[texcoordIndex].y, 
            texcoords[texcoordIndex+1].x, texcoords[texcoordIndex+1].y, 
            texcoords[texcoordIndex+2].x, texcoords[texcoordIndex+2].y);
        fwrite(outStr, sizeof(char), written, meshFile);
    }

    // Write that we are using material0 (i.e. the texture)
    std::string material = "usemtl material0\n";
    fwrite(material.c_str(), sizeof(char), material.length(), meshFile);


    // Sequentially write the 3 vertex indices of the triangle face, for each triangle
    // Note this is typically 1-indexed in an OBJ file when using absolute referencing!
    for (unsigned int t=0, baseIndex=1; t < numTriangles; ++t, baseIndex += 3) // Start at baseIndex=1 for the 1-based indexing.
    {
        written = sprintf_s(outStr, bufSize, "f %u/%u/%u %u/%u/%u %u/%u/%u\n", 
            baseIndex, baseIndex, baseIndex, baseIndex+1, baseIndex+1, baseIndex+1, baseIndex+2, baseIndex+2, baseIndex+2);
        fwrite(outStr, sizeof(char), written, meshFile);
    }

    fflush(meshFile);
    fclose(meshFile);

    // Write the material description file
    header = "#\n# OBJ file created by Microsoft Kinect Fusion\n#\n";
    fwrite(header.c_str(), sizeof(char), header.length(), mtlFile);

    material = "newmtl material0\n";
    fwrite(material.c_str(), sizeof(char), material.length(), mtlFile);

    // Create the texture filename
    std::string textureFilename = filenameRelative + ".bmp";

    // Write the generic materials definition together with the texture filename.
    std::string mtldescription ="Ka 1.000000 1.000000 1.000000\n"
        "Kd 1.000000 1.000000 1.000000\n" 
        "Ks 0.000000 0.000000 0.000000\n"
        "Tr 1.000000\n"
        "illum 1\n"
        "Ns 0.000000\n"
        "map_Kd " + textureFilename + "\n";
    fwrite(mtldescription.c_str(), sizeof(char), mtldescription.length(), mtlFile);

    fflush(mtlFile);
    fclose(mtlFile);

    std::string textureFilenamePath = filenamePath + textureFilename;
    CA2W pszW( textureFilenamePath.c_str() );

    // Write out the texture
    NUI_FUSION_BUFFER *fusionColorBuffer = pTexture->pFrameBuffer;

    if (fusionColorBuffer->Pitch == 0)
    {
        return E_FAIL;
    }

    // Save the texture
    hr = SaveBMPFile(pszW, fusionColorBuffer->pBits, pTexture->width, pTexture->height);

    return hr;
}

/// <summary>
/// Returns whether this is running as a 32 or 64bit application.
/// </summary>
/// <returns>TRUE indicates a 64bit app.</returns>
BOOL Is64BitApp()
{
#if defined(_WIN64)
    // If _WIN64 is defined, we are a 64-bit version as 
    // this will only be defined on Win64
    return TRUE; 
#else
    // 32-bit programs run on both 32-bit and 64-bit Windows with WOW64, 
    // however the restrictions are the same for our application.
    return FALSE;
#endif
}

/// <summary>
/// Write 32bit BMP image file
/// </summary>
/// <param name="pszFile">The full path and filename of the file to save.</param>
/// <param name="pImageBytes">A pointer to the image bytes to save.</param>
/// <param name="width">The width of the image to save.</param>
/// <param name="height">The width of the image to save.</param>
/// <returns>indicates success or failure</returns>
HRESULT SaveBMPFile(LPCWSTR pszFile, const byte *pImageBytes, unsigned int width, unsigned int height)
{
    // Each pixel is 8 bits per color, 4 values interleaved (R,G,B,A) = 32 bits total
    const WORD cBitsPerColor = 8;
    const WORD cColorValues = 4;
    WORD cColorBits = cBitsPerColor * cColorValues;

    // No need to pad the array as we already have 32bit pixels.
    DWORD imageByteSize = static_cast<DWORD>(width) * static_cast<DWORD>(height) * static_cast<DWORD>(cColorBits/8);

    // However, we may need to swap R and B byte ordering.

    // Set headers
    BITMAPFILEHEADER bmfh;
    BITMAPINFOHEADER info;
    memset (&bmfh, 0, sizeof(BITMAPFILEHEADER));
    memset (&info, 0, sizeof(BITMAPINFOHEADER));

    bmfh.bfType = 0x4d42;   // Magic number "BM"
    bmfh.bfReserved1 = 0;
    bmfh.bfReserved2 = 0;
    bmfh.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + imageByteSize;
    bmfh.bfOffBits = 0x36;

    info.biSize = sizeof(BITMAPINFOHEADER);
    info.biWidth = width;
    info.biHeight = height;
    info.biPlanes = 1;
    info.biBitCount = cColorBits;
    info.biCompression = BI_RGB;
    info.biSizeImage = 0;
    info.biXPelsPerMeter = 0x0ec4;
    info.biYPelsPerMeter = 0x0ec4;
    info.biClrUsed = 0;    
    info.biClrImportant = 0; 

    // Open file and write
    HANDLE file = CreateFileW(pszFile, GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (INVALID_HANDLE_VALUE == file)
    {
        return E_INVALIDARG;
    }

    unsigned long bytesWritten = 0;

    if (FALSE == WriteFile(file, &bmfh, sizeof(BITMAPFILEHEADER), &bytesWritten, NULL))
    {
        CloseHandle(file);
        return E_FAIL;
    }

    if (FALSE == WriteFile(file, &info, sizeof(BITMAPINFOHEADER), &bytesWritten, NULL))
    {
        CloseHandle(file);
        return E_FAIL;
    }

    if (FALSE == WriteFile(file, pImageBytes, imageByteSize, &bytesWritten, NULL))
    {
        CloseHandle(file);
        return E_FAIL;
    }

    CloseHandle(file);

    return S_OK;
}

/// <summary>
/// Copy an image with identical sizes and parameters.
/// </summary>
/// <param name="pSrc">A pointer to the source image.</param>
/// <param name="pDest">A pointer to the destination image.</param>
/// <returns>indicates success or failure</returns>
HRESULT CopyImageFrame(const NUI_FUSION_IMAGE_FRAME *pSrc, const NUI_FUSION_IMAGE_FRAME *pDest)
{
    HRESULT hr = S_OK;

    if (nullptr == pSrc || nullptr == pSrc->pFrameBuffer || nullptr == pDest || nullptr == pDest->pFrameBuffer)
    {
        return E_INVALIDARG;
    }

    if (pSrc->imageType != pDest->imageType)
    {
        return E_INVALIDARG;
    }

    if (0 == pSrc->width || 0 == pSrc->height || pSrc->width != pDest->width || pSrc->height != pDest->height)
    {
        return E_NOINTERFACE;
    }

    NUI_FUSION_BUFFER *srcImageFrameBuffer = pSrc->pFrameBuffer;

    // Make sure we've received valid data
    if (srcImageFrameBuffer->Pitch != 0)
    {
        NUI_FUSION_BUFFER *destImageFrameBuffer = pDest->pFrameBuffer;

        // Make sure we've received valid data
        if (destImageFrameBuffer->Pitch != 0)
        {
            // Copy
            errno_t err = memcpy_s(
                destImageFrameBuffer->pBits, 
                destImageFrameBuffer->Pitch * pDest->height, 
                srcImageFrameBuffer->pBits, 
                srcImageFrameBuffer->Pitch * pSrc->height);

            if (0 != err)
            {
                hr = E_FAIL;
            }
        }
    }

    return hr;
}

/// <summary>
/// Color the residual/delta image from the AlignDepthFloatToReconstruction call
/// </summary>
/// <param name="pFloatDeltaFromReference">A pointer to the source FloatDeltaFromReference image.</param>
/// <param name="pShadedDeltaFromReference">A pointer to the destination color ShadedDeltaFromReference image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT ColorResiduals(const NUI_FUSION_IMAGE_FRAME *pFloatDeltaFromReference, const NUI_FUSION_IMAGE_FRAME *pShadedDeltaFromReference)
{
    if (nullptr == pShadedDeltaFromReference || 
        nullptr == pFloatDeltaFromReference)
    {
        return E_FAIL;
    }

    if (nullptr == pShadedDeltaFromReference->pFrameBuffer ||
        nullptr == pFloatDeltaFromReference->pFrameBuffer)
    {
        return E_NOINTERFACE;
    }

    if (pFloatDeltaFromReference->imageType !=  NUI_FUSION_IMAGE_TYPE_FLOAT || pShadedDeltaFromReference->imageType !=  NUI_FUSION_IMAGE_TYPE_COLOR)
    {
        return E_INVALIDARG;
    }

    unsigned int width = pShadedDeltaFromReference->width;
    unsigned int height = pShadedDeltaFromReference->height;

    if (width != pFloatDeltaFromReference->width 
        || height != pFloatDeltaFromReference->height)
    {
        return E_INVALIDARG;
    }

    if (pShadedDeltaFromReference->pFrameBuffer->Pitch == 0
        || pFloatDeltaFromReference->pFrameBuffer->Pitch == 0)
    {
        return E_INVALIDARG;
    }

    unsigned int *pColorBuffer = reinterpret_cast<unsigned int *>(pShadedDeltaFromReference->pFrameBuffer->pBits);
    const float *pFloatBuffer = reinterpret_cast<float *>(pFloatDeltaFromReference->pFrameBuffer->pBits);

    Concurrency::parallel_for(0u, height, [&](unsigned int y)
    {
        unsigned int* pColorRow = reinterpret_cast<unsigned int*>(reinterpret_cast<unsigned char*>(pColorBuffer) + (y * pShadedDeltaFromReference->pFrameBuffer->Pitch));
        const float* pFloatRow = reinterpret_cast<const float*>(reinterpret_cast<const unsigned char*>(pFloatBuffer) + (y * pFloatDeltaFromReference->pFrameBuffer->Pitch));

        for (unsigned int x = 0; x < width; ++x)
        {
            float residue = pFloatRow[x];
            unsigned int color = 0;

            if (residue <= 1.0f)   // Pixel byte ordering: ARGB
            {
                color |= (255 << 24);                                                                               // a
                color |= (static_cast<unsigned char>(255.0f * clamp(1.0f + residue, 0.0f, 1.0f)) << 16);            // r
                color |= (static_cast<unsigned char>(255.0f * clamp(1.0f - std::abs(residue), 0.0f, 1.0f)) << 8);   // g
                color |= (static_cast<unsigned char>(255.0f * clamp(1.0f - residue, 0.0f, 1.0f)));                  // b
            }

            pColorRow[x] = color;
        }
    });

    return S_OK;
}

/// <summary>
/// Calculate statistics on the residual/delta image from the AlignDepthFloatToReconstruction call.
/// </summary>
/// <param name="pFloatDeltaFromReference">A pointer to the source FloatDeltaFromReference image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT CalculateResidualStatistics(const NUI_FUSION_IMAGE_FRAME *pFloatDeltaFromReference, DeltaFromReferenceImageStatistics *stats)
{
    if (nullptr == pFloatDeltaFromReference || nullptr == stats)
    {
        return E_INVALIDARG;
    }

    if (nullptr == pFloatDeltaFromReference->pFrameBuffer)
    {
        return E_NOINTERFACE;
    }

    unsigned int width = pFloatDeltaFromReference->width;
    unsigned int height = pFloatDeltaFromReference->height;

    if (0 == width || 0 == height || pFloatDeltaFromReference->pFrameBuffer->Pitch == 0)
    {
        return E_INVALIDARG;
    }

    const float *pFloatBuffer = reinterpret_cast<float *>(pFloatDeltaFromReference->pFrameBuffer->pBits);

    // Measurement stats
    std::vector<unsigned int> zeroPixelsRow;
    std::vector<unsigned int> validPixelsRow;
    std::vector<unsigned int> invalidDepthOutsideVolumePixelsRow;
    std::vector<float> validPixelDistanceRow;

    zeroPixelsRow.resize(height, 0);
    validPixelsRow.resize(height, 0);
    invalidDepthOutsideVolumePixelsRow.resize(height, 0);
    validPixelDistanceRow.resize(height, 0);

    Concurrency::parallel_for(0u, height, [&](unsigned int y)
    {
        const float* pFloatRow = reinterpret_cast<const float*>(reinterpret_cast<const unsigned char*>(pFloatBuffer) + (y * pFloatDeltaFromReference->pFrameBuffer->Pitch));

        for (unsigned int x = 0; x < width; ++x)
        {
            float residue = pFloatRow[x];

            // If the depth was invalid or the depth back-projected outside the volume, the residual is set to 2.0f
            // However, if the voxel contents are 0 the residual will also return 0 here.
            if (residue == 0.0f)
            {
                ++zeroPixelsRow[y];
            }
            else if (residue == 2.0f)
            {
                ++invalidDepthOutsideVolumePixelsRow[y];
            }
            else if (residue <= 1.0f)   // Pixel byte ordering: ARGB
            {
                ++validPixelsRow[y];
                validPixelDistanceRow[y] += residue;
            }
        }
    });

    stats->validPixels = stats->zeroPixels = stats->invalidDepthOutsideVolumePixels = 0;
    stats->totalValidPixelsDistance = 0;
    stats->totalPixels = width * height;

    for (unsigned int y=0; y<height; ++y)
    {
        stats->zeroPixels += zeroPixelsRow[y];
        stats->validPixels += validPixelsRow[y];
        stats->invalidDepthOutsideVolumePixels += invalidDepthOutsideVolumePixelsRow[y];

        stats->totalValidPixelsDistance += validPixelDistanceRow[y];
    }

    return S_OK;
}

/// <summary>
/// Horizontally mirror a 32bit (color/float) image in-place.
/// </summary>
/// <param name="pImage">A pointer to the image to mirror.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT HorizontalMirror32bitImageInPlace(const NUI_FUSION_IMAGE_FRAME *pImage)
{
    if (nullptr == pImage || !(pImage->imageType == NUI_FUSION_IMAGE_TYPE_COLOR || pImage->imageType == NUI_FUSION_IMAGE_TYPE_FLOAT))
    {
        return E_INVALIDARG;
    }

    if (nullptr == pImage->pFrameBuffer)
    {
        return E_NOINTERFACE;
    }

    unsigned int width = pImage->width;
    unsigned int height = pImage->height;

    if (0 == width || 0 == height || pImage->pFrameBuffer->Pitch == 0)
    {
        return E_INVALIDARG;
    }

    unsigned int *rawPixels = reinterpret_cast<unsigned int *>(pImage->pFrameBuffer->pBits);

    Concurrency::parallel_for(0u, height, [&](unsigned int y)
    {
        unsigned int index = y * width;
        unsigned int mirrorIndex = index + width - 1;

        for (unsigned int x = 0; x < (width / 2); ++x, ++index, --mirrorIndex)
        {
            // In-place swap to mirror
            unsigned int temp = rawPixels[index];
            rawPixels[index] = rawPixels[mirrorIndex];
            rawPixels[mirrorIndex] = temp;
        }
    });

    return S_OK;
}

/// <summary>
/// Horizontally mirror a 32bit (color/float) image.
/// </summary>
/// <param name="pSrcImage">A pointer to the image to mirror.</param>
/// <param name="pDestImage">A pointer to the destination mirrored image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT HorizontalMirror32bitImage(const NUI_FUSION_IMAGE_FRAME *pSrcImage, const NUI_FUSION_IMAGE_FRAME *pDestImage)
{
    if (nullptr == pSrcImage || nullptr == pDestImage)
    {
        return E_INVALIDARG;
    }

    if (nullptr == pSrcImage->pFrameBuffer || nullptr == pDestImage->pFrameBuffer)
    {
        return E_NOINTERFACE;
    }

    if (!(pSrcImage->imageType == NUI_FUSION_IMAGE_TYPE_FLOAT || pSrcImage->imageType == NUI_FUSION_IMAGE_TYPE_COLOR) 
        || pSrcImage->imageType != pDestImage->imageType)
    {
        return E_INVALIDARG;
    }

    unsigned int width = pSrcImage->width;
    unsigned int height = pSrcImage->height;

    if (width != pDestImage->width || height != pDestImage->height)
    {
        return E_INVALIDARG;
    }

    if (pSrcImage->pFrameBuffer->Pitch == 0 || pDestImage->pFrameBuffer->Pitch == 0)
    {
        return E_INVALIDARG;
    }

    const unsigned int *pSrcBuffer = reinterpret_cast<const unsigned int *>(pSrcImage->pFrameBuffer->pBits);
    unsigned int *pDestBuffer = reinterpret_cast<unsigned int *>(pDestImage->pFrameBuffer->pBits);

    Concurrency::parallel_for(0u, height, [&](unsigned int y)
    {
        const unsigned int *pSrcRow = reinterpret_cast<const unsigned int*>(reinterpret_cast<const unsigned char*>(pSrcBuffer) + (y * pSrcImage->pFrameBuffer->Pitch));
        unsigned int *pDestRow = reinterpret_cast<unsigned int*>(reinterpret_cast<unsigned char*>(pDestBuffer) + (y * pDestImage->pFrameBuffer->Pitch));

        for (unsigned int x = 0, flippedX = width-1; x < width; ++x, --flippedX)
        {
            pDestRow[flippedX] = pSrcRow[x];
        }
    });

    return S_OK;
}

/// <summary>
/// Tests whether a resampling factor is valid.
/// </summary>
/// <param name="factor">The resampling factor.</param>
/// <returns>true if a valid resampling factor, otherwise false.</returns>
/// <remarks>
/// Valid resampling factors are powers of two between 1 and 16, inclusive.
/// </remarks>
static inline bool IsValidResampleFactor(unsigned int factor)
{
    return (1 == factor || 2 == factor || 4 == factor || 8 == factor || 16 == factor);
}

/// <summary>
/// Down sample color frame with nearest neighbor to the depth frame resolution
/// </summary>
/// <param name="src">The source color image.</param>
/// <param name="dest">The destination down sampled  image.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT DownsampleColorFrameToDepthResolution(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest)
{
    if (nullptr == src || nullptr == dest)
    {
        return E_INVALIDARG;
    }

    if (src->imageType != NUI_FUSION_IMAGE_TYPE_COLOR || src->imageType != dest->imageType 
        || src->width != 1920 || src->height != 1080 || dest->width != NUI_DEPTH_RAW_WIDTH || dest->height != NUI_DEPTH_RAW_HEIGHT)
    {
        return E_INVALIDARG;
    }

    NUI_FUSION_BUFFER *srcFrameBuffer = src->pFrameBuffer;
    NUI_FUSION_BUFFER *downsampledFloatFrameBuffer = dest->pFrameBuffer;

    float factor = 1080.0f / NUI_DEPTH_RAW_HEIGHT;

    // Make sure we've received valid data
    if (srcFrameBuffer->Pitch == 0 || downsampledFloatFrameBuffer->Pitch == 0)
    {
        return E_NOINTERFACE;
    }

    HRESULT hr = S_OK;
    float *srcValues = (float *)srcFrameBuffer->pBits;
    float *downsampledDestValues = (float *)downsampledFloatFrameBuffer->pBits;

    const unsigned int filledZeroMargin = 0;
    const unsigned int downsampledWidth = dest->width;
    const unsigned int srcImageWidth = src->width;

    ZeroMemory(downsampledDestValues, downsampledFloatFrameBuffer->Pitch * dest->height);
    Concurrency::parallel_for(filledZeroMargin, dest->height - filledZeroMargin, [=, &downsampledDestValues, &srcValues](unsigned int y)
    {
        unsigned int index = dest->width * y;
        for (unsigned int x=0; x < downsampledWidth; ++x, ++index)
        {
            int srcX = (int)(x * factor);
            int srcY = (int)(y * factor);
            int srcIndex = srcY * srcImageWidth + srcX;
            downsampledDestValues[index] = srcValues[srcIndex];
        }
    });

    return hr;
}

/// <summary>
/// Down sample color, depth float or point cloud frame with nearest neighbor
/// </summary>
/// <param name="src">The source depth float or pointcloud image.</param>
/// <param name="dest">The destination down sampled depth float or pointcloud image.</param>
/// <param name="factor">The down sample factor (1=just copy, 2=x/2,y/2, 4=x/4,y/4).</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT DownsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor)
{
    if (nullptr == src || nullptr == dest)
    {
        return E_INVALIDARG;
    }

    if (!(src->imageType == NUI_FUSION_IMAGE_TYPE_COLOR || src->imageType == NUI_FUSION_IMAGE_TYPE_FLOAT || src->imageType == NUI_FUSION_IMAGE_TYPE_POINT_CLOUD)
        || src->imageType != dest->imageType)
    {
        return E_INVALIDARG;
    }

    if (!IsValidResampleFactor(factor))
    {
        return E_INVALIDARG;
    }

    NUI_FUSION_BUFFER *srcFrameBuffer = src->pFrameBuffer;
    NUI_FUSION_BUFFER *downsampledFloatFrameBuffer = dest->pFrameBuffer;

    unsigned int downsampledWidth = src->width / factor;
    unsigned int downsampleHeight = src->height / factor;

    if (1 == factor && srcFrameBuffer->Pitch * src->height != downsampledFloatFrameBuffer->Pitch * dest->height)
    {
        return E_INVALIDARG;
    }
    else if (dest->width != downsampledWidth || dest->height != downsampleHeight)
    {
        return E_INVALIDARG;
    }

    // Make sure we've received valid data
    if (srcFrameBuffer->Pitch == 0 || downsampledFloatFrameBuffer->Pitch == 0)
    {
        return E_NOINTERFACE;
    }

    HRESULT hr = S_OK;
    float *srcValues = (float *)srcFrameBuffer->pBits;
    float *downsampledDestValues = (float *)downsampledFloatFrameBuffer->pBits;

    const unsigned int srcImageWidth = src->width;

    if (1 == factor)
    {
        errno_t err = memcpy_s(downsampledDestValues, downsampledFloatFrameBuffer->Pitch * dest->height, srcValues, srcFrameBuffer->Pitch * src->height);
        if (0 != err)
        {
            hr = E_FAIL;
        }
    }
    else
    {
        // Adjust for point cloud image size (6 floats per pixel)
        unsigned int step = (src->imageType == NUI_FUSION_IMAGE_TYPE_POINT_CLOUD) ? 6 : 1;
        unsigned int factorStep = factor * step;

        Concurrency::parallel_for(0u, downsampleHeight, [=, &downsampledDestValues, &srcValues](unsigned int y)
        {
            unsigned int index = downsampledWidth * y * step;
            unsigned int srcIndex = srcImageWidth * y * factorStep;

            for (unsigned int x=0; x<downsampledWidth; ++x, srcIndex += factorStep)
            {
                for (unsigned int s=0, localSourceIndex=srcIndex; s<step; ++s, ++index, ++localSourceIndex)
                {
                    downsampledDestValues[index] = srcValues[localSourceIndex];
                }
            }
        });
    }

    return hr;
}

/// <summary>
/// Up sample color or depth float (32bits/pixel) frame with nearest neighbor - replicates pixels
/// </summary>
/// <param name="src">The source color image.</param>
/// <param name="dest">The destination up-sampled color image.</param>
/// <param name="factor">The up sample factor (1=just copy, 2=x*2,y*2, 4=x*4,y*4).</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT UpsampleFrameNearestNeighbor(NUI_FUSION_IMAGE_FRAME *src, NUI_FUSION_IMAGE_FRAME *dest, unsigned int factor)
{
    if (nullptr == src || nullptr == dest)
    {
        return E_INVALIDARG;
    }

    if (src->imageType != dest->imageType || !(src->imageType == NUI_FUSION_IMAGE_TYPE_COLOR || src->imageType == NUI_FUSION_IMAGE_TYPE_FLOAT))
    {
        return E_INVALIDARG;
    }

    if (!IsValidResampleFactor(factor))
    {
        return E_INVALIDARG;
    }

    NUI_FUSION_BUFFER *srcFrameBuffer = src->pFrameBuffer;
    NUI_FUSION_BUFFER *upsampledDestFrameBuffer = dest->pFrameBuffer;

    unsigned int upsampledWidth = src->width * factor;
    unsigned int upsampleHeight = src->height * factor;

    if (1 == factor && srcFrameBuffer->Pitch * src->height != upsampledDestFrameBuffer->Pitch * dest->height)
    {
        return E_INVALIDARG;
    }
    else if (dest->width != upsampledWidth || dest->height != upsampleHeight)
    {
        return E_INVALIDARG;
    }

    // Make sure we've received valid data
    if (srcFrameBuffer->Pitch == 0 || upsampledDestFrameBuffer->Pitch == 0)
    {
        return E_NOINTERFACE;
    }

    HRESULT hr = S_OK;
    unsigned int *srcValues = (unsigned int *)srcFrameBuffer->pBits;
    unsigned int *upsampledDestValues = (unsigned int *)upsampledDestFrameBuffer->pBits;

    const unsigned int srcImageWidth = src->width;
    const unsigned int srcImageHeight = src->height;

    if (1 == factor)
    {
        errno_t err = memcpy_s(upsampledDestValues, upsampledDestFrameBuffer->Pitch * dest->height, srcValues, srcFrameBuffer->Pitch * src->height);
        if (0 != err)
        {
            hr = E_FAIL;
        }
    }
    else
    {
        unsigned int upsampleRowMultiplier = upsampledWidth * factor;

        // Note we run this only for the source image height pixels to sparsely fill the destination with rows
        Concurrency::parallel_for(0u, srcImageHeight, [=, &upsampledDestValues, &srcValues](unsigned int y)
        {
            unsigned int index = upsampleRowMultiplier * y;
            unsigned int srcIndex = srcImageWidth * y;

            // Fill row
            for (unsigned int x=0; x<srcImageWidth; ++x, ++srcIndex)
            {
                unsigned int color = srcValues[srcIndex];

                // Replicate pixels horizontally
                for (unsigned int s=0; s<factor; ++s, ++index)
                {
                    upsampledDestValues[index] = color;
                }
            }
        });

        unsigned int rowByteSize = upsampledWidth * sizeof(unsigned int);

        // Duplicate the remaining rows with memcpy
        for (unsigned int y=0; y<srcImageHeight; ++y)   // iterate only for the smaller number of rows
        {
            unsigned int srcRowIndex = upsampleRowMultiplier * y;

            // Duplicate lines
            for (unsigned int r=1; r<factor; ++r)
            {
                unsigned int index = upsampledWidth * ((y*factor) + r);

                errno_t err = memcpy_s(&(upsampledDestValues[index]), rowByteSize, &(upsampledDestValues[srcRowIndex]), rowByteSize);
                if (0 != err)
                {
                    hr = E_FAIL;
                }
            }
        }
    }

    return hr;
}