/**
* Some common definitions.
*
* Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
* Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM.
*/

#ifndef _DEFINE_H
#define _DEFINE_H

#include "Vector.h"
#include "Matrix.h"
#include "Image.h"
#include "ImagesBlock.h"
#include "VisibleListBlock.h"

#ifndef PI
#define PI float(3.1415926535897932384626433832795)
#endif

#ifndef NULL
#define NULL 0
#endif

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

//==========================================

typedef class Basis::Matrix3<float> Matrix3f;
typedef class Basis::Matrix4<float> Matrix4f;

typedef class Basis::Vector2<short> Vector2s;
typedef class Basis::Vector2<int> Vector2i;
typedef class Basis::Vector2<float> Vector2f;
typedef class Basis::Vector2<double> Vector2d;

typedef class Basis::Vector3<short> Vector3s;
typedef class Basis::Vector3<double> Vector3d;
typedef class Basis::Vector3<int> Vector3i;
typedef class Basis::Vector3<uint> Vector3ui;
typedef class Basis::Vector3<uchar> Vector3u;
typedef class Basis::Vector3<float> Vector3f;

typedef class Basis::Vector4<float> Vector4f;
typedef class Basis::Vector4<int> Vector4i;
typedef class Basis::Vector4<short> Vector4s;
typedef class Basis::Vector4<uchar> Vector4u;
typedef class Basis::Vector4<double> Vector4d;

typedef class Basis::Vector6<float> Vector6f;

#ifndef TO_INT_ROUND3
#define TO_INT_ROUND3(x) (x).toIntRound()
#endif

#ifndef TO_INT_ROUND4
#define TO_INT_ROUND4(x) (x).toIntRound()
#endif

#ifndef TO_INT_FLOOR3
#define TO_INT_FLOOR3(inted, coeffs, in) inted = (in).toIntFloor(coeffs)
#endif

#ifndef TO_SHORT_FLOOR3
#define TO_SHORT_FLOOR3(x) (x).toShortFloor()
#endif

#ifndef TO_UCHAR3
#define TO_UCHAR3(x) (x).toUChar()
#endif

#ifndef TO_FLOAT3
#define TO_FLOAT3(x) (x).toFloat()
#endif

#ifndef TO_VECTOR3
#define TO_VECTOR3(a) (a).toVector3()
#endif

#ifndef IS_EQUAL3
#define IS_EQUAL3(a,b) (((a).x == (b).x) && ((a).y == (b).y) && ((a).z == (b).z))
#endif

//==========================================
#ifndef FloatImage
#define FloatImage Basis::Image<float>
#endif

#ifndef Float2Image
#define Float2Image Basis::Image<Vector2f>
#endif

#ifndef Float4Image
#define Float4Image Basis::Image<Vector4f>
#endif

#ifndef ShortImage
#define ShortImage Basis::Image<short>
#endif

#ifndef Short3Image
#define Short3Image Basis::Image<Vector3s>
#endif

#ifndef Short4Image
#define Short4Image Basis::Image<Vector4s>
#endif

#ifndef UShortImage
#define UShortImage Basis::Image<ushort>
#endif

#ifndef UIntImage
#define UIntImage Basis::Image<uint>
#endif

#ifndef IntImage
#define IntImage Basis::Image<int>
#endif

#ifndef UCharImage
#define UCharImage Basis::Image<uchar>
#endif

#ifndef UChar4Image
#define UChar4Image Basis::Image<Vector4u>
#endif

#ifndef BoolImage
#define BoolImage Basis::Image<bool>
#endif

#ifndef UChar4ImagesBlock
#define UChar4ImagesBlock Basis::ImagesBlock<Vector4u>
#endif

#ifndef ShortImagesBlock
#define ShortImagesBlock Basis::ImagesBlock<short>
#endif

#ifndef BitVisibleListBlock
#define BitVisibleListBlock Basis::VisibleListBlock<uchar>
#endif

//======================================================
#ifndef	USE_IMAGES_BLOCK
#define USE_IMAGES_BLOCK
#endif

#ifndef	USE_VISIBLELIST_BLOCK
#define USE_VISIBLELIST_BLOCK
#endif

#ifndef	IMAGES_BLOCK_SIZE 
#define IMAGES_BLOCK_SIZE 3000
#endif

#endif