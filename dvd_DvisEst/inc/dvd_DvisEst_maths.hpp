#ifndef DVD_DVISEST_MATHS_HPP
#define DVD_DVISEST_MATHS_HPP

// This header just contains useful math macros and functions, mostly used by dvd_DvisEst_estimate

// Indexing functions for matrices
#define MAT2X2(x) x[4]
#define MAT3X3(x) x[9]
#define MAT4X4(x) x[16]

#define VEC2(x)   x[2]
#define VEC3(x)   x[3]
#define VEC4(x)   x[4]

// Row-major
#define i2x2(r,c)   (2*(r)+(c))
#define i3x3(r,c)   (3*(r)+(c))
#define i4x4(r,c)   (4*(r)+(c))

#define CLOSE_TO_ZERO (0.0001)

#endif // DVD_DVISEST_MATHS_HPP
