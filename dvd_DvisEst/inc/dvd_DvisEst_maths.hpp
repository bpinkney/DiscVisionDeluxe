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

#define CLOSE_TO_ZERO (0.000001)

// Functions
#define MS_TO_S(ms)       ((ms) * 0.001)
#define US_TO_S(us)       ((us) * 0.000001)
#define NS_TO_S(ns)       ((ns) * 0.000000001)
#define S_TO_MS(s)        ((s)  * 1000.0)
#define S_TO_US(s)        ((s)  * 1000000.0)
#define S_TO_NS(s)        ((s)  * 1000000000.0)
#define MS_TO_NS(ms)      ((ms) * 1000000.0)
#define NS_TO_MS(ns)      ((ns) * 0.000001)

#define DEG_TO_RAD(deg)                 ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(rad)                 ((rad) * 180.0 / M_PI)

#define WRAP_2PI(rad)     \
  while ((rad) > M_PI)    \
  {                       \
    (rad) -= 2*M_PI;      \
  }                       \
  while ((rad) < -1*M_PI) \
  {                       \
    (rad) += 2*M_PI;      \
  }

#define BOUND_VARIABLE(VAR,LOW,HIGH) \
  do \
  { \
    if ((VAR) < (LOW)) \
    { \
      (VAR) = (LOW); \
    } \
    else if ((VAR) > (HIGH)) \
    { \
      (VAR) = (HIGH); \
    } \
  } \
  while (0)

#endif // DVD_DVISEST_MATHS_HPP
