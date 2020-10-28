#ifndef DVD_MATHS_HPP
#define DVD_MATHS_HPP

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
#define NS_TO_US(ns)      ((ns) * 0.001)

#define MM_TO_M(mm)       ((mm) * 0.001)
#define M_TO_MM(m)        ((m)  * 1000.0)

#ifndef M_PI
#define M_PI (3.14159265359)
#endif

#ifndef M_PI_2
#define M_PI_2 (3.14159265359*0.5)
#endif

#define GRAV    (9.80665)

//air density
#define ISA_RHO       1.225

#define DT_1Hz    (1.0)
#define DT_10Hz   (1.0/10.0)
#define DT_50Hz   (1.0/50.0)
#define DT_100Hz  (1.0/100.0)
#define DT_200Hz  (1.0/200.0)

#define DEG_TO_RAD(deg)                 ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(rad)                 ((rad) * 180.0 / M_PI)
#define MRAD_TO_RAD(mrad)               ((mrad) * 0.001)

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

static inline int signum(float x) {
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

static inline void R2Q( float MAT3X3(R), float VEC4(Q) )
{
  // Note the reverse ordering
  const float m00 = R[i3x3(0,0)];
  const float m10 = R[i3x3(0,1)];
  const float m20 = R[i3x3(0,2)];
  const float m01 = R[i3x3(1,0)];
  const float m11 = R[i3x3(1,1)];
  const float m21 = R[i3x3(1,2)];
  const float m02 = R[i3x3(2,0)];
  const float m12 = R[i3x3(2,1)];
  const float m22 = R[i3x3(2,2)];
  
  float trace1 = 1.0 + m00 - m11 - m22;
  float trace2 = 1.0 - m00 + m11 - m22;
  float trace3 = 1.0 - m00 - m11 + m22;

  if( (trace1 > trace2) && (trace1 > trace3) )
  {
    if( trace1 < 0.0 ) trace1 = 0.0;
    
    const float s = 0.5 / sqrt(trace1);
    Q[0] = (m12 - m21) * s;
    Q[1] = 0.25 / s;
    Q[2] = (m01 + m10) * s; 
    Q[3] = (m02 + m20) * s; 
  }
  else if( (trace2 > trace1) && (trace2 > trace3) )
  {
    if( trace2 < 0.0 ) trace2 = 0.0;
    
    const float s = 0.5 / sqrt(trace2);
    Q[0] = (m20 - m02) * s;
    Q[1] = (m01 + m10) * s; 
    Q[2] = 0.25 / s;
    Q[3] = (m12 + m21) * s; 
  }
  else
  {
    if( trace3 < 0.0 ) trace3 = 0.0;
  
    const float s = 0.5 / sqrt(trace3);
    Q[0] = (m01 - m10) * s;
    Q[1] = (m02 + m20) * s;
    Q[2] = (m12 + m21) * s;
    Q[3] = 0.25 / s;
  }
}

static inline void norm_quat(float quat[4]) 
{
  uint8_t i;
  float mag = 0;
  for(i=0; i<4; i++)
    mag += quat[i] * quat[i];
  mag = sqrt( mag );
  for(i=0; i<4; i++)
    quat[i] /= mag;
}

static inline void Q2R( float VEC4(Q), float MAT3X3(R) )
{
  const float q0 = Q[0] * 1.414213562373095;
  const float q1 = Q[1] * 1.414213562373095;
  const float q2 = Q[2] * 1.414213562373095;
  const float q3 = Q[3] * 1.414213562373095;
  
  const float q0q1 = q0*q1;
  const float q0q2 = q0*q2;
  const float q0q3 = q0*q3;
  const float q1q1 = q1*q1;
  const float q1q2 = q1*q2;
  const float q1q3 = q1*q3;
  const float q2q2 = q2*q2;
  const float q2q3 = q2*q3;
  const float q3q3 = q3*q3;
  
  // Row-major storage
  R[i3x3(0,0)] = 1.0-(q2q2 + q3q3);
  R[i3x3(0,1)] =     (q1q2 - q0q3);
  R[i3x3(0,2)] =     (q1q3 + q0q2);
  
  R[i3x3(1,0)] =     (q1q2 + q0q3);
  R[i3x3(1,1)] = 1.0-(q1q1 + q3q3);
  R[i3x3(1,2)] =     (q2q3 - q0q1);
  
  R[i3x3(2,0)] =     (q1q3 - q0q2);
  R[i3x3(2,1)] =     (q2q3 + q0q1);
  R[i3x3(2,2)] = 1.0-(q1q1 + q2q2);
}

#endif // DVD_MATHS_HPP
