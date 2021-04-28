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

#ifndef MIN
#define MIN(A,B)      (((A) < (B)) ? (A) : (B))
#endif
#ifndef MAX
#define MAX(A,B)      (((A) < (B)) ? (B) : (A))
#endif

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

// [-pi, pi]
#define WRAP_2PI(rad)     \
  while ((rad) > M_PI)    \
  {                       \
    (rad) -= 2*M_PI;      \
  }                       \
  while ((rad) < -1*M_PI) \
  {                       \
    (rad) += 2*M_PI;      \
  }

// [0, 2*pi]
#define WRAP_TO_2PI(rad)  \
  while ((rad) > 2*M_PI)  \
  {                       \
    rad -= 2*M_PI;        \
  }                       \
  while ((rad) < 0)       \
  {                       \
    rad += 2*M_PI;        \
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

// only works for +-90deg roll or pitch though...
// eul returned in rads
static inline void Rxyz2eulxyz(float MAT3X3(Rxyz), float VEC3(eul_xyz))
{
  float X;
  float Y;
  float Z;

  if(Rxyz[i3x3(0,2)] < 1)
  {
    if(Rxyz[i3x3(0,2)] > -1)
    {
      Y = asin(  Rxyz[i3x3(0,2)]);
      X = atan2(-Rxyz[i3x3(1,2)], Rxyz[i3x3(2,2)]);
      Z = atan2(-Rxyz[i3x3(0,1)], Rxyz[i3x3(0,0)]);
    }
    else//Rxyz[i3x3(0,2)]=-1
    {
      //Notauniquesolution:Z-X=atan2(Rxyz[i3x3(1,0)],Rxyz[i3x3(1,1)])
      Y = -M_PI / 2.0;
      X = -atan2(Rxyz[i3x3(1,0)], Rxyz[i3x3(1,1)]);
      Z = 0;
    }
  }
  else //Rxyz[i3x3(0,2)]=+1
  {
    // Notauniquesolution:Z+X=atan2(Rxyz[i3x3(1,0)],Rxyz[i3x3(1,1)])
    Y = M_PI / 2.0;
    X = atan2(Rxyz[i3x3(1,0)], Rxyz[i3x3(1,1)]);
    Z = 0;
  }

  eul_xyz[0] = X;
  eul_xyz[1] = Y;
  eul_xyz[2] = Z;
}

// only works for +-90deg roll or pitch though...
// eul returned in rads
static inline void Ryxz2eulyxz(float MAT3X3(Ryxz), float VEC3(eul_yxz))
{
  float X;
  float Y;
  float Z;

  if(Ryxz[i3x3(1,2)] < 1)
  {
    if(Ryxz[i3x3(1,2)] > -1)
    {
      X = asin(-Ryxz[i3x3(1,2)]);
      Y = atan2(Ryxz[i3x3(0,2)], Ryxz[i3x3(2,2)]);
      Z = atan2(Ryxz[i3x3(1,0)], Ryxz[i3x3(1,1)]);
    }
    else//r12=−1
    {
      //Notauniquesolution:thetaZ−thetaY=atan2(−r01,r00)
      X = M_PI/2;
      Y = -atan2(-Ryxz[i3x3(0,1)], Ryxz[i3x3(0,0)]);
      Z = 0;
    }
  }
  else//r12=+1
  {
    //Notauniquesolution:thetaZ+thetaY=atan2(−r01,r00)
    X = -M_PI/2;
    Y = atan2(-Ryxz[i3x3(0,1)], Ryxz[i3x3(0,0)]);
    Z = 0;
  }

  eul_yxz[0] = Y;
  eul_yxz[1] = X;
  eul_yxz[2] = Z;
}

// only works for +-90deg roll or pitch though...
// eul returned in rads
static inline void Ryzx2eulyzx(float MAT3X3(Ryzx), float VEC3(eul_yzx))
{
  float X;
  float Y;
  float Z;

  if(Ryzx[i3x3(1,0)] < 1)
  {
    if(Ryzx[i3x3(1,0)] > -1)
    {
      Z = asin(Ryzx[i3x3(1,0)]);
      Y = atan2(-Ryzx[i3x3(2,0)],Ryzx[i3x3(0,0)]);
      X = atan2(-Ryzx[i3x3(1,2)],Ryzx[i3x3(1,1)]);
    }
    else//r10=-1
    {
      //Notauniquesolution:X-Y=atan2(r21,r22)
      Z = -M_PI/2;
      Y = -atan2(Ryzx[i3x3(2,1)],Ryzx[i3x3(2,2)]);
      X = 0;
    }
  }
  else
  {
    //Notauniquesolution:XY=atan2(r21,r22)
    Z = M_PI/2;
    Y = atan2(Ryzx[i3x3(2,1)],Ryzx[i3x3(2,2)]);
    X = 0;
  }

  eul_yzx[0] = Y;
  eul_yzx[1] = Z;
  eul_yzx[2] = X;
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

// Low-pass first-order IIR filter
// Time constant is ~ (N * T_sample), N=0 for no filter
#define LP_FILT(var, new_val, N)  \
  do { var = ((float)var * ((float)N) + ((float)new_val)) / ((float)N+1); } while(0)

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

/**
 * Calculate a rotation matrix from 2 normalized vectors.
 *
 * v1 and v2 must be unit length.
 */
/* -------------------------------------------------------------------- */
/* Math Lib declarations */

static void unit_m3(float m[3][3]);
static float dot_v3v3(const float a[3], const float b[3]);
static float normalize_v3(float n[3]);
static void cross_v3_v3v3(float r[3], const float a[3], const float b[3]);
static void mul_v3_v3fl(float r[3], const float a[3], float f);
static void ortho_v3_v3(float p[3], const float v[3]);
static void axis_angle_normalized_to_mat3_ex(
        float mat[3][3], const float axis[3],
        const float angle_sin, const float angle_cos);


/* -------------------------------------------------------------------- */
/* Main function */
static void rotation_between_vecs_to_mat3(float m[3][3], const float v1[3], const float v2[3]);
    
static void rotation_between_vecs_to_mat3(float m[3][3], const float v1[3], const float v2[3])
{
    float axis[3];
    /* avoid calculating the angle */
    float angle_sin;
    float angle_cos;

    cross_v3_v3v3(axis, v1, v2);

    angle_sin = normalize_v3(axis);
    angle_cos = dot_v3v3(v1, v2);

    if (angle_sin > FLT_EPSILON) {
axis_calc:
        axis_angle_normalized_to_mat3_ex(m, axis, angle_sin, angle_cos);
    }
    else {
        /* Degenerate (co-linear) vectors */
        if (angle_cos > 0.0f) {
            /* Same vectors, zero rotation... */
            unit_m3(m);
        }
        else {
            /* Colinear but opposed vectors, 180 rotation... */
            ortho_v3_v3(axis, v1);
            normalize_v3(axis);
            angle_sin =  0.0f;  /* sin(M_PI) */
            angle_cos = -1.0f;  /* cos(M_PI) */
            goto axis_calc;
        }
    }
}


/* -------------------------------------------------------------------- */
/* Math Lib */

static void unit_m3(float m[3][3])
{
    m[0][0] = m[1][1] = m[2][2] = 1.0;
    m[0][1] = m[0][2] = 0.0;
    m[1][0] = m[1][2] = 0.0;
    m[2][0] = m[2][1] = 0.0;
}

static float dot_v3v3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross_v3_v3v3(float r[3], const float a[3], const float b[3])
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

static void mul_v3_v3fl(float r[3], const float a[3], float f)
{
    r[0] = a[0] * f;
    r[1] = a[1] * f;
    r[2] = a[2] * f;
}

static float normalize_v3_v3(float r[3], const float a[3])
{
    float d = dot_v3v3(a, a);

    if (d > 1.0e-35f) {
        d = sqrtf(d);
        mul_v3_v3fl(r, a, 1.0f / d);
    }
    else {
        d = r[0] = r[1] = r[2] = 0.0f;
    }

    return d;
}

static float normalize_v3(float n[3])
{
    return normalize_v3_v3(n, n);
}

static int axis_dominant_v3_single(const float vec[3])
{
    const float x = fabsf(vec[0]);
    const float y = fabsf(vec[1]);
    const float z = fabsf(vec[2]);
    return ((x > y) ?
           ((x > z) ? 0 : 2) :
           ((y > z) ? 1 : 2));
}

static void ortho_v3_v3(float p[3], const float v[3])
{
    const int axis = axis_dominant_v3_single(v);

    switch (axis) {
        case 0:
            p[0] = -v[1] - v[2];
            p[1] =  v[0];
            p[2] =  v[0];
            break;
        case 1:
            p[0] =  v[1];
            p[1] = -v[0] - v[2];
            p[2] =  v[1];
            break;
        case 2:
            p[0] =  v[2];
            p[1] =  v[2];
            p[2] = -v[0] - v[1];
            break;
    }
}

/* axis must be unit length */
static void axis_angle_normalized_to_mat3_ex(
        float mat[3][3], const float axis[3],
        const float angle_sin, const float angle_cos)
{
    float nsi[3], ico;
    float n_00, n_01, n_11, n_02, n_12, n_22;

    ico = (1.0f - angle_cos);
    nsi[0] = axis[0] * angle_sin;
    nsi[1] = axis[1] * angle_sin;
    nsi[2] = axis[2] * angle_sin;

    n_00 = (axis[0] * axis[0]) * ico;
    n_01 = (axis[0] * axis[1]) * ico;
    n_11 = (axis[1] * axis[1]) * ico;
    n_02 = (axis[0] * axis[2]) * ico;
    n_12 = (axis[1] * axis[2]) * ico;
    n_22 = (axis[2] * axis[2]) * ico;

    mat[0][0] = n_00 + angle_cos;
    mat[0][1] = n_01 + nsi[2];
    mat[0][2] = n_02 - nsi[1];
    mat[1][0] = n_01 - nsi[2];
    mat[1][1] = n_11 + angle_cos;
    mat[1][2] = n_12 + nsi[0];
    mat[2][0] = n_02 + nsi[1];
    mat[2][1] = n_12 - nsi[0];
    mat[2][2] = n_22 + angle_cos;
}

#endif // DVD_MATHS_HPP
