#include <assert.h>
#include <stdio.h>

#include "vector.h"


//////////////////////////////////////////////////////////////////////////////
// Create a zero vector
ias_vector ias_vector_zero()
{
  ias_vector v;

  v.x = 0;
  v.y = 0;
  v.z = 0;

  return v;
}


//////////////////////////////////////////////////////////////////////////////
// Create a vector from the given values
ias_vector ias_vector_set(double x, double y, double z)
{
  ias_vector v;

  v.x = x;
  v.y = y;
  v.z = z;

  return v;
}


//////////////////////////////////////////////////////////////////////////////
// Create a vector from a quaternion
ias_vector ias_vector_set_quatern(ias_quatern q)
{
  ias_vector v;

  v.x = q.x;
  v.y = q.y;
  v.z = q.z;

  return v;
}


//////////////////////////////////////////////////////////////////////////////
// Add one vector to another (element by element)
// c = b + a
ias_vector ias_vector_add(ias_vector b, ias_vector a)
{
  return ias_vector_set(b.x + a.x, b.y + a.y, b.z + a.z);
}


//////////////////////////////////////////////////////////////////////////////
// Subtract one vector from another (element by element)
// c = b - a
ias_vector ias_vector_sub(ias_vector b, ias_vector a)
{
  return ias_vector_set(b.x - a.x, b.y - a.y, b.z - a.z);
}


//////////////////////////////////////////////////////////////////////////////
// Mutliply vector by scalar
ias_vector ias_vector_mul(double s, ias_vector a)
{
  ias_vector b;

  b.x = s * a.x;
  b.y = s * a.y;
  b.z = s * a.z;

  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Compute vector magnitude
double ias_vector_mag(ias_vector a)
{
  return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}


//////////////////////////////////////////////////////////////////////////////
// Normalize a vector to unit length
ias_vector ias_vector_unit(ias_vector a)
{
  double d;
  ias_vector b;

  d = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  b.x = a.x / d;
  b.y = a.y / d;
  b.z = a.z / d;

  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Take cross product of two vectors
ias_vector ias_vector_cross(ias_vector a, ias_vector b)
{
  ias_vector c;

  c.x =  a.y * b.z - a.z * b.y;
  c.y = -a.x * b.z + a.z * b.x;
  c.z =  a.x * b.y - a.y * b.x;

  return c;
}


//////////////////////////////////////////////////////////////////////////////
// Take dot product of two vectors
double ias_vector_dot(ias_vector a, ias_vector b)
{
  return a.x * b.x + a.y * b.y + a.x * b.z;
}


//////////////////////////////////////////////////////////////////////////////
// See if a vector is finite (e.g., not nan)
int ias_vector_finite(ias_vector a)
{
  return finite(a.x) && finite(a.y) && finite(a.z);
}


//////////////////////////////////////////////////////////////////////////////
// Create a quaternion from elements
ias_quatern ias_quatern_set(double u, double x, double y, double z)
{
  ias_quatern q;

  q.u = u;
  q.x = x;
  q.y = y;
  q.z = z;

  return q;
}


//////////////////////////////////////////////////////////////////////////////
// Create a quaternion from a vector
ias_quatern ias_quatern_set_vector(ias_vector v)
{
  ias_quatern q;

  q.u = 0.0;
  q.x = v.x;
  q.y = v.y;
  q.z = v.z;

  return q;
}


//////////////////////////////////////////////////////////////////////////////
// Create an identity quaternion
ias_quatern ias_quatern_ident()
{
  ias_quatern q;

  q.u = 1.0;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;

  return q;
}


//////////////////////////////////////////////////////////////////////////////
// Create a quaternion from an axis and angle
ias_quatern ias_quatern_from_axis(double x, double y, double z, double a)
{
  double l;
  ias_quatern q;

  l = x * x + y * y + z * z;

  if (l > 0.0)
  {
    a *= 0.5;
    l = sin(a) / sqrt(l);
    q.u = cos(a);
    q.x = x * l;
    q.y = y * l;
    q.z = z * l;
  }
  else
  {
    q.u = 1;
    q.x = 0;
    q.y = 0;
    q.z = 0;
  }

  q = ias_quatern_normal(q);

  return q;
}


//////////////////////////////////////////////////////////////////////////////
// Convert quaterion to axis and angle
ias_quatern ias_quatern_to_axis(ias_quatern a)
{
  ias_quatern b;

  b.x = a.x;
  b.y = a.y;
  b.z = a.z;
  b.u = acos(a.u) * 2;

  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Create a quaternion from Euler angles
ias_quatern ias_quatern_from_euler(double roll, double pitch, double yaw)
{
  double phi, the, psi;
  ias_quatern p;

  phi = roll / 2;
  the = pitch / 2;
  psi = yaw / 2;

  p.u = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  p.x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  p.y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  p.z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

  p = ias_quatern_normal(p);

  return p;
}

//////////////////////////////////////////////////////////////////////////////
// Create a quaternion from rotation matrix
ias_quatern ias_quatern_from_homo(ias_homo a)
{
  ias_quatern b;

  double trace = a.m[0][0] + a.m[1][1] + a.m[2][2];
  if(trace > 0) {
    double s = 0.5 / sqrt(1.0 + trace);
    b.u = 0.25 / s;
    b.x = (a.m[2][1] - a.m[1][2]) * s;
    b.y = (a.m[0][2] - a.m[2][0]) * s;
    b.z = (a.m[1][0] - a.m[0][1]) * s;
  } else {
    if (a.m[0][0] > a.m[1][1] && a.m[0][0] > a.m[2][2]) {
      double s = 0.5 / sqrt(1.0 + a.m[0][0] - a.m[1][1] - a.m[2][2]);
      b.u = (a.m[2][1] - a.m[1][2]) * s;
      b.x = 0.25 / s;
      b.y = (a.m[0][1] + a.m[1][0]) * s;
      b.z = (a.m[0][2] + a.m[2][0]) * s;
    } else if (a.m[1][1] > a.m[2][2]) {
      double s = 0.5 / sqrt(1.0 + a.m[1][1] - a.m[0][0] - a.m[2][2]);
      b.u = (a.m[0][2] - a.m[2][0]) * s;
      b.x = (a.m[0][1] + a.m[1][0]) * s;
      b.y = 0.25 / s;
      b.z = (a.m[1][2] + a.m[2][1]) * s;
    } else {
      double s = 0.5 / sqrt(1.0 + a.m[2][2] - a.m[0][0] - a.m[1][1]);
      b.u = (a.m[1][0] - a.m[0][1]) * s;
      b.x = (a.m[0][2] + a.m[2][0]) * s;
      b.y = (a.m[1][2] + a.m[2][1]) * s;
      b.z = 0.25 / s;
    }
  }
  b = ias_quatern_normal(b);
  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Convert quaternion to Euler angles
ias_vector ias_quatern_to_euler(ias_quatern q)
{
  double phi, the, psi;

  q = ias_quatern_normal(q);

  phi = atan2(2 * (q.y*q.z + q.u*q.x), (q.u*q.u - q.x*q.x - q.y*q.y + q.z*q.z));
  the = asin(-2 * (q.x*q.z - q.u * q.y));
  psi = atan2(2 * (q.x*q.y + q.u*q.z), (q.u*q.u + q.x*q.x - q.y*q.y - q.z*q.z));

  return ias_vector_set(phi, the, psi);
}


/* REMOVE (not correct)
//////////////////////////////////////////////////////////////////////////////
// Create a Quaternion from a 3x3 rotation matrix
ias_quatern ias_quaternFromRot( double rot[9] )
{
  int which;
  double temp;
  ias_quatern q;

  q.u = (1 + rot[0] + rot[4] + rot[8]) / 4;
  q.x = (1 + rot[0] - rot[4] - rot[8]) / 4;
  q.y = (1 - rot[0] + rot[4] - rot[8]) / 4;
  q.z = (1 - rot[0] - rot[4] + rot[8]) / 4;

  temp = q.u;
  which = 1;
  if (q.x > temp)
  {
    temp = q.x;
    which = 2;
  }
  if (q.y > temp)
  {
    temp = q.y;
    which = 3;
  }
  if (q.z > temp)
  {
    which = 4;
  }

  switch(which)
  {
    case 1:
      q.u = sqrt(q.u);
      temp = 1.0/(4.0*q.u);
      q.x = (rot[7] - rot[5]) * temp;
      q.y = (rot[2] - rot[6]) * temp;
      q.z = (rot[3] - rot[1]) * temp;
      break;
    case 2:
      q.x = sqrt(q.x);
      temp = 1.0/(4.0*q.x);
      q.u = (rot[7] - rot[5]) * temp;
      q.y = (rot[1] + rot[3]) * temp;
      q.z = (rot[2] + rot[6]) * temp;
      break;
    case 3:
      q.y = sqrt(q.y);
      temp = 1.0/(4.0*q.y);
      q.y = (rot[2] - rot[6]) * temp;
      q.x = (rot[1] + rot[3]) * temp;
      q.z = (rot[5] + rot[7]) * temp;
      break;
    case 4:
      q.z = sqrt(q.z);
      temp = 1.0/(4.0*q.z);
      q.u = (rot[3] - rot[1]) * temp;
      q.x = (rot[2] + rot[6]) * temp;
      q.y = (rot[5] + rot[7]) * temp;
      break;
  }

  return q;
}
*/


/* REMOVE
//////////////////////////////////////////////////////////////////////////////
//  Create a quaternion from a pair of points.
ias_quatern ias_quaternFromLookAt(ias_vector center, ias_vector up)
{
  ias_quatern a;
  ias_vector x, y, z;
  double R[9];

  x = ias_vectorUnit(center);
  y = ias_vectorCross(ias_vectorUnit(up), x);
  z = ias_vectorCross(x, y);

  R[0] = x.x;
  R[3] = x.y;
  R[6] = x.z;

  R[1] = y.x;
  R[4] = y.y;
  R[7] = y.z;

  R[2] = z.x;
  R[5] = z.y;
  R[8] = z.z;

  a = ias_quaternFromRot(R);

  return a;
}
*/


//////////////////////////////////////////////////////////////////////////////
// Invert a quaternion
ias_quatern ias_quatern_inverse(ias_quatern a)
{
  ias_quatern b;

  b.u = a.u;
  b.x = -a.x;
  b.y = -a.y;
  b.z = -a.z;

  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Normalize a quaternion
ias_quatern ias_quatern_normal(ias_quatern a)
{
  ias_quatern b;
  double s;

  s = sqrt(a.u * a.u + a.x * a.x + a.y * a.y + a.z * a.z);

  b.u = a.u / s;
  b.x = a.x / s;
  b.y = a.y / s;
  b.z = a.z / s;

  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Multiple two quaternions
ias_quatern ias_quatern_mul(ias_quatern a, ias_quatern b)
{
  ias_quatern c;

  c.u = a.u * b.u - a.x * b.x - a.y * b.y - a.z * b.z;
  c.x = a.u * b.x + a.x * b.u + a.y * b.z - a.z * b.y;
  c.y = a.u * b.y - a.x * b.z + a.y * b.u + a.z * b.x;
  c.z = a.u * b.z + a.x * b.y - a.y * b.x + a.z * b.u;

  return c;
}


//////////////////////////////////////////////////////////////////////////////
// Scale a quaternion rotation
ias_quatern ias_quatern_scale(double s, ias_quatern a)
{
  ias_quatern b;

  // Convert to axis-and-angle
  b = ias_quatern_to_axis(a);

  // Scale angle
  b.u *= s;

  // Convert back again
  return ias_quatern_from_axis(b.x, b.y, b.z, b.u);
}



//////////////////////////////////////////////////////////////////////////////
// See if a quatern is finite (e.g., not nan)
int ias_quatern_finite(ias_quatern a)
{
  return finite(a.x) && finite(a.y) && finite(a.z) && finite(a.u);
}


//////////////////////////////////////////////////////////////////////////////
// Create a pose from the given position and rotation
ias_pose ias_pose_set(ias_vector pos, ias_quatern rot)
{
  ias_pose p;

  p.pos = pos;
  p.rot = rot;

  return p;
}


//////////////////////////////////////////////////////////////////////////////
//  Create a pose from a triplet of vectors
ias_pose ias_pose_point_at(ias_vector pos, ias_vector at, ias_vector up)
{
  ias_pose pose;
  ias_quatern b;
  ias_vector x, y, z;
  double R[3][3];

  x = ias_vector_unit(ias_vector_sub(at, pos));
  y = ias_vector_unit(ias_vector_cross(ias_vector_unit(up), x));
  z = ias_vector_cross(x, y);

  R[0][0] = x.x;
  R[1][0] = x.y;
  R[2][0] = x.z;

  R[0][1] = y.x;
  R[1][1] = y.y;
  R[2][1] = y.z;

  R[0][2] = z.x;
  R[1][2] = z.y;
  R[2][2] = z.z;

  // TODO: switch cases to avoid instability
  b.u = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2;
  if (b.u < 1e-16)
    fprintf(stderr,"pointing singularity (gimbal lock)");
  b.x = (R[2][1] - R[1][2]) / (4 * b.u);
  b.y = (R[0][2] - R[2][0]) / (4 * b.u);
  b.z = (R[1][0] - R[0][1]) / (4 * b.u);
  b = ias_quatern_normal(b);

  /*
  printf("x = %f %f %f  \n", x.x, x.y, x.z);
  printf("y = %f %f %f  \n", y.x, y.y, y.z);
  printf("z = %f %f %f  \n", z.x, z.y, z.z);
  printf("%f  \n", b.u);
  */

  pose.pos = pos;
  pose.rot = b;

  return pose;
}



//////////////////////////////////////////////////////////////////////////////
// See if a pose is finite (e.g., not nan)
int ias_pose_finite(ias_pose a)
{
  return ias_vector_finite(a.pos) && ias_quatern_finite(a.rot);
}


//////////////////////////////////////////////////////////////////////////////
// Add one pose to another: c = b + a
ias_pose ias_coord_pose_add(ias_pose bpose, ias_pose apose)
{
  ias_pose cpose;

  cpose.pos = ias_coord_position_add(bpose.pos, apose.pos, apose.rot);
  cpose.rot = ias_coord_rotation_add(bpose.rot, apose.rot);

  return cpose;
}


//////////////////////////////////////////////////////////////////////////////
// Subtract one pose from another: c = b - a
ias_pose ias_coord_pose_sub(ias_pose bpose, ias_pose apose)
{
  ias_pose cpose;

  cpose.pos = ias_coord_position_sub(bpose.pos, apose.pos, apose.rot);
  cpose.rot = ias_coord_rotation_sub(bpose.rot, apose.rot);

  return cpose;
}


//////////////////////////////////////////////////////////////////////////////
// Find the inverse of a pose; i.e., if b = ba + a, given b and ba,
// find a
ias_pose ias_coord_pose_solve(ias_pose ba, ias_pose b)
{
  ias_quatern q;
  ias_pose a;

  a.rot = ias_quatern_mul(ias_quatern_inverse(ba.rot), b.rot);
  q = ias_quatern_mul(a.rot, ias_quatern_set_vector(ba.pos));
  q = ias_quatern_mul(q, ias_quatern_inverse(a.rot));
  a.pos = ias_vector_sub(b.pos, ias_vector_set_quatern(q));

  return a;
}


//////////////////////////////////////////////////////////////////////////////
// Add one position to another: c = b + a
ias_vector ias_coord_position_add(ias_vector bpos, ias_vector apos, ias_quatern arot)
{
  ias_vector cpos;

  // cpos = apos + arot * bpos * arot!
  cpos = ias_vector_set_quatern(ias_quatern_mul(arot, ias_quatern_mul(ias_quatern_set_vector(bpos), ias_quatern_inverse(arot))));
  cpos = ias_vector_add(apos, cpos);

  return cpos;
}


//////////////////////////////////////////////////////////////////////////////
// Subtract one position from another: c = b - a
ias_vector ias_coord_position_sub(ias_vector bpos, ias_vector apos, ias_quatern arot)
{
  ias_vector cpos;

  // cpos = arot! * (bpos - apos) * arot
  cpos = ias_vector_sub(bpos, apos);
  cpos = ias_vector_set_quatern(ias_quatern_mul(ias_quatern_inverse(arot), ias_quatern_mul(ias_quatern_set_vector(cpos), arot)));

  return cpos;
}


//////////////////////////////////////////////////////////////////////////////
// Add one rotation to another: b = r + a
ias_quatern ias_coord_rotation_add(ias_quatern r, ias_quatern a)
{
  ias_quatern b;

  // b = a * r
  b = ias_quatern_mul(a, r);

  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Subtract one rotation from another: r = b - a
ias_quatern ias_coord_rotation_sub(ias_quatern b, ias_quatern a)
{
  ias_quatern r;

  // r = a! * b
  r = ias_quatern_mul(ias_quatern_inverse(a), b);

  return r;
}

//////////////////////////////////////////////////////////////////////////////
// Compute a homogeneous transform matrix from a Quatern
ias_homo ias_homo_from_quatern(ias_quatern q)
{
  ias_homo b;

  b.m[0][0] = 1 - (2*(q.y*q.y) + 2*(q.z*q.z));
  b.m[0][1] = 2 * q.x * q.y - 2 * q.z * q.u;
  b.m[0][2] = 2 * q.x * q.z + 2 * q.y * q.u;
  
  b.m[1][0] = 2 * q.x * q.y + 2 * q.z * q.u;
  b.m[1][1] = 1 - (2*(q.x*q.x) + 2*(q.z*q.z));
  b.m[1][2] = 2 * q.y * q.z - 2 * q.x * q.u;

  b.m[2][0] = 2 * q.x * q.z - 2 * q.y * q.u;
  b.m[2][1] = 2 * q.y * q.z + 2 * q.x * q.u;
  b.m[2][2] = 1 - (2*(q.x*q.x) + 2*(q.y*q.y));

  return b;
}


//////////////////////////////////////////////////////////////////////////////
// Compute the inverse of the given transform matrix
ias_homo ias_homo_inverse(ias_homo a)
{
  double det;
  ias_homo b;

  det = ( + a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) - a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) + a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]));
  b.m[0][0] = ( + (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1])) / det;
  b.m[0][1] = ( - (a.m[0][1] * a.m[2][2] - a.m[0][2] * a.m[2][1])) / det;
  b.m[0][2] = ( + (a.m[0][1] * a.m[1][2] - a.m[0][2] * a.m[1][1])) / det;
  b.m[1][0] = ( - (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0])) / det;
  b.m[1][1] = ( + (a.m[0][0] * a.m[2][2] - a.m[0][2] * a.m[2][0])) / det;
  b.m[1][2] = ( - (a.m[0][0] * a.m[1][2] - a.m[0][2] * a.m[1][0])) / det;
  b.m[2][0] = ( + (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0])) / det;
  b.m[2][1] = ( - (a.m[0][0] * a.m[2][1] - a.m[0][1] * a.m[2][0])) / det;
  b.m[2][2] = ( + (a.m[0][0] * a.m[1][1] - a.m[0][1] * a.m[1][0])) / det;

  return b;
}


//////////////////////////////////////////////////////////////////////////////
/// Apply a homogeneous transform
ias_vector ias_homo_mul(ias_homo m, ias_vector a)
{
  ias_vector b;

  b.x = m.m[0][0] * a.x + m.m[0][1] * a.y + m.m[0][2] * a.z;
  b.y = m.m[1][0] * a.x + m.m[1][1] * a.y + m.m[1][2] * a.z;
  b.z = m.m[2][0] * a.x + m.m[2][1] * a.y + m.m[2][2] * a.z;

  return b;
}

