/*
 Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
 Author: Ingo Kresse <kresse at in.tum.de>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef VECTOR_H_
#define VECTOR_H_

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// Basic 3 vector
typedef struct
{
  double x, y, z;
} ias_vector;



// Quatnernion
typedef struct
{
  double u, x, y, z;
} ias_quatern;


// Combined pose
typedef struct
{
  ias_vector pos;
  ias_quatern rot;
} ias_pose;

/// @brief Matrix for homogeneous transforms on 2D points
typedef struct
{
  double m[3][3];
} ias_homo;

// Create a zero vector
ias_vector ias_vector_zero();

// Create a vector from the given values
ias_vector ias_vector_set(double x, double y, double z);

// Create a vector from a quaternion
ias_vector ias_vector_set_quatern(ias_quatern q);

// Add one vector to another (element by element)
ias_vector ias_vector_add(ias_vector b, ias_vector a);

// Subtract one vector from another (element by element)
ias_vector ias_vector_sub(ias_vector b, ias_vector a);

// Mutliply vector by scalar
ias_vector ias_vector_mul(double s, ias_vector a);

// Compute vector magnitude
double ias_vector_mag(ias_vector a);

// Normalize a vector to unit length
ias_vector ias_vector_unit(ias_vector a);

// Take cross product of two vectors
ias_vector ias_vector_cross(ias_vector a, ias_vector b);

// Take dot product of two vectors
double ias_vector_dot(ias_vector a, ias_vector b);

// See if a vector is finite (e.g., not nan)
int ias_vector_finite(ias_vector a);


// Create an identity quaternion
ias_quatern ias_quatern_ident();

// Create a quaternion from elements
ias_quatern ias_quatern_set(double u, double x, double y, double z);

// Create a quaternion form a vector
ias_quatern ias_quatern_set_vector(ias_vector v);

// Invert a quaternion
ias_quatern ias_quatern_inverse(ias_quatern a);

// Normalize a quaternion
ias_quatern ias_quatern_normal(ias_quatern a);

// Create a quaternion from an axis and angle
ias_quatern ias_quatern_from_axis(double x, double y, double z, double a);

// Create a quaternion from Euler angles
ias_quatern ias_quatern_from_euler(double roll, double pitch, double yaw);

// Create a quatern from Homogeneous matrix
ias_quatern ias_quatern_from_homo(ias_homo homo);

// Convert quaternion to Euler angles (roll, pitch, yaw)
ias_vector ias_quatern_to_euler(ias_quatern q);

// Convert quaterion to axis and angle (x, y, z, rotation)
ias_quatern ias_quatern_to_axis(ias_quatern a);

// Multiply two quaternions
ias_quatern ias_quatern_mul(ias_quatern a, ias_quatern b);

// Scale a quaternion rotation
ias_quatern ias_quatern_scale(double s, ias_quatern a);

// See if a quaternion is finite (e.g., not nan)
int ias_quatern_finite(ias_quatern a);


// Create a pose from the given position and rotation
ias_pose ias_pose_set(ias_vector pos, ias_quatern rot);

/// @brief Create a pose from a triplet of vectors.
/// @param pos specifies the object position.
/// @param at specifies the point to look at (x axis).
/// @param up Specifies the "up" direction (z axis).
ias_pose ias_pose_point_at(ias_vector pos, ias_vector at, ias_vector up);

// See if a pose is finite (e.g., not nan)
int ias_pose_finite(ias_pose a);




// Add one pose to another: c = b + a
ias_pose ias_coord_pose_add(ias_pose bpose, ias_pose apose);

// Subtract one pose from another: c = b - a
ias_pose ias_coord_pose_sub(ias_pose bpose, ias_pose apose);

// Find the inverse of a pose; i.e., if b = ba + a, given b and ba,
// find a
ias_pose ias_coord_pose_solve(ias_pose ba, ias_pose b);

// Add one position to another: c = b + a
ias_vector ias_coord_position_add(ias_vector bpos, ias_vector apos, ias_quatern arot);

// Subtract one position from another: c = b - a
ias_vector ias_coord_position_sub(ias_vector bpos, ias_vector apos, ias_quatern arot);

// Add one rotation to another: c = b + a
ias_quatern ias_coord_rotation_add(ias_quatern b, ias_quatern a);

// Subtract one rotation from another: c = b - a
ias_quatern ias_coord_rotation_sub(ias_quatern b, ias_quatern a);


/// @brief Compute a homogeneous transform matrix from a Quatern
ias_homo ias_homo_from_quatern(ias_quatern q);

/// @brief Compute the inverse of the given transform matrix
ias_homo ias_homo_inverse(ias_homo a);

/// @brief Apply a homogeneous transform
ias_vector ias_homo_mul(ias_homo m, ias_vector a);

#ifdef __cplusplus
}
#endif

#endif
