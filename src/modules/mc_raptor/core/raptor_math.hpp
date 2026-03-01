#pragma once

namespace mc_raptor_core
{
namespace math
{

template <typename T>
inline T clip(T x, T max, T min)
{
	if (x > max) {
		return max;
	}

	if (x < min) {
		return min;
	}

	return x;
}

template <typename T>
inline void quaternion_multiplication(const T q1[4], const T q2[4], T q_res[4])
{
	q_res[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	q_res[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	q_res[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	q_res[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

template <typename T>
inline void quaternion_conjugate(const T q[4], T q_res[4])
{
	q_res[0] = +q[0];
	q_res[1] = -q[1];
	q_res[2] = -q[2];
	q_res[3] = -q[3];
}

template <typename T>
inline void quaternion_to_rotation_matrix(const T q[4], T R[9])
{
	const T qw = q[0];
	const T qx = q[1];
	const T qy = q[2];
	const T qz = q[3];

	R[0] = 1 - 2 * qy * qy - 2 * qz * qz;
	R[1] = 2 * qx * qy - 2 * qw * qz;
	R[2] = 2 * qx * qz + 2 * qw * qy;
	R[3] = 2 * qx * qy + 2 * qw * qz;
	R[4] = 1 - 2 * qx * qx - 2 * qz * qz;
	R[5] = 2 * qy * qz - 2 * qw * qx;
	R[6] = 2 * qx * qz - 2 * qw * qy;
	R[7] = 2 * qy * qz + 2 * qw * qx;
	R[8] = 1 - 2 * qx * qx - 2 * qy * qy;
}

template <typename T>
inline void rotate_vector(const T R[9], const T v[3], T v_rotated[3])
{
	v_rotated[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
	v_rotated[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
	v_rotated[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

} // namespace math
} // namespace mc_raptor_core
