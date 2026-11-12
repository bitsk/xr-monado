// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <xr/head/production-v2/a081_qvr_pose.h>
#include "math/m_imu_3dof.h"

// Presentation state only. Raw OEM packets and their acceptance gates are unchanged.
// All durations use sensor time, never packet counts or the render frame rate.
#define A081_INERTIAL_NS UINT64_C(150000000)
#define A081_RECOVER_STABLE_NS UINT64_C(150000000)
#define A081_RECOVER_BLEND_NS UINT64_C(200000000)
#define A081_IMU_GAP_NS UINT64_C(50000000)

enum a081_head_mode {
	A081_HEAD_LOST,
	A081_HEAD_TRACKED,
	A081_HEAD_INERTIAL,
	A081_HEAD_ROTATION_ONLY,
	A081_HEAD_RECOVERING,
};

struct a081_head_recovery {
	enum a081_head_mode mode;
	uint64_t last_imu_ns, last_tracked_ns, stable_since_ns, blend_since_ns;
	bool anchored, translation_stopped;
	struct xrt_space_relation output;
	struct xrt_vec3 last_position, velocity, accel_bias, position_error;
	struct xrt_quat fusion_to_world, orientation_error;
};

static inline const char *
a081_head_mode_name(enum a081_head_mode mode)
{
	static const char *names[] = {"lost", "tracked", "inertial", "rotation_only", "recovering"};
	return names[mode];
}

static inline void
a081_head_recovery_reset(struct a081_head_recovery *s, struct m_imu_3dof *fusion)
{
	memset(s, 0, sizeof(*s));
	s->output.pose.orientation.w = 1;
	m_imu_3dof_reset(fusion);
}

static inline float
a081_head_length(struct xrt_vec3 v)
{
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline void
a081_head_anchor(struct a081_head_recovery *s, const struct m_imu_3dof *fusion)
{
	struct xrt_quat inverse;
	math_quat_invert(&fusion->rot, &inverse);
	math_quat_rotate(&s->output.pose.orientation, &inverse, &s->fusion_to_world);
}

static inline void
a081_head_infer(struct a081_head_recovery *s, const struct m_imu_3dof *fusion,
	              struct xrt_vec3 accel, struct xrt_vec3 gyro, uint64_t time, uint64_t previous)
{
	math_quat_rotate(&s->fusion_to_world, &fusion->rot, &s->output.pose.orientation);
	math_quat_rotate_vec3(&s->output.pose.orientation, &gyro, &s->output.angular_velocity);
	s->output.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                           XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                           XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT |
	                           XRT_SPACE_RELATION_POSITION_VALID_BIT;
	// Specific force -> tracker world -> remove gravity and stationary bias.
	struct xrt_vec3 world;
	math_quat_rotate_vec3(&s->output.pose.orientation, &accel, &world);
	world.x -= s->accel_bias.x;
	world.y -= 9.80665f + s->accel_bias.y;
	world.z -= s->accel_bias.z;
	const uint64_t end = s->last_tracked_ns + A081_INERTIAL_NS;
	const uint64_t until = time < end ? time : end;
	if (!s->translation_stopped && until > previous) {
		float dt = (float)(until - previous) / 1e9f;
		// Saturation/impact is not a usable acceleration measurement.
		if (a081_head_length(world) > 15.f) s->translation_stopped = true;
		else {
			struct xrt_vec3 next = {
			    s->output.pose.position.x + s->velocity.x * dt + .5f * world.x * dt * dt,
			    s->output.pose.position.y + s->velocity.y * dt + .5f * world.y * dt * dt,
			    s->output.pose.position.z + s->velocity.z * dt + .5f * world.z * dt * dt};
			struct xrt_vec3 displacement = {next.x - s->last_position.x,
			                                 next.y - s->last_position.y,
			                                 next.z - s->last_position.z};
			if (a081_head_length(displacement) > .25f) s->translation_stopped = true;
			else {
				s->output.pose.position = next;
				s->velocity.x += world.x * dt;
				s->velocity.y += world.y * dt;
				s->velocity.z += world.z * dt;
			}
		}
	}
	if (time >= end) s->translation_stopped = true;
	s->mode = s->translation_stopped ? A081_HEAD_ROTATION_ONLY : A081_HEAD_INERTIAL;
	// No linear velocity flag: the compositor must not extend this bounded
	// positional inference with a second prediction past the cutoff.
	s->output.linear_velocity = (struct xrt_vec3){0};
}

static inline bool
a081_head_recovery_update(struct a081_head_recovery *s, struct m_imu_3dof *fusion,
	                        const struct a081_tracking_packet *p, uint64_t now)
{
	struct xrt_space_relation measured;
	bool tracked = a081_qvr_pose(p, now, &measured);
	uint64_t time = p->gyro_boottime_ns, previous = s->last_imu_ns;
	bool fresh = (p->flags & (A081_TRACKING_IMU | A081_TRACKING_CLOCK)) ==
	             (A081_TRACKING_IMU | A081_TRACKING_CLOCK) &&
	             p->sample_monotonic_ns && p->sample_monotonic_ns <= p->send_monotonic_ns &&
	             p->send_monotonic_ns <= now && now - p->sample_monotonic_ns < A081_TRACKING_MAX_AGE_NS &&
	             p->clock_bracket_ns <= A081_TRACKING_MAX_BRACKET_NS &&
	             p->accel_boottime_ns && p->accel_boottime_ns <= time &&
	             time - p->accel_boottime_ns < A081_IMU_GAP_NS;
	for (unsigned i = 0; i < 3; ++i)
		fresh = fresh && isfinite(p->accel[i]) && isfinite(p->gyro[i]);
	struct xrt_vec3 raw_accel = {p->accel[0], p->accel[1], p->accel[2]};
	struct xrt_vec3 raw_gyro = {p->gyro[0], p->gyro[1], p->gyro[2]};
	fresh = fresh && time && a081_head_length(raw_accel) < 200.f && a081_head_length(raw_gyro) < 35.f;
	if (!fresh || (previous && (time <= previous || time - previous > A081_IMU_GAP_NS))) {
		a081_head_recovery_reset(s, fusion);
		return tracked; // Next coherent sample starts recovery; never bridge a missing IMU interval.
	}
	s->last_imu_ns = time;
	float dt = previous ? (float)(time - previous) / 1e9f : 0;
	// Same Android-portrait basis as a081_qvr_pose; one shared IMU owner.
	struct xrt_vec3 accel = {-p->accel[1], p->accel[0], p->accel[2]};
	struct xrt_vec3 gyro = {-p->gyro[1], p->gyro[0], p->gyro[2]};
	m_imu_3dof_update(fusion, time, &accel, &gyro);
	if (!tracked) {
		s->stable_since_ns = s->blend_since_ns = 0;
		if (s->anchored) a081_head_infer(s, fusion, accel, gyro, time, previous);
		return false;
	}
	if (s->mode == A081_HEAD_TRACKED) {
		s->output = measured;
	} else {
		if (!s->stable_since_ns) s->stable_since_ns = time;
		if (time - s->stable_since_ns < A081_RECOVER_STABLE_NS) {
			if (s->anchored) a081_head_infer(s, fusion, accel, gyro, time, previous);
			return true;
		}
		if (!s->anchored) {
			s->output = measured; // First localization: there is no old world to blend from.
		} else {
			if (!s->blend_since_ns) {
				a081_head_infer(s, fusion, accel, gyro, time, previous);
				s->position_error = (struct xrt_vec3){
				    s->output.pose.position.x - measured.pose.position.x,
				    s->output.pose.position.y - measured.pose.position.y,
				    s->output.pose.position.z - measured.pose.position.z};
				struct xrt_quat inverse;
				math_quat_invert(&measured.pose.orientation, &inverse);
				math_quat_rotate(&s->output.pose.orientation, &inverse, &s->orientation_error);
				// Large map corrections should not drag the world through the user.
				// Withdraw validity, then require a fresh stable localization.
				if (a081_head_length(s->position_error) > .25f ||
				    fabsf(s->orientation_error.w) < .98480775f) {
					a081_head_recovery_reset(s, fusion);
					return true;
				}
				s->blend_since_ns = time;
			}
			float t = fminf(1.f, (float)(time - s->blend_since_ns) / (float)A081_RECOVER_BLEND_NS);
			float weight = 1.f - t * t * (3.f - 2.f * t);
			float derivative = -6.f * t * (1.f - t) / ((float)A081_RECOVER_BLEND_NS / 1e9f);
			s->output = measured;
			s->output.pose.position.x += s->position_error.x * weight;
			s->output.pose.position.y += s->position_error.y * weight;
			s->output.pose.position.z += s->position_error.z * weight;
			s->output.linear_velocity.x += s->position_error.x * derivative;
			s->output.linear_velocity.y += s->position_error.y * derivative;
			s->output.linear_velocity.z += s->position_error.z * derivative;
			struct xrt_quat correction, identity = {0, 0, 0, 1};
			math_quat_slerp(&identity, &s->orientation_error, weight, &correction);
			math_quat_rotate(&correction, &measured.pose.orientation, &s->output.pose.orientation);
			if (t < 1.f) {
				s->mode = A081_HEAD_RECOVERING;
				s->output.relation_flags &= ~(XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
				                              XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT |
				                              XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
				// No compositor prediction using an uncorrected angular velocity.
				a081_head_anchor(s, fusion);
				return true;
			}
		}
	}
	s->mode = A081_HEAD_TRACKED;
	s->anchored = true;
	s->translation_stopped = false;
	s->last_tracked_ns = time;
	s->last_position = measured.pose.position;
	s->velocity = measured.linear_velocity;
	a081_head_anchor(s, fusion);
	// Learn accelerometer residual only from trusted, nearly stationary poses.
	// This is volatile filter state; it never overwrites device calibration.
	if (dt > 0 && a081_head_length(measured.linear_velocity) < .05f && a081_head_length(gyro) < .08f) {
		struct xrt_vec3 world;
		math_quat_rotate_vec3(&measured.pose.orientation, &accel, &world);
		world.y -= 9.80665f;
		if (a081_head_length(world) < .5f) {
			float alpha = fminf(1.f, dt);
			s->accel_bias.x += alpha * (world.x - s->accel_bias.x);
			s->accel_bias.y += alpha * (world.y - s->accel_bias.y);
			s->accel_bias.z += alpha * (world.z - s->accel_bias.z);
		}
	}
	return true;
}
