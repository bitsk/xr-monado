// SPDX-License-Identifier: BSL-1.0
#pragma once

#include "xrt/xrt_defines.h"
#include "math/m_api.h"

// The MIX SDK applies gSensorHeadOffset after its portrait-to-view conversion,
// before recentering. In the existing android-portrait-v1 XR convention its
// camera-to-world position is p_sdk = p_body + R*h - h. h is already in that
// head/view basis: it must not be portrait-rotated or negated again.
// Preserve the shared raw tracking world used by head, controllers and hands:
// p_eye = p_sdk + h = p_body + R*h. The SDK's constant world-origin shift
// -h must not be applied only to the head while other tracked points stay raw.
// Factory svrapi_config.txt: gSensorHeadOffset{X,Y,Z}, in metres.
// The DSP/body record, recovery state and observed-IMU anchor remain unchanged.
// Apply this once, after advancement, to a presentation copy of that relation.
static inline void
a081_eye_center_relation(struct xrt_space_relation *relation)
{
	const enum xrt_space_relation_flags pose_valid =
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT;
	if ((relation->relation_flags & pose_valid) != pose_valid) return;

	const struct xrt_vec3 h = {-.057022f, .032557f, .068516f};
	struct xrt_vec3 arm;
	math_quat_rotate_vec3(&relation->pose.orientation, &h, &arm);
	relation->pose.position.x += arm.x;
	relation->pose.position.y += arm.y;
	relation->pose.position.z += arm.z;

	// Differentiate the same rotating arm in world space. A body-origin
	// linear velocity alone is not the velocity of the center of the eyes.
	if (relation->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) {
		if (relation->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) {
			const struct xrt_vec3 w = relation->angular_velocity;
			relation->linear_velocity.x += w.y * arm.z - w.z * arm.y;
			relation->linear_velocity.y += w.z * arm.x - w.x * arm.z;
			relation->linear_velocity.z += w.x * arm.y - w.y * arm.x;
		} else {
			relation->linear_velocity = (struct xrt_vec3){0};
			relation->relation_flags &= ~XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
		}
	}
}
