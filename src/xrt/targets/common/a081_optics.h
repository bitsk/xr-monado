// SPDX-License-Identifier: BSL-1.0
// Qiyu stock optical profiles. See devices/<model>/optics-profile.json.
#pragma once

#include "xrt/xrt_device.h"
#include "qiyu_hmd_profile.h"
#include <math.h>

static xrt_result_t
a081_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v,
                        struct xrt_uv_triplet *out)
{
	const struct qiyu_hmd_profile *profile = qiyu_profile_for_serial(xdev->serial);
	if (!profile || view >= 2 || !isfinite(u) || !isfinite(v)) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	// The caller has already rotated panel coordinates into the upright eye.
	// SXR builds an oversize mesh with position +/-FrustumDisplay/Display,
	// while its base texture coordinates span [0,1]. Evaluate that mesh at
	// the actual panel position, not at its overscan boundary.
	const float x = 2.0f * u - 1.0f;
	const float y = 2.0f * v - 1.0f;
	const float scale_x = profile->frustum_width_mm / 60.1812f;
	const float scale_y = profile->frustum_height_mm / 63.072f;
	// GenerateWarpCoords' samplePoint branch uses width/4 as focal length
	// in pixels, with width=3664 and height=1920 BEFORE panel rotation.
	const float aspect = 1920.0f / 1832.0f;
	const float r2 = x * x + y * y * aspect * aspect;
	const float radial = 1.0f + r2 * (profile->lens_even[0] + r2 * (profile->lens_even[1] + r2 * profile->lens_even[2]));
	const float sx = x * radial / scale_x;
	const float sy = y * radial / scale_y;
	out->r = (struct xrt_vec2){0.5f + 0.5f * sx * 0.9965f, 0.5f + 0.5f * sy * 0.9965f};
	out->g = (struct xrt_vec2){0.5f + 0.5f * sx, 0.5f + 0.5f * sy};
	out->b = (struct xrt_vec2){0.5f + 0.5f * sx * 1.0040f, 0.5f + 0.5f * sy * 1.0040f};
	// Keep out-of-image UVs: the compositor must show the black border,
	// rather than stretching the last texel across the lens periphery.
	return XRT_SUCCESS;
}

static void
a081_setup_optics(struct xrt_device *xdev)
{
	xdev->compute_distortion = a081_compute_distortion;
	xdev->hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	xdev->hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
}
