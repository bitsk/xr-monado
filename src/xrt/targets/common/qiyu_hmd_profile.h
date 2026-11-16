// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <stdlib.h>
#include <string.h>

struct qiyu_hmd_profile
{
	const char *device;
	const char *model;
	const char *serial;
	const char *dt_model;
	float frustum_width_mm;
	float frustum_height_mm;
	float frustum_half_width_m;
	float frustum_half_height_m;
	float lens_even[3];
};

static const struct qiyu_hmd_profile qiyu_a081_profile = {
	"a081", "Qiyu MIX A081", "A081-LINUX-IIO", "A081",
	82.4f, 91.6f, .0609f, .068f, {.218861984f, -.026857263f, .178005457f},
};

static const struct qiyu_hmd_profile qiyu_vr700_profile = {
	"vr700", "Qiyu DREAM PRO VR700", "VR700-LINUX-IIO", "VR700",
	77.4746528f, 77.4746528f, .0594416876f, .0594416876f,
	{.213878493f, -.015922299f, .152495336f},
};

static inline const struct qiyu_hmd_profile *
qiyu_selected_profile(void)
{
	const char *device = getenv("XR_DEVICE");
	if (!device || !strcmp(device, "a081")) return &qiyu_a081_profile;
	if (!strcmp(device, "vr700")) return &qiyu_vr700_profile;
	return NULL;
}

static inline const struct qiyu_hmd_profile *
qiyu_profile_for_serial(const char *serial)
{
	if (!strcmp(serial, qiyu_a081_profile.serial)) return &qiyu_a081_profile;
	if (!strcmp(serial, qiyu_vr700_profile.serial)) return &qiyu_vr700_profile;
	return NULL;
}
