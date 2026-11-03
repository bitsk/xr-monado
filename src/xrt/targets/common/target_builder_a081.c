// SPDX-License-Identifier: BSL-1.0
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_system.h"
#include "util/u_misc.h"
#include "target_builder_helpers.h"
#include <stdlib.h>

extern bool a081_present(void);
extern struct xrt_device *a081_create(void);
static const char *drivers[] = {"a081"};

static xrt_result_t estimate(struct xrt_builder *b, cJSON *c, struct xrt_prober *p, struct xrt_builder_estimate *e)
{
	(void)b; (void)c; (void)p;
	e->certain.head = a081_present(); e->priority = 100;
	return XRT_SUCCESS;
}
static xrt_result_t open_system(struct xrt_builder *b, cJSON *c, struct xrt_prober *p,
    struct xrt_tracking_origin *o, struct xrt_system_devices *s, struct xrt_frame_context *f,
    struct t_builder_roles_helper *roles)
{
	(void)b; (void)c; (void)p; (void)o; (void)f;
	struct xrt_device *head = a081_create();
	if (!head) return XRT_ERROR_DEVICE_CREATION_FAILED;
	s->static_xdevs[s->static_xdev_count++] = head;
	roles->head = head;
	return XRT_SUCCESS;
}
static void destroy_builder(struct xrt_builder *b) { free(b); }
struct xrt_builder *t_builder_a081_create(void)
{
	struct t_builder *b = U_TYPED_CALLOC(struct t_builder);
	b->base.estimate_system = estimate; b->base.open_system = t_builder_open_system_static_roles;
	b->base.destroy = destroy_builder; b->base.identifier = "a081"; b->base.name = "A081 Linux IIO";
	b->base.driver_identifiers = drivers; b->base.driver_identifier_count = 1;
	b->open_system_static_roles = open_system;
	return &b->base;
}
