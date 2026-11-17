// SPDX-License-Identifier: BSL-1.0
// Qiyu HMD consumes the sole OEM body-pose stream; hardware belongs to device-io.
#include "xrt/xrt_device.h"
#include "math/m_api.h"
#include "os/os_time.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "a081_optics.h"
#include "util/u_logging.h"
#include <xr/head/production-v2/a081_tracking_client.h>
#include <xr/head/production-v2/a081_qvr_pose.h>
#include "a081_eye_origin.h"
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

struct a081 {
	struct xrt_device base;
	pthread_mutex_t mutex;
	pthread_t thread;
	atomic_bool running;
	bool started;
	int64_t sample_time_ns;
	struct xrt_space_relation stock_relation;
	uint64_t positive_poses;
	char tracking_socket[108];
	uint64_t pose_anchor_ns, pose_state_ns, pose_query_ns;
	unsigned pose_query_flags;
	uint64_t stream_packets, stock_pose_packets;
};

static bool read_text(const char *path, char *value, size_t size)
{
	FILE *f = fopen(path, "r");
	if (!f) return false;
	bool ok = fgets(value, size, f) != NULL;
	fclose(f);
	if (ok) value[strcspn(value, "\r\n")] = 0;
	return ok;
}

static void reset_stream_tracking(struct a081 *d)
{
	pthread_mutex_lock(&d->mutex);
	d->sample_time_ns = 0;
	memset(&d->stock_relation, 0, sizeof(d->stock_relation));
	pthread_mutex_unlock(&d->mutex);
}

static void *read_tracking_stream(void *ptr)
{
	struct a081 *d = ptr;
	int fd = -1;
	uint64_t session = 0, sequence = 0, last_gyro = 0;
	int64_t retry_at = 0;
	while (atomic_load(&d->running)) {
		if (fd < 0) {
			int64_t now = os_monotonic_get_ns();
			if (now >= retry_at) {
				fd = a081_tracking_connect(d->tracking_socket);
				retry_at = now + 250000000;
				if (fd >= 0) { session = sequence = last_gyro = 0; reset_stream_tracking(d); }
			}
			if (fd < 0) { poll(NULL, 0, 20); continue; }
		}
		struct pollfd p = {fd, POLLIN, 0};
		int ready = poll(&p, 1, 50);
		if (ready < 0 && errno == EINTR) continue;
		bool disconnected = ready < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL));
		for (int batch = 0; !disconnected && ready > 0 && batch < 256; ++batch) {
			struct a081_tracking_packet packet;
			int rc = a081_tracking_receive(fd, &packet);
			if (!rc) break;
			if (rc < 0) { disconnected = true; break; }
			bool same_session = session == packet.session_id;
			if (!same_session) reset_stream_tracking(d);
			// A sequence gap does not erase a world origin. The next coherent
			// OEM pose owns its state.
			if (same_session && (packet.sequence <= sequence || packet.gyro_boottime_ns <= last_gyro)) {
				disconnected = true; break;
			}
			session = packet.session_id; sequence = packet.sequence; last_gyro = packet.gyro_boottime_ns;
			uint64_t now = (uint64_t)os_monotonic_get_ns();
			if (!(packet.flags & A081_TRACKING_CLOCK) || packet.sample_monotonic_ns > now) {
				reset_stream_tracking(d); continue;
			}
			pthread_mutex_lock(&d->mutex);
			const enum xrt_space_relation_flags before = d->stock_relation.relation_flags;
			struct xrt_space_relation relation;
			bool valid = a081_qvr_pose(&packet, now, &relation);
			// The OEM high-rate estimator already carries its own IMU,
			// visual corrections and quality. Publish one estimator rather
			// than switching to a second filter on position-quality changes.
			d->stock_relation = relation;
			if (before != relation.relation_flags)
				U_LOG_I("A081_HEAD_STATE seq=%lu raw_valid=%d relation_flags=%u position_quality=%g",
				        packet.sequence, valid, (unsigned)relation.relation_flags, packet.pose_quality);
			d->sample_time_ns = (int64_t)packet.sample_monotonic_ns;
			++d->stream_packets;
			if (packet.flags & A081_TRACKING_RAW_POSE) ++d->stock_pose_packets;
			if (valid) ++d->positive_poses;
			pthread_mutex_unlock(&d->mutex);
			if (d->stream_packets == 1 || d->stream_packets % 500 == 0)
				U_LOG_I("A081 QVR pose: packets=%lu accepted=%lu session=%lu seq=%lu quality=%g state=%u valid=%d p=%g,%g,%g q=%g,%g,%g,%g accuracy=uncalibrated origin=raw_tracker",
				        d->stream_packets, d->positive_poses, packet.session_id, packet.sequence,
				        packet.pose_quality, packet.tracking_state, valid,
				        relation.pose.position.x, relation.pose.position.y, relation.pose.position.z,
				        relation.pose.orientation.x, relation.pose.orientation.y, relation.pose.orientation.z, relation.pose.orientation.w);
		}
		if (disconnected) {
			close(fd); fd = -1; reset_stream_tracking(d);
			U_LOG_I("A081 tracking stream disconnected; all pose validity cleared");
		}
	}
	if (fd >= 0) close(fd);
	reset_stream_tracking(d);
	return NULL;
}

// Private SteamVR bridge returns body state at its observed IMU epoch.
// The static body-to-eye extrinsic is owned by the OpenVR binding so its
// runtime applies it after prediction. Raw OEM packets are never rewritten.
bool a081_qvr_latest_pose(struct xrt_device *xdev, struct xrt_space_relation *out, int64_t *sample_ns)
{
	struct a081 *d = (struct a081 *)xdev;
	memset(out, 0, sizeof(*out)); out->pose.orientation.w = 1;
	pthread_mutex_lock(&d->mutex);
	int64_t now = os_monotonic_get_ns();
	*sample_ns = now;
	d->pose_anchor_ns = d->sample_time_ns > 0 ? (uint64_t)d->sample_time_ns : 0;
	if (atomic_load(&d->running) && d->sample_time_ns > 0) {
		*out = d->stock_relation;
		*sample_ns = d->sample_time_ns;
		// OpenVR predicts this body state, then applies DriverFromHead once.
		// Do not pre-shift to the eyes or pre-add their tangential velocity.
	}
	// Pair timing with this exact query, including invalid returned states.
	d->pose_state_ns = (out->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) ? *sample_ns : 0;
	d->pose_query_ns = os_monotonic_get_ns();
	d->pose_query_flags = out->relation_flags;
	pthread_mutex_unlock(&d->mutex);
	return true;
}

// On-demand diagnostics only: no polling writer and no persistent trace.
void a081_qvr_pose_timing(struct xrt_device *xdev, char *buffer, unsigned size)
{
    struct a081 *d=(struct a081 *)xdev;
    pthread_mutex_lock(&d->mutex);
    const unsigned flags = d->pose_query_flags;
    const char *mode = (flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) ? "tracked" :
                       (flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) ? "orientation_only" : "lost";
    snprintf(buffer,size,"{\"mode\":\"%s\",\"anchor_mono_ns\":%llu,\"state_mono_ns\":%llu,\"query_mono_ns\":%llu,\"producer_anchor_mono_ns\":%lld,\"relation_flags\":%u,\"state_reference\":\"tracker_body\",\"head_transform\":\"runtime_post_prediction\",\"prediction_owner\":\"SteamVR\",\"physical_verified\":false}",
             mode,(unsigned long long)d->pose_anchor_ns,(unsigned long long)d->pose_state_ns,
             (unsigned long long)d->pose_query_ns,(long long)d->sample_time_ns,flags);
    pthread_mutex_unlock(&d->mutex);
}

static xrt_result_t get_pose(struct xrt_device *xdev, enum xrt_input_name name, int64_t at, struct xrt_space_relation *out)
{
	(void)at; // This endpoint publishes the measured state; its runtime owns future prediction.
	struct a081 *d = (struct a081 *)xdev;
	if (name != XRT_INPUT_GENERIC_HEAD_POSE) return XRT_ERROR_INPUT_UNSUPPORTED;
	memset(out, 0, sizeof(*out)); out->pose.orientation.w = 1;
	pthread_mutex_lock(&d->mutex);
	if (atomic_load(&d->running) && d->sample_time_ns > 0) {
		*out = d->stock_relation;
		if (qiyu_profile_for_serial(xdev->serial) == &qiyu_a081_profile)
			a081_eye_center_relation(out);
	}
	pthread_mutex_unlock(&d->mutex);
	return XRT_SUCCESS;
}

static xrt_result_t get_views(struct xrt_device *d, const struct xrt_vec3 *eye_relation, int64_t at,
                     enum xrt_view_type view_type, uint32_t count,
                     struct xrt_space_relation *head, struct xrt_fov *fovs, struct xrt_pose *poses)
{
	(void)eye_relation;
	const struct xrt_vec3 calibrated_ipd = {0.0634f, 0, 0}; // Stock svrapi_config, not a measured current IPD.
	return u_device_get_view_poses(d, &calibrated_ipd, at, view_type, count, head, fovs, poses);
}

static void destroy(struct xrt_device *xdev)
{
	struct a081 *d = (struct a081 *)xdev;
	atomic_store(&d->running, false);
	if (d->started) pthread_join(d->thread, NULL);
	U_LOG_I("A081 stopped: packets=%lu raw_pose_records=%lu", d->stream_packets, d->stock_pose_packets);
	pthread_mutex_destroy(&d->mutex);
	u_device_free(xdev);
}

bool a081_present(void)
{
	char model[128];
	const struct qiyu_hmd_profile *profile = qiyu_selected_profile();
	return profile && read_text("/proc/device-tree/model", model, sizeof(model)) && strstr(model, profile->dt_model);
}

struct xrt_device *a081_create(void)
{
	if (!a081_present() || !getenv("A081_BRINGUP") || strcmp(getenv("A081_BRINGUP"), "1")) {
		U_LOG_E("Matching Qiyu hardware, XR_DEVICE=a081|vr700 and A081_BRINGUP=1 required"); return NULL;
	}
	const struct qiyu_hmd_profile *profile = qiyu_selected_profile();
	struct a081 *d = U_DEVICE_ALLOCATE(struct a081, U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	atomic_init(&d->running, false);
	pthread_mutex_init(&d->mutex, NULL);
	const char *pose_profile = getenv("A081_QVR_POSE_PROFILE");
	if (!pose_profile || strcmp(pose_profile, "android-portrait-v1")) {
		U_LOG_E("QVR requires A081_QVR_POSE_PROFILE=android-portrait-v1");
		destroy(&d->base); return NULL;
	}
	const char *path = getenv("A081_QVR_TRACKING_SOCKET");
	if (!path || path[0] != '/' || strlen(path) >= sizeof(d->tracking_socket)) {
		U_LOG_E("QVR requires an absolute A081_QVR_TRACKING_SOCKET"); destroy(&d->base); return NULL;
	}
	strcpy(d->tracking_socket, path);
	struct xrt_device *xdev = &d->base;
	xdev->name = XRT_DEVICE_GENERIC_HMD; xdev->device_type = XRT_DEVICE_TYPE_HMD;
	snprintf(xdev->str, sizeof(xdev->str), "%s QVR 6DoF", profile->model);
	snprintf(xdev->serial, sizeof(xdev->serial), "%s", profile->serial);
	xdev->inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	xdev->supported.orientation_tracking = true; xdev->supported.position_tracking = true;
	xdev->tracking_origin->type = XRT_TRACKING_TYPE_OTHER;
	xdev->update_inputs = u_device_noop_update_inputs;
	xdev->get_tracked_pose = get_pose; xdev->get_view_poses = get_views;
	xdev->get_visibility_mask = u_device_get_visibility_mask; xdev->destroy = destroy;
	xdev->hmd->view_count = 2;
	xdev->hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	xdev->hmd->blend_mode_count = 1;
	xdev->hmd->screens[0].w_pixels = 1920; xdev->hmd->screens[0].h_pixels = 3664;
	xdev->hmd->screens[0].nominal_frame_interval_ns = 1000000000 / 72;
	for (unsigned i = 0; i < 2; i++) {
		xdev->hmd->views[i].display.w_pixels = 1832; xdev->hmd->views[i].display.h_pixels = 1920;
		xdev->hmd->views[i].viewport.x_pixels = 0; xdev->hmd->views[i].viewport.y_pixels = 1832 * i;
		xdev->hmd->views[i].viewport.w_pixels = 1920; xdev->hmd->views[i].viewport.h_pixels = 1832;
		xdev->hmd->views[i].rot = u_device_rotation_right;
		xdev->hmd->distortion.fov[i] = (struct xrt_fov){-atanf(profile->frustum_half_width_m/.0578449f), atanf(profile->frustum_half_width_m/.0578449f),
		                                                        atanf(profile->frustum_half_height_m/.0578449f), -atanf(profile->frustum_half_height_m/.0578449f)};
	}
	a081_setup_optics(xdev);
	U_LOG_I("%s stock optical map: display FOV, RGB polynomial, panel overscan scale; physical acceptance pending", profile->model);
	U_LOG_W("%s QVR pose: SDK android-portrait-v1, %s; metric factory calibration; physical acceptance pending; OEM motion state for runtime prediction", profile->model,
	        profile == &qiyu_a081_profile ? "factory body-to-eye extrinsic, shared tracking world; OpenVR applies it after prediction" : "raw tracking origin");
	atomic_store(&d->running, true);
	if (pthread_create(&d->thread, NULL, read_tracking_stream, d)) { destroy(xdev); return NULL; }
	d->started = true;
	return xdev;
}
