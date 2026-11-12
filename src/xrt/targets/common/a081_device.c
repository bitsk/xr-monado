// SPDX-License-Identifier: BSL-1.0
// A081 real IIO or explicit experimental QVR pose stream; no synthetic input.
#include "xrt/xrt_device.h"
#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "os/os_time.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "a081_optics.h"
#include "util/u_logging.h"
#include <xr/head/production-v2/a081_tracking_client.h>
#include <xr/head/production-v2/a081_qvr_pose.h>
#include "a081_tracking_recovery.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

struct saved_attr { char path[512], value[128]; };
struct stream {
	int fd;
	char path[256];
	struct saved_attr saved[8];
	unsigned saved_count;
	bool enabled;
	double scale;
	uint64_t samples;
	int64_t time_ns;
	struct xrt_vec3 value;
};
struct a081 {
	struct xrt_device base;
	struct stream accel, gyro;
	struct m_imu_3dof fusion;
	pthread_mutex_t mutex;
	pthread_t thread;
	atomic_bool running;
	bool started;
	int lock_fd;
	int64_t fused_time_ns;
	uint64_t fused_samples;
	bool stock_stream, stock_pose;
	struct xrt_space_relation stock_relation;
	struct a081_head_recovery recovery;
	uint64_t positive_poses;
	char tracking_socket[108];
	uint64_t stream_packets, stock_pose_packets;
	int64_t stream_start_ns;
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

static bool write_text(const char *path, const char *value)
{
	int fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd < 0) return false;
	size_t n = strlen(value);
	bool ok = write(fd, value, n) == (ssize_t)n;
	close(fd);
	return ok;
}

static bool change_attr(struct stream *s, const char *attr, const char *value)
{
	if (s->saved_count >= 8) return false;
	struct saved_attr *old = &s->saved[s->saved_count];
	snprintf(old->path, sizeof(old->path), "%s/%s", s->path, attr);
	if (!read_text(old->path, old->value, sizeof(old->value))) return false;
	if (!write_text(old->path, value)) return false;
	s->saved_count++;
	return true;
}

static void close_stream(struct stream *s)
{
	if (s->enabled) {
		char path[512]; snprintf(path, sizeof(path), "%s/buffer/enable", s->path);
		if (!write_text(path, "0")) U_LOG_E("A081: failed to disable %s", path);
	}
	if (s->fd >= 0) close(s->fd);
	while (s->saved_count) {
		struct saved_attr *a = &s->saved[--s->saved_count];
		if (!write_text(a->path, a->value)) U_LOG_E("A081: failed to restore %s", a->path);
	}
	s->fd = -1;
}

static bool open_stream(struct stream *s, const char *name, const char *channel)
{
	DIR *dir = opendir("/sys/bus/iio/devices");
	if (!dir) return false;
	struct dirent *e;
	char node[256] = "", path[512], value[128];
	while ((e = readdir(dir))) {
		if (strncmp(e->d_name, "iio:device", 10)) continue;
		snprintf(path, sizeof(path), "/sys/bus/iio/devices/%s/name", e->d_name);
		if (read_text(path, value, sizeof(value)) && !strcmp(value, name)) {
			snprintf(node, sizeof(node), "%s", e->d_name); break;
		}
	}
	closedir(dir);
	if (!node[0]) { U_LOG_E("A081: missing %s", name); return false; }
	snprintf(s->path, sizeof(s->path), "/sys/bus/iio/devices/%.200s", node);
	snprintf(path, sizeof(path), "%s/buffer/enable", s->path);
	if (!read_text(path, value, sizeof(value)) || strcmp(value, "0")) {
		U_LOG_E("A081: %s buffer already owned by another reader", name); return false;
	}
	// Accept only the actual SEE ABI: xyz signed 32-bit SI micro-units,
	// padding to 16 bytes, followed by a signed 64-bit nanosecond timestamp.
	for (int i = 0; i < 4; i++) {
		char stem[100], expected[8];
		if (i < 3) snprintf(stem, sizeof(stem), "in_%s_%c", channel, "xyz"[i]);
		else snprintf(stem, sizeof(stem), "in_timestamp");
		snprintf(path, sizeof(path), "%s/scan_elements/%s_index", s->path, stem);
		snprintf(expected, sizeof(expected), "%d", i);
		if (!read_text(path, value, sizeof(value)) || strcmp(value, expected)) return false;
		snprintf(path, sizeof(path), "%s/scan_elements/%s_type", s->path, stem);
		if (!read_text(path, value, sizeof(value)) ||
		    strcmp(value, i < 3 ? "le:s32/32>>0" : "le:s64/64>>0")) return false;
		char attr[128]; snprintf(attr, sizeof(attr), "scan_elements/%s_en", stem);
		if (!change_attr(s, attr, "1")) return false;
	}
	snprintf(path, sizeof(path), "%s/in_%s_scale", s->path, channel);
	if (!read_text(path, value, sizeof(value))) return false;
	s->scale = strtod(value, NULL);
	if (!isfinite(s->scale) || s->scale <= 0) return false;
	if (!change_attr(s, "current_timestamp_clock", "monotonic") ||
	    !change_attr(s, "buffer/length", "256") || !change_attr(s, "buffer/watermark", "1")) return false;
	snprintf(path, sizeof(path), "%s/buffer/hwfifo_timeout", s->path);
	if (access(path, F_OK) == 0 && !change_attr(s, "buffer/hwfifo_timeout", "0")) return false;
	snprintf(path, sizeof(path), "/dev/%s", node);
	s->fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (s->fd < 0) return false;
	snprintf(path, sizeof(path), "%s/buffer/enable", s->path);
	if (!write_text(path, "1")) return false;
	s->enabled = true;
	U_LOG_I("A081 real IIO: %s %s scale=%g clock=monotonic", name, node, s->scale);
	return true;
}

static void *read_imu(void *ptr)
{
	struct a081 *d = ptr;
	struct pollfd p[2] = {{d->accel.fd, POLLIN, 0}, {d->gyro.fd, POLLIN, 0}};
	while (atomic_load(&d->running)) {
		int ready = poll(p, 2, 50);
		if (ready < 0 && errno != EINTR) break;
		for (int i = 0; i < 2 && ready > 0; i++) {
			if (p[i].revents & (POLLERR | POLLHUP | POLLNVAL)) goto done;
			if (!(p[i].revents & POLLIN)) continue;
			struct stream *s = i ? &d->gyro : &d->accel;
			struct { int32_t xyz[3], padding; int64_t time_ns; } sample;
			ssize_t n;
			// Process bounded batches so shutdown cannot starve.
			for (int batch = 0; batch < 256; batch++) {
				n = read(s->fd, &sample, sizeof(sample));
				if (n < 0 && (errno == EAGAIN || errno == EINTR)) break;
				if (n != sizeof(sample)) goto done;
				if (sample.time_ns <= s->time_ns) continue;
				s->time_ns = sample.time_ns;
				s->value = (struct xrt_vec3){sample.xyz[0] * s->scale, sample.xyz[1] * s->scale, sample.xyz[2] * s->scale};
				s->samples++;
				if (!i || !d->accel.time_ns || llabs(s->time_ns - d->accel.time_ns) > 100000000) continue;
				pthread_mutex_lock(&d->mutex);
				if (d->fused_time_ns && s->time_ns - d->fused_time_ns > 100000000) m_imu_3dof_reset(&d->fusion);
				m_imu_3dof_update(&d->fusion, s->time_ns, &d->accel.value, &s->value);
				d->fused_time_ns = s->time_ns;
				d->fused_samples++;
				if (d->fused_samples % 500 == 0)
					U_LOG_I("A081 real samples: accel=%lu gyro=%lu fused=%lu timestamp=%ld q=%g,%g,%g,%g",
					        d->accel.samples, d->gyro.samples, d->fused_samples, d->fused_time_ns,
					        d->fusion.rot.x, d->fusion.rot.y, d->fusion.rot.z, d->fusion.rot.w);
				pthread_mutex_unlock(&d->mutex);
			}
		}
	}
done:
	atomic_store(&d->running, false);
	return NULL;
}

static void reset_stream_tracking(struct a081 *d)
{
	pthread_mutex_lock(&d->mutex);
	a081_head_recovery_reset(&d->recovery, &d->fusion);
	d->fused_samples = 0;
	d->fused_time_ns = 0;
	d->stream_start_ns = 0;
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
			// A sequence gap does not erase a world origin: recovery checks the
			// actual IMU interval and refuses integration over a long gap.
			if (same_session && (packet.sequence <= sequence || packet.gyro_boottime_ns <= last_gyro)) {
				disconnected = true; break;
			}
			session = packet.session_id; sequence = packet.sequence; last_gyro = packet.gyro_boottime_ns;
			uint64_t now = (uint64_t)os_monotonic_get_ns();
			if (!(packet.flags & A081_TRACKING_CLOCK) || packet.sample_monotonic_ns > now ||
			    now - packet.sample_monotonic_ns >= A081_TRACKING_MAX_AGE_NS) {
				reset_stream_tracking(d); continue;
			}
			if (d->stock_pose) {
				pthread_mutex_lock(&d->mutex);
				enum a081_head_mode before = d->recovery.mode;
				bool valid = a081_head_recovery_update(&d->recovery, &d->fusion, &packet, now);
				struct xrt_space_relation relation = d->recovery.output;
				d->stock_relation = relation;
				if (before != d->recovery.mode)
					U_LOG_I("A081_HEAD_MODE=%s previous=%s seq=%lu raw_valid=%d relation_flags=%u",
					        a081_head_mode_name(d->recovery.mode), a081_head_mode_name(before),
					        packet.sequence, valid, (unsigned)relation.relation_flags);
				d->fused_time_ns = (int64_t)packet.sample_monotonic_ns;
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
				continue;
			}
			struct xrt_vec3 accel = {packet.accel[0], packet.accel[1], packet.accel[2]};
			struct xrt_vec3 gyro = {packet.gyro[0], packet.gyro[1], packet.gyro[2]};
			pthread_mutex_lock(&d->mutex);
			int64_t timestamp = (int64_t)packet.sample_monotonic_ns;
			if (d->fused_time_ns && timestamp <= d->fused_time_ns) {
				m_imu_3dof_reset(&d->fusion); d->fused_samples = 0;
				d->fused_time_ns = d->stream_start_ns = 0;
				pthread_mutex_unlock(&d->mutex); continue;
			}
			if (d->fused_time_ns && timestamp - d->fused_time_ns > 100000000) {
				m_imu_3dof_reset(&d->fusion); d->fused_samples = 0; d->stream_start_ns = 0;
			}
			if (!d->stream_start_ns) d->stream_start_ns = timestamp;
			m_imu_3dof_update(&d->fusion, timestamp, &accel, &gyro);
			d->fused_time_ns = timestamp; ++d->fused_samples;
			++d->stream_packets;
			if (packet.flags & A081_TRACKING_RAW_POSE) ++d->stock_pose_packets;
			pthread_mutex_unlock(&d->mutex);
			if (d->stream_packets == 1 || d->stream_packets % 500 == 0)
				U_LOG_I("A081 stock stream: packets=%lu pose_records=%lu session=%lu seq=%lu state=%u warning=%u quality=%g pose_read=%u position_valid=0",
				        d->stream_packets, d->stock_pose_packets, packet.session_id, packet.sequence,
				        packet.tracking_state, packet.warning_flags, packet.pose_quality, packet.pose_read_status);
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

// Private SteamVR bridge reads one measured pose with its matching sample time.
bool a081_qvr_latest_pose(struct xrt_device *xdev, struct xrt_space_relation *out, int64_t *sample_ns)
{
	struct a081 *d = (struct a081 *)xdev;
	if (!d->stock_pose) return false;
	memset(out, 0, sizeof(*out)); out->pose.orientation.w = 1;
	pthread_mutex_lock(&d->mutex);
	int64_t now = os_monotonic_get_ns();
	int64_t age = now - d->fused_time_ns;
	*sample_ns = now;
	if (atomic_load(&d->running) && age >= 0 && age < 100000000) {
		*out = d->stock_relation;
		*sample_ns = d->fused_time_ns;
	}
	pthread_mutex_unlock(&d->mutex);
	return true;
}

static xrt_result_t get_pose(struct xrt_device *xdev, enum xrt_input_name name, int64_t at, struct xrt_space_relation *out)
{
	(void)at; // No prediction until the board's IMU-to-eye transform is verified.
	struct a081 *d = (struct a081 *)xdev;
	if (name != XRT_INPUT_GENERIC_HEAD_POSE) return XRT_ERROR_INPUT_UNSUPPORTED;
	memset(out, 0, sizeof(*out)); out->pose.orientation.w = 1;
	pthread_mutex_lock(&d->mutex);
	int64_t age = os_monotonic_get_ns() - d->fused_time_ns;
	if (d->stock_pose) {
		if (atomic_load(&d->running) && age >= 0 && age < 100000000) *out = d->stock_relation;
		pthread_mutex_unlock(&d->mutex);
		return XRT_SUCCESS;
	}
	bool warmed_up = !d->stock_stream || (d->stream_start_ns && d->fused_time_ns - d->stream_start_ns >= 300000000);
	if (atomic_load(&d->running) && d->fused_samples >= 30 && warmed_up && age >= 0 && age < 100000000) {
		out->pose.orientation = d->fusion.rot;
		out->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
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
	U_LOG_I("A081 stopped: real accel=%lu gyro=%lu fused=%lu", d->accel.samples, d->gyro.samples, d->fused_samples);
	if (d->stock_stream) U_LOG_I("A081 stream stopped: packets=%lu raw_pose_records=%lu", d->stream_packets, d->stock_pose_packets);
	close_stream(&d->gyro); close_stream(&d->accel);
	if (d->lock_fd >= 0) close(d->lock_fd);
	m_imu_3dof_close(&d->fusion);
	pthread_mutex_destroy(&d->mutex);
	u_device_free(xdev);
}

bool a081_present(void)
{
	char model[128];
	return read_text("/proc/device-tree/model", model, sizeof(model)) && strstr(model, "A081");
}

struct xrt_device *a081_create(void)
{
	if (!a081_present() || !getenv("A081_BRINGUP") || strcmp(getenv("A081_BRINGUP"), "1")) {
		U_LOG_E("A081 real hardware and explicit A081_BRINGUP=1 required"); return NULL;
	}
	struct a081 *d = U_DEVICE_ALLOCATE(struct a081, U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	d->accel.fd = d->gyro.fd = d->lock_fd = -1;
	atomic_init(&d->running, false);
	pthread_mutex_init(&d->mutex, NULL);
	m_imu_3dof_init(&d->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_300MS);
	const char *source = getenv("A081_TRACKING_SOURCE");
	if (source && strcmp(source, "iio") && strcmp(source, "qvr-stream") && strcmp(source, "qvr-pose")) {
		U_LOG_E("A081_TRACKING_SOURCE must be iio, qvr-stream or qvr-pose"); destroy(&d->base); return NULL;
	}
	d->stock_pose = source && !strcmp(source, "qvr-pose");
	d->stock_stream = d->stock_pose || (source && !strcmp(source, "qvr-stream"));
	if (d->stock_pose) {
		const char *profile = getenv("A081_QVR_POSE_PROFILE");
		if (!profile || strcmp(profile, "android-portrait-v1")) {
			U_LOG_E("qvr-pose requires explicit A081_QVR_POSE_PROFILE=android-portrait-v1");
			destroy(&d->base); return NULL;
		}
	}
	if (d->stock_stream) {
		const char *path = getenv("A081_QVR_TRACKING_SOCKET");
		if (!path || path[0] != '/' || strlen(path) >= sizeof(d->tracking_socket)) {
			U_LOG_E("qvr-stream requires an absolute A081_QVR_TRACKING_SOCKET"); destroy(&d->base); return NULL;
		}
		strcpy(d->tracking_socket, path);
	} else {
		// Share the same global ownership lock with the original-DSP producer.
		d->lock_fd = open("/run/a081-iio.lock", O_CREAT | O_RDWR | O_CLOEXEC, 0600);
		if (d->lock_fd < 0 || flock(d->lock_fd, LOCK_EX | LOCK_NB) ||
		    !open_stream(&d->accel, "qcom-see-accel", "accel") || !open_stream(&d->gyro, "qcom-see-gyro", "anglvel")) {
			U_LOG_E("A081 IIO open failed: %s", strerror(errno)); destroy(&d->base); return NULL;
		}
	}
	struct xrt_device *xdev = &d->base;
	xdev->name = XRT_DEVICE_GENERIC_HMD; xdev->device_type = XRT_DEVICE_TYPE_HMD;
	snprintf(xdev->str, sizeof(xdev->str), "Qiyu MIX A081 %s (uncalibrated)", d->stock_pose ? "QVR experimental 6DoF" : d->stock_stream ? "QVR input 3DoF" : "real IIO 3DoF");
	snprintf(xdev->serial, sizeof(xdev->serial), "A081-LINUX-IIO");
	xdev->inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	xdev->supported.orientation_tracking = true; xdev->supported.position_tracking = d->stock_pose;
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
		xdev->hmd->distortion.fov[i] = (struct xrt_fov){-atanf(.0609f/.0578449f), atanf(.0609f/.0578449f),
		                                                        atanf(.068f/.0578449f), -atanf(.068f/.0578449f)};
	}
	a081_setup_optics(xdev);
	U_LOG_I("A081 stock optical map: display FOV, RGB polynomial, panel overscan scale; physical acceptance pending");
	if (d->stock_pose)
		U_LOG_W("A081 experimental QVR pose: SDK android-portrait-v1, raw tracker origin (IMU-reference assumed), metric scale from factory calibration; physical axes/scale/eye offset/accuracy uncalibrated; OEM motion state for runtime prediction");
	else
		U_LOG_W("A081 bring-up: source=%s; real IMU 3DoF, provisional optics, NO position or controllers", d->stock_stream ? "qvr-stream" : "iio");
	atomic_store(&d->running, true);
	if (pthread_create(&d->thread, NULL, d->stock_stream ? read_tracking_stream : read_imu, d)) { destroy(xdev); return NULL; }
	d->started = true;
	return xdev;
}
