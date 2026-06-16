/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 *
 * changed by Vladimir Zdravkov to match requirents of the cap project for ECEN 5623 at the University of Colorado Boulder
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>

#include <syslog.h>

#include <getopt.h> /* getopt_long() */

#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <time.h>
#include <limits.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define MAX_HRES (1920)
#define MAX_VRES (1080)
#define MAX_PIXEL_SIZE (3) // for RGB24, which is the largest pixel size we will be using, so that we can allocate a scratchpad buffer that is large enough for any format we will be using

#define HRES (640)
#define VRES (480)
#define PIXEL_SIZE (2)
#define HRES_STR "640"
#define VRES_STR "480"

// #define HRES (320)
// #define VRES (240)
// #define PIXEL_SIZE (2)
// #define HRES_STR "320"
// #define VRES_STR "240"

#define STARTUP_FRAMES (30)
#define AUTOFOCUS_OFF_FRAMES (15)
#define LAST_FRAMES (1)
#define CAPTURE_FRAMES (300 + LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + STARTUP_FRAMES + LAST_FRAMES)
#define FRAMES_TO_SKIP (1)

// #define FRAMES_PER_SEC (1)
#define FRAMES_PER_SEC (10)
// #define FRAMES_PER_SEC (20)
// #define FRAMES_PER_SEC (25)
// #define FRAMES_PER_SEC (30)

#define COLOR_CONVERT_RGB
// #define COLOR_CONVERT_GRAY
#define DUMP_FRAMES
#define RING_BUF_RESERVE_FRAMES (3)
#define FRAMES_PER_ACQ_PERIOD (3) // input ring buffer size should be large enough, so that we can have a good chance of not losing frames while we are processing and saving frames, but also not so large that we are wasting a lot of memory on the ring buffer
#define DRIVER_MMAP_BUFFERS (6)       // request buffers for delay
#define RING_OUTPUT_BUFFER_SIZE (FRAMES_PER_ACQ_PERIOD + RING_BUF_RESERVE_FRAMES)
#define DIFF_MIN (0.45f)                   // value from experiment
#define DIFF_MAX (0.65f)                   // value from experiment
#define CAPTURE_HEAD_STABILITY_PERIODS (3) // require this many consecutive periods with the same computed head_idx before adopting it

// adjacent string literals are concatenated at compile time, so we can use this to build our log message format string
#define COURSE_FRM_CAPT_SYSLOG(crs, nmb) "[COURSE " crs "][" nmb "][Frame Count: %d][Image Capture Start Time: %lf seconds]"
#define COURSE "#:4"
#define ASS "Final Project"

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;
struct v4l2_buffer frame_buf;

struct buffer
{
    void *start;
    size_t length;
};

struct save_frame_t
{
    unsigned char frame[HRES * VRES * PIXEL_SIZE];
    struct timespec time_stamp;
    char identifier_str[80];
    bool is_different_from_previous;
    bool is_selected_to_save;
};

struct ring_buffer_t
{
    unsigned int ring_size;

    unsigned int tail_idx;
    unsigned int head_idx;
    int count;

    struct save_frame_t save_frame[FRAMES_PER_ACQ_PERIOD * FRAMES_PER_SEC];
};

static struct ring_buffer_t ring_buffer;

struct save_out_frame_t
{
    unsigned char frame[HRES * VRES * PIXEL_SIZE * 3]; // for RGB24, which is the largest pixel size we will be using, so that we can allocate a buffer that is large enough for any format we will be using
    struct timespec time_stamp;
    char identifier_str[80];
    bool is_ready_to_save;
    bool is_filter_applied;
};

struct ring_out_buffer_t
{
    unsigned int ring_size;

    unsigned int tail_idx;
    unsigned int head_idx;
    int count;

    struct save_out_frame_t save_out_frame[FRAMES_PER_ACQ_PERIOD * FRAMES_PER_SEC];
};

static struct ring_out_buffer_t ring_output_buffer;

unsigned char scratchpad_buffer[MAX_HRES * MAX_VRES * MAX_PIXEL_SIZE]; // this is used for processing and saving frames, so that we don't have to worry about the ring buffer being overwritten by the acquisition loop while we are processing or saving a frame
bool is_scratchpad_buffer_in_use = false;                              // this is used to indicate whether the scratchpad buffer is currently being used for processing or saving a frame, so that we can avoid overwriting it with new frames from the acquisition loop while we are processing or saving a frame

static unsigned int diff_counter = 0;
static unsigned int diff_counter2 = 0;
static double diff_frame[2] = {0.0, 0.0};
static double diff_frame2[2] = {0.0, 0.0};

static unsigned int first_startup_head_idx = UINT_MAX;
static unsigned int second_startup_head_idx = UINT_MAX;
static unsigned int sel_startup_head_idx = UINT_MAX;
static unsigned int capture_head_idx = UINT_MAX;
static unsigned int header_idx_delay_counter = 0;

static int camera_device_fd = -1;
struct buffer *buffers;
static unsigned int n_buffers;
static int force_format = 1;

static double fnow = 0.0, fstart = 0.0, fstop = 0.0;
static struct timespec time_now, time_start, time_stop;

static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
    int rc;

    do
    {
        rc = ioctl(fh, request, arg);

    } while (-1 == rc && EINTR == errno);

    return rc;
}

char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n" HRES_STR " " VRES_STR "\n255\n";
char ppm_dumpname[] = "frames/Time-0000.ppm";

// forward declarations for functions used in the main loop that are defined after it
unsigned int frame_diff_yuyv(const unsigned char *prev,
                             const unsigned char *curr,
                             int width, int height);
double frame_percent_diff(unsigned int diffsum, int width, int height, bool is_rgb);
unsigned int frame_diff_rgb(const unsigned char *prev,
                            const unsigned char *curr,
                            int width, int height);

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;

    snprintf(&ppm_dumpname[11], 9, "%04d", tag);
    strncat(&ppm_dumpname[15], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    strncat(&ppm_header[29], " msec \n" HRES_STR " " VRES_STR "\n255\n", 19);

    // subtract 1 from sizeof header because it includes the null terminator for the string
    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);

    total = 0;

    do
    {
        written = write(dumpfd, p, size);
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    // syslog(LOG_CRIT, "Frame written to flash at %lf, %d, bytes\n", (fnow - fstart), total);

    close(dumpfd);
}

char pgm_header[] = "P5\n#9999999999 sec 9999999999 msec \n" HRES_STR " " VRES_STR "\n255\n";
char pgm_dumpname[] = "frames/Time-0000.pgm";

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;

    snprintf(&pgm_dumpname[11], 9, "%04d", tag);
    strncat(&pgm_dumpname[15], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    strncat(&pgm_header[29], " msec \n" HRES_STR " " VRES_STR "\n255\n", 19);

    // subtract 1 from sizeof header because it includes the null terminator for the string
    written = write(dumpfd, pgm_header, sizeof(pgm_header) - 1);

    total = 0;

    do
    {
        written = write(dumpfd, p, size);
        total += written;
    } while (total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    // syslog(LOG_CRIT, "Frame written to flash at %lf, %d, bytes\n", (fnow - fstart), total);

    close(dumpfd);
}

void yuv2rgb_float(float y, float u, float v,
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164 * (y - 16.0) + 1.1596 * (v - 128.0);
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164 * (y - 16.0) - 0.813 * (v - 128.0) - 0.391 * (u - 128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164 * (y - 16.0) + 2.018 * (u - 128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}

// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
    int r1, g1, b1;

    // replaces floating point coefficients
    int c = y - 16, d = u - 128, e = v - 128;

    // Conversion that avoids floating point
    r1 = (298 * c + 409 * e + 128) >> 8;
    g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b1 = (298 * c + 516 * d + 128) >> 8;

    // Computed values may need clipping.
    if (r1 > 255)
        r1 = 255;
    if (g1 > 255)
        g1 = 255;
    if (b1 > 255)
        b1 = 255;

    if (r1 < 0)
        r1 = 0;
    if (g1 < 0)
        g1 = 0;
    if (b1 < 0)
        b1 = 0;

    *r = r1;
    *g = g1;
    *b = b1;
}

// always ignore STARTUP_FRAMES while camera adjusts to lighting, focuses, etc.
int read_framecnt = -STARTUP_FRAMES;
int process_framecnt = 0;
int save_framecnt = 0;
int filter_framecnt = 0;

static int save_image(const void *p, int size, struct timespec *frame_time)
{
    int i, newi, newsize = 0;
    unsigned char *frame_ptr = (unsigned char *)p;

    save_framecnt++;
    // syslog(LOG_CRIT,"save frame %d: ", save_framecnt);

#ifdef DUMP_FRAMES

    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        // syslog(LOG_CRIT, "Dump graymap as-is size %d", size);
        dump_pgm(frame_ptr, size, save_framecnt, frame_time);
    }

    else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

#if defined(COLOR_CONVERT_RGB)

        if (save_framecnt > 0)
        {
            dump_ppm(frame_ptr, ((size * 6) / 4), save_framecnt, frame_time);
            // syslog(LOG_CRIT, "Dump YUYV converted to RGB size %d", size);
        }
#elif defined(COLOR_CONVERT_GRAY)
        if (save_framecnt > 0)
        {
            dump_pgm(frame_ptr, (size / 2), process_framecnt, frame_time);
            printf("Dump YUYV converted to YY size %d\n", size);
        }
#endif
    }

    else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        // printf("Dump RGB as-is size %d\n", size);
        dump_ppm(frame_ptr, size, process_framecnt, frame_time);
    }
    else
    {
        printf("ERROR - unknown dump format\n");
    }
#endif

    return save_framecnt;
}

static int process_image(const void *p, int size)
{
    int i, newi, newsize = 0;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *frame_ptr = (unsigned char *)p;

    process_framecnt++;
    // syslog(LOG_CRIT,"process frame %d: \n", process_framecnt);

    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("NO PROCESSING for graymap as-is size %d\n", size);
    }

    else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {
#if defined(COLOR_CONVERT_RGB)

        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for (i = 0, newi = 0; i < size; i = i + 4, newi = newi + 6)
        {
            y_temp = (int)frame_ptr[i];
            u_temp = (int)frame_ptr[i + 1];
            y2_temp = (int)frame_ptr[i + 2];
            v_temp = (int)frame_ptr[i + 3];
            if (is_scratchpad_buffer_in_use)
            {
                // printf("Scratchpad buffer is currently in use.\n");
                yuv2rgb(y_temp, u_temp, v_temp, &scratchpad_buffer[newi], &scratchpad_buffer[newi + 1], &scratchpad_buffer[newi + 2]);
                yuv2rgb(y2_temp, u_temp, v_temp, &scratchpad_buffer[newi + 3], &scratchpad_buffer[newi + 4], &scratchpad_buffer[newi + 5]);
            }
            else
            {
                yuv2rgb(y_temp, u_temp, v_temp, ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame + newi, &ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame[newi + 1], &ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame[newi + 2]);
                yuv2rgb(y2_temp, u_temp, v_temp, &ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame[newi + 3], &ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame[newi + 4], &ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame[newi + 5]);
            }
        }
        if (!is_scratchpad_buffer_in_use)
        {
            if (ring_output_buffer.count < (int)ring_output_buffer.ring_size)
            {
                ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].is_ready_to_save = true;
                ring_output_buffer.tail_idx = (ring_output_buffer.tail_idx + 1) % ring_output_buffer.ring_size;
                ring_output_buffer.count++;
            }
            else
            {
                syslog(LOG_CRIT, "output ring buffer overflow, frame dropped");
            }
        }

#elif defined(COLOR_CONVERT_GRAY)
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want Y, so YY which is 2 bytes
        //
        for (i = 0, newi = 0; i < size; i = i + 4, newi = newi + 2)
        {
            // Y1=first byte and Y2=third byte
            if (is_scratchpad_buffer_in_use)
            {
                // printf("Scratchpad buffer is currently in use.\n");
                scratchpad_buffer[newi] = frame_ptr[i];
                scratchpad_buffer[newi + 1] = frame_ptr[i + 2];
            }
            else
            {
                ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame[newi] = frame_ptr[i];
                ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].frame[newi + 1] = frame_ptr[i + 2];
            }
        }
        if (!is_scratchpad_buffer_in_use)
        {
            if (ring_output_buffer.count < (int)ring_output_buffer.ring_size)
            {
                ring_output_buffer.save_out_frame[ring_output_buffer.tail_idx].is_ready_to_save = true;
                ring_output_buffer.tail_idx = (ring_output_buffer.tail_idx + 1) % ring_output_buffer.ring_size;
                ring_output_buffer.count++;
            }
            else
            {
                syslog(LOG_CRIT, "output ring buffer overflow, frame dropped");
            }
        }
#endif
    }

    else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("NO PROCESSING for RGB as-is size %d\n", size);
    }
    else
    {
        printf("NO PROCESSING ERROR - unknown format\n");
    }

    return process_framecnt;
}

static int read_frame(void)
{
    CLEAR(frame_buf);

    frame_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frame_buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(camera_device_fd, VIDIOC_DQBUF, &frame_buf))
    {
        switch (errno)
        {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, but drivers should only set for serious errors, although some set for
               non-fatal errors too.
             */
            return 0;

        default:
            printf("mmap failure\n");
            errno_exit("VIDIOC_DQBUF");
        }
    }

    read_framecnt++;

    // printf("frame %d ", read_framecnt);

    if (read_framecnt == 0)
    {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

    assert(frame_buf.index < n_buffers);

    return 1;
}

// 3x3 Laplacian sharpening kernel applied to RGB24 frame data in-place.
//
// Kernel:  [ 0 -1  0 ]
//          [-1  5 -1 ]    output = 5*center - top - bottom - left - right
//          [ 0 -1  0 ]
//
// Deadline safety margin is very large here because:
//   - No dynamic allocation: uses global scratchpad_buffer as read-only copy.
//   - No system calls: pure integer arithmetic.
//   - Bounded WCET: 638x478 interior pixels x 3 channels x ~12 ops
//     = ~11M ops => ~3-6 ms on RPi4 @ 1.5 GHz. Safety margin > 150x (15x for 10Hz).
//   - scratchpad_buffer is free here because is_scratchpad_buffer_in_use == false
//     in the threaded path (apply_filter is only called from seq_frame_filter).
static int apply_filter(void *p, int size)
{
    unsigned char *frame = (unsigned char *)p;
    const int width = HRES;
    const int height = VRES;
    const int channels = 3; // RGB24: 3 bytes per pixel
    const int stride = width * channels;
    const int data_size = height * stride;

    // Copy original frame so we read unmodified neighbors while writing output.
    memcpy(scratchpad_buffer, frame, data_size);

    // Skip border rows/columns — output pixels at edge keep their original value.
    for (int r = 1; r < height - 1; r++)
    {
        for (int c = 1; c < width - 1; c++)
        {
            for (int k = 0; k < channels; k++)
            {
                int center = scratchpad_buffer[r * stride + c * channels + k];
                int top = scratchpad_buffer[(r - 1) * stride + c * channels + k];
                int bottom = scratchpad_buffer[(r + 1) * stride + c * channels + k];
                int left = scratchpad_buffer[r * stride + (c - 1) * channels + k];
                int right = scratchpad_buffer[r * stride + (c + 1) * channels + k];

                int val = 5 * center - top - bottom - left - right;

                if (val < 0)
                    val = 0;
                else if (val > 255)
                    val = 255;

                frame[r * stride + c * channels + k] = (unsigned char)val;
            }
        }
    }

    return data_size;
}

// Re-enable continuous autofocus, restoring camera to its default state before exit.
static void enable_autofocus(void)
{
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_FOCUS_AUTO;
    ctrl.value = 1;
    if (xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl) == -1)
        perror("enable_autofocus: V4L2_CID_FOCUS_AUTO");
    else
        syslog(LOG_CRIT, "Autofocus re-enabled on shutdown\n");
}

// Disable continuous autofocus via V4L2 control.
// Called once after the camera has had time to settle focus.
static void disable_autofocus(void)
{
    struct v4l2_control ctrl;

    // Step 1: disable continuous autofocus (focus_automatic_continuous = V4L2_CID_FOCUS_AUTO)
    ctrl.id = V4L2_CID_FOCUS_AUTO;
    ctrl.value = 0;
    if (xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl) == -1)
    {
        perror("disable_autofocus: V4L2_CID_FOCUS_AUTO");
        return;
    }

    // Step 2: read back the focus position the lens settled at
    ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl.value = 0;
    if (xioctl(camera_device_fd, VIDIOC_G_CTRL, &ctrl) == -1)
    {
        perror("disable_autofocus: VIDIOC_G_CTRL V4L2_CID_FOCUS_ABSOLUTE");
        return;
    }

    int settled_focus = ctrl.value;

    // Step 3: write it back to lock the motor at that position
    if (xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl) == -1)
        perror("disable_autofocus: VIDIOC_S_CTRL V4L2_CID_FOCUS_ABSOLUTE");
    else
        syslog(LOG_CRIT, "Autofocus disabled at %d of read frame counter, focus locked at %d\n",
               read_framecnt, settled_focus);
}

// Re-enable auto exposure, restoring camera to its default state before exit.
static void enable_auto_exposure(void)
{
    struct v4l2_control ctrl;

    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_APERTURE_PRIORITY; /* typical webcam auto-exposure default */
    if (xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl) == -1)
        perror("enable_auto_exposure: V4L2_CID_EXPOSURE_AUTO");
    else
        syslog(LOG_CRIT, "Auto exposure re-enabled on shutdown\n");

    /* restore auto gain if the camera supports it; ignore EINVAL */
    ctrl.id = V4L2_CID_AUTOGAIN;
    ctrl.value = 1;
    xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl);
}

// Disable auto exposure and auto gain after the camera has settled.
// Step 1: read settled exposure; step 2: switch to manual; step 3: lock it.
static void disable_auto_exposure(void)
{
    struct v4l2_control ctrl;

    /* Step 1: read the exposure the camera settled at under auto control */
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    if (xioctl(camera_device_fd, VIDIOC_G_CTRL, &ctrl) == -1)
    {
        perror("disable_auto_exposure: VIDIOC_G_CTRL V4L2_CID_EXPOSURE_ABSOLUTE");
        return;
    }
    int settled_exposure = ctrl.value;

    /* Step 2: switch to manual exposure mode */
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    if (xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl) == -1)
    {
        perror("disable_auto_exposure: VIDIOC_S_CTRL V4L2_CID_EXPOSURE_AUTO");
        return;
    }

    /* Step 3: write back the settled value to prevent the driver from drifting */
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = settled_exposure;
    if (xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl) == -1)
        perror("disable_auto_exposure: VIDIOC_S_CTRL V4L2_CID_EXPOSURE_ABSOLUTE");
    else
        syslog(LOG_CRIT, "Auto exposure disabled at read_framecnt=%d, exposure locked at %d (units: 100 us)\n",
               read_framecnt, settled_exposure);

    /* Step 4: disable auto gain (optional — ignore EINVAL if unsupported) */
    ctrl.id = V4L2_CID_AUTOGAIN;
    ctrl.value = 0;
    if (xioctl(camera_device_fd, VIDIOC_S_CTRL, &ctrl) == -1)
        perror("disable_auto_exposure: V4L2_CID_AUTOGAIN (may be unsupported on this camera)");
}

// we carefully adjust head_idx dependently on the pattern of 4 subsequent difference detections
static unsigned int compute_head_idx(const double *slots, int n_slots,
                                     unsigned int count, unsigned int fallback)
{
    static long double thres = 0.06; // threshold
    if (count == 1)
    {
        if (slots[0] > 0)
            return 0;
        if (slots[1] > 0)
            return 1;
    }
    else if (count == 2)
    {
        if (slots[0] > (slots[1] + thres))
        {
                return 0;
        }
        if (slots[1] >  (slots[0] + thres))
                return 1;
    }
    
    return fallback;
}

int seq_frame_read(void)
{
    fd_set fds;
    struct timeval tv;
    int rc;
    int curr_frame_idx;
    double frame_diff_pers;
    unsigned int diffsum;
    FD_ZERO(&fds);
    FD_SET(camera_device_fd, &fds);

    /* Timeout */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    static unsigned int prev_head_idx = UINT_MAX;
    static unsigned int pending_capture_head_idx = UINT_MAX;
    static unsigned int pending_capture_count = 0;
    int is_read_unsafe = INT_MIN;
    rc = select(camera_device_fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == rc)
    {
        if (!(EINTR == errno))
            errno_exit("select");
    }

    if (0 == rc)
    {
        fprintf(stderr, "select timeout\n");
        exit(EXIT_FAILURE);
    }

    struct timespec read_ts_start;
    double read_start;
    clock_gettime(CLOCK_MONOTONIC, &read_ts_start);
    read_start = (double)read_ts_start.tv_sec + (double)read_ts_start.tv_nsec / 1000000000.0;

    read_frame();

    // save off copy of image with time-stamp here
    // printf("memcpy to %p from %p for %d bytes\n", (void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);
    // syslog(LOG_CRIT, "memcpy to %p from %p for %d bytes\n", (void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);
    memcpy((void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);
    curr_frame_idx = ring_buffer.tail_idx;
    ring_buffer.tail_idx = (ring_buffer.tail_idx + 1) % ring_buffer.ring_size;
    ring_buffer.count++;

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;

    if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &frame_buf))
        errno_exit("VIDIOC_QBUF");

    if (read_framecnt == ((AUTOFOCUS_OFF_FRAMES) - (STARTUP_FRAMES)))
    {
        static int autofocus_disabled = 0;
        if (!autofocus_disabled)
        {
            disable_autofocus();
            // disable_auto_exposure();  // may be helfull at certain environment
            autofocus_disabled = 1;
        }
    }
    // we try to determine correct startup head_idx two times to launch frame processing with the appropriate head_idx
    // it is attempt to avoid blur of the second hand on the saved frames
    if ((read_framecnt > -5 && read_framecnt < -2) && curr_frame_idx > 0)
    {

        diffsum = frame_diff_yuyv((void *)&(ring_buffer.save_frame[curr_frame_idx].frame[0]), (void *)&(ring_buffer.save_frame[curr_frame_idx - 1].frame[0]), HRES, VRES);
        frame_diff_pers = frame_percent_diff(diffsum, HRES, VRES, false);
        if (frame_diff_pers > DIFF_MIN && frame_diff_pers < DIFF_MAX) // values from experiment
        {
            int slot = read_framecnt + 4; // -4→0, -3→1
            diff_frame[slot] = frame_diff_pers;
            diff_counter++;
        }
        if (read_framecnt == -3)  // we are at end of the range
        {
            syslog(LOG_CRIT, "the first startup: diff_counter=%d, slots=[%lf,%lf]",
                   diff_counter, diff_frame[0], diff_frame[1]);
            first_startup_head_idx = compute_head_idx(diff_frame,
                                                      sizeof(diff_frame) / sizeof(diff_frame[0]),
                                                      diff_counter, UINT_MAX /*default*/);
            syslog(LOG_CRIT, "The first startup_head_idx: %u, curr_frame_idx= %d", first_startup_head_idx, curr_frame_idx);
        }
    }

    if (read_framecnt == -2) // gap between phase 1 (ends at -3) and phase 2 (starts at -1)
    {
        diff_counter = 0;
        memset(diff_frame, 0, sizeof(diff_frame));
    }

    if ((read_framecnt > -2 && read_framecnt < 1) && curr_frame_idx > 0)
    {

        diffsum = frame_diff_yuyv((void *)&(ring_buffer.save_frame[curr_frame_idx].frame[0]), (void *)&(ring_buffer.save_frame[curr_frame_idx - 1].frame[0]), HRES, VRES);
        frame_diff_pers = frame_percent_diff(diffsum, HRES, VRES, false);
        if (frame_diff_pers > DIFF_MIN && frame_diff_pers < DIFF_MAX) // values from experiment
        {
            int slot = read_framecnt + 1; // -1→0, 0→1
            diff_frame[slot] = frame_diff_pers;
            diff_counter++;
        }
        if (read_framecnt == 0)
        {
            syslog(LOG_CRIT, "The second startup: diff_counter=%d, slots=[%lf,%lf]",
                   diff_counter, diff_frame[0], diff_frame[1]);
            second_startup_head_idx = compute_head_idx(diff_frame,
                                                       sizeof(diff_frame) / sizeof(diff_frame[0]),
                                                       diff_counter, UINT_MAX /*default*/);
            syslog(LOG_CRIT, "The second startup_head_idx: %u, curr_frame_idx= %d", second_startup_head_idx, curr_frame_idx);
        }
    }

    sel_startup_head_idx = second_startup_head_idx == UINT_MAX ? (first_startup_head_idx == UINT_MAX ? 0 : first_startup_head_idx) : second_startup_head_idx;

    if (read_framecnt > 0)
    {
        ring_buffer.save_frame[curr_frame_idx].time_stamp = time_now;
        diff_counter = 0;
        memset(diff_frame, 0, sizeof(diff_frame));
        if (read_framecnt == 1)
            syslog(LOG_CRIT, "The selected startup_head_idx: %u, curr_frame_idx= %d", sel_startup_head_idx, curr_frame_idx);

        // printf("Acquisitation: read_framecnt=%d, rb.tail=%d, rb.head=%d, rb.count=%d at %lf and %lf FPS.\n", read_framecnt, ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count, (fnow - fstart), (double)(read_framecnt) / (fnow - fstart));

        // syslog(LOG_CRIT, "read_framecnt=%d, rb.tail=%d, rb.head=%d, rb.count=%d at %lf and %lf FPS", read_framecnt, ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count, (fnow-fstart), (double)(read_framecnt) / (fnow-fstart));
        // syslog(LOG_CRIT, "read_framecnt=%d at %lf and %lf FPS", read_framecnt, (fnow - fstart), (double)(read_framecnt) / (fnow - fstart));
    }
    else
    {
        //syslog(LOG_CRIT, "at %lf", fnow);
    }

    // printf("--Acquisitation read frame at: %lf\n", (fnow - read_start));

    if (read_framecnt > 0)
    {
        // Select frames to save based on whether they are different from the previous frame
        struct timespec sel_ts_start, sel_ts_now;
        double sel_start, sel_now;

        clock_gettime(CLOCK_MONOTONIC, &sel_ts_start);
        sel_start = (double)sel_ts_start.tv_sec + (double)sel_ts_start.tv_nsec / 1000000000.0;
        // we should apply the new head idx with delay to avoid the frames with stucked second hand and jumps of the second hand
        if (capture_head_idx == UINT_MAX)
        {
            ring_buffer.save_frame[sel_startup_head_idx].is_selected_to_save = true;
            ring_buffer.head_idx = (sel_startup_head_idx) % ring_buffer.ring_size;
            prev_head_idx = sel_startup_head_idx;
        }
        else
        {
            if (prev_head_idx == capture_head_idx)
            {
                ring_buffer.save_frame[capture_head_idx].is_selected_to_save = true;
                ring_buffer.head_idx = (capture_head_idx) % ring_buffer.ring_size;
            }
            else
            {
                if (header_idx_delay_counter > 4)
                {
                    prev_head_idx = capture_head_idx;
                    header_idx_delay_counter = 0;
                }
                header_idx_delay_counter++;
            }
        }

        // curr_frame_idx should be more than 1 here because we skip the first few frames
        if (curr_frame_idx > 0)
        {
            // too scared enironment, abort
            if ((second_startup_head_idx == UINT_MAX) && (first_startup_head_idx == UINT_MAX) && (read_framecnt == 30))
            {
                syslog(LOG_CRIT, "Unsafe frame detection. Frame: %d", read_framecnt);
                is_read_unsafe++;
            }

            diffsum = frame_diff_yuyv((void *)&(ring_buffer.save_frame[curr_frame_idx].frame[0]), (void *)&(ring_buffer.save_frame[curr_frame_idx - 1].frame[0]), HRES, VRES);
            frame_diff_pers = frame_percent_diff(diffsum, HRES, VRES, false);

            // compare the frame we are processing to the one that is currently at the tail of the ring buffer, which should be the most recently acquired frame, and if they are not the same,
            // then we know that we mark the frame
            if (frame_diff_pers > DIFF_MIN && frame_diff_pers < DIFF_MAX) // values from experiment
            {
                ring_buffer.save_frame[curr_frame_idx].is_different_from_previous = true;
                // **************************************************************************************
                // syslog(LOG_CRIT, "Frame at tail of ring buffer %d is not the same as previous frame, marked.\n", curr_frame_idx);
                // **************************************************************************************
                //syslog(LOG_CRIT, "Difference in frame %d : %lf detectecd, read counter: %d", curr_frame_idx, frame_diff_pers, read_framecnt);
                // syslog(LOG_CRIT, "BEFORE Frame: %d, diff_counter2= %d, diff_frame2: [%lf, %lf, %lf, %lf]", read_framecnt, diff_counter2, diff_frame2[0], diff_frame2[1], diff_frame2[2], diff_frame2[3]);

                int slot = curr_frame_idx - 1; // 1→0, 2→1, 3→2
                diff_frame2[slot] = frame_diff_pers;
                diff_counter2++;
            }
            else
            {
                ring_buffer.save_frame[curr_frame_idx].is_different_from_previous = false;
                // **************************************************************************************************************
                // syslog(LOG_CRIT, "Frame at tail of ring buffer %d is the same as previous frame, not marked.\n", curr_frame_idx);
                // **************************************************************************************************************
            }
            if (curr_frame_idx == ring_buffer.ring_size - 1) // we are at the end of period
            {
                //syslog(LOG_CRIT, "capture: diff_counter2= %d, slots=[ %lf,%lf,%lf,%lf ]",
                //       diff_counter2, diff_frame2[0], diff_frame2[1], diff_frame2[2], diff_frame2[3]);
                unsigned int computed_idx = compute_head_idx(diff_frame2,
                                                             sizeof(diff_frame2) / sizeof(diff_frame2[0]),
                                                             diff_counter2, capture_head_idx == UINT_MAX ? sel_startup_head_idx : capture_head_idx /*fallback*/);
                if (computed_idx == pending_capture_head_idx)
                {
                    pending_capture_count++;
                    if (pending_capture_count >= CAPTURE_HEAD_STABILITY_PERIODS && capture_head_idx != computed_idx)
                    {
                        capture_head_idx = computed_idx;
                        //syslog(LOG_CRIT, "capture_head_idx stabilized at %u after %u periods, read counter= %d", capture_head_idx, pending_capture_count, read_framecnt);
                    }
                }
                else
                {
                    pending_capture_head_idx = computed_idx;
                    pending_capture_count = 1;
                }
                //syslog(LOG_CRIT, "capture_head_idx: %u (pending=%u count=%u), read counter= %d, curr_frame_idx=%d", capture_head_idx, pending_capture_head_idx, pending_capture_count, read_framecnt, curr_frame_idx);
                diff_counter2 = 0;
                memset(diff_frame2, 0, sizeof(diff_frame2));
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &sel_ts_now);
        sel_now = (double)sel_ts_now.tv_sec + (double)sel_ts_now.tv_nsec / 1000000000.0;

        // syslog(LOG_CRIT, " Selection: read_framecnt=%d, rb.tail=%d, rb.head=%d, rb.count=%d at %lf and %lf FPS.", read_framecnt, ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count, (sel_now - sel_start), (double)(read_framecnt) / (sel_now - fstart));
    }

    return (is_read_unsafe > INT_MIN) ? INT_MIN : read_framecnt;
}

int seq_frame_process(void)
{
    int cnt = 0;
    struct timespec proc_ts_start, proc_ts_now;
    double proc_start, proc_now;

    // syslog(LOG_CRIT, "processing rb.tail=%d, rb.head=%d, rb.count=%d\n", ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count);

    // ring_buffer.head_idx = (ring_buffer.head_idx + 2) % ring_buffer.ring_size;  // reference logic from startup code

    if (read_framecnt > 0)
    {
        clock_gettime(CLOCK_MONOTONIC, &proc_ts_start);
        proc_start = (double)proc_ts_start.tv_sec + (double)proc_ts_start.tv_nsec / 1000000000.0;
        cnt = process_image((void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]), HRES * VRES * PIXEL_SIZE);
        // ************************************************************************************************************************
        // syslog(LOG_CRIT, "Processed frame %d from ring buffer at head index %d, whith selected to save = %d\n", process_framecnt, ring_buffer.head_idx, ring_buffer.save_frame[ring_buffer.head_idx].is_selected_to_save);
        ring_buffer.save_frame[ring_buffer.head_idx].is_selected_to_save = false;
        // ************************************************************************************************************************
    }
    else
    {
        printf("\nNo processing because no frames have been read yet.\n");
    }

    // syslog(LOG_CRIT, "rb.tail=%d, rb.head=%d, rb.count=%d ", ring_buffer.tail_idx, ring_buffer.head_idx, ring_buffer.count);

    if (process_framecnt > 0)
    {
        clock_gettime(CLOCK_MONOTONIC, &time_now);
        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
        // syslog(LOG_CRIT, " processed at %lf, @ %lf FPS\n", (fnow - fstart), (double)(process_framecnt) / (fnow - fstart));
        // syslog(LOG_CRIT, "---Processing time for this frame: %lf\n", (fnow - proc_start));
    }
    else
    {
        //syslog(LOG_CRIT,"at %lf", fnow - fstart);
    }

    return cnt;
}

int seq_frame_store(void)
{
    int cnt, cnt2 = 0;
    struct timespec store_ts_start, store_ts_now;
    double store_start, store_now;

    if (read_framecnt > 0)
    {
        clock_gettime(CLOCK_MONOTONIC, &store_ts_start);
        store_start = (double)store_ts_start.tv_sec + (double)store_ts_start.tv_nsec / 1000000000.0;
        // cnt = save_image(scratchpad_buffer, HRES * VRES * PIXEL_SIZE, &time_now);

        if ((ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].is_ready_to_save) && (ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].is_filter_applied))
        {

            cnt2 = save_image((void *)&(ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].frame[0]), HRES * VRES * PIXEL_SIZE, &time_now);

            ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].is_ready_to_save = false;
            ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].is_filter_applied = false;

            ring_output_buffer.head_idx = (ring_output_buffer.head_idx + 1u) % ring_output_buffer.ring_size;
            ring_output_buffer.count--;
            // printf("save_framecnt=%d ", save_framecnt);

            clock_gettime(CLOCK_MONOTONIC, &time_now);
            fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
            // ------------------------------------------------------------------------------------------
            syslog(LOG_CRIT, COURSE_FRM_CAPT_SYSLOG(COURSE, ASS), save_framecnt, (fnow - fstart));
            // ------------------------------------------------------------------------------------------
            // syslog(LOG_CRIT, "save_framecnt=%d at %lf and %lf FPS", save_framecnt, (fnow - fstart), (double)(save_framecnt) / (fnow - fstart));
            // printf(" saved at %lf, @ %lf FPS\n", (fnow - fstart), (double)(save_framecnt) / (fnow - fstart));
            // printf("---Saving time for this frame: %lf\n", (fnow - store_start));
        }
        else
        {
            // ********************************************************************************************************
            // syslog(LOG_CRIT, "The frame %u from output ring buffer rejected to save.", ring_output_buffer.head_idx);
            // ********************************************************************************************************
        }
    }
    else
    {
        printf("\nNo saving because no frames have been read yet.\n");
        printf("at %lf\n", fnow - fstart);
    }

    return cnt2;
}

int seq_frame_filter(void)
{
    // this is where we would implement any additional filtering or processing on the processed frames that are in the output ring buffer, but for simplicity, we will just save the processed frames to disk without any additional filtering or processing, so this function is just a placeholder for where that code would go if we wanted to implement it

    if ((ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].is_ready_to_save) && (!ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].is_filter_applied))
    {
        // process the frame in the output ring buffer, which should already be in RGB format, so we can just save it to disk without any additional processing, but if we wanted to implement any additional filtering or processing on the processed frames, we would do it here before saving the frame to disk
        // save_image((void *)&(ring_output_buffer.save_out_frame[i].frame[0]), HRES * VRES * PIXEL_SIZE, &time_now);

        apply_filter((void *)&(ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].frame[0]), HRES * VRES * PIXEL_SIZE * 3);
        ring_output_buffer.save_out_frame[ring_output_buffer.head_idx].is_filter_applied = true;

        // syslog(LOG_CRIT, "Saved %d frame from output ring buffer processing by filter.", save_framecnt);
        filter_framecnt++;
        clock_gettime(CLOCK_MONOTONIC, &time_now);
        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
        // syslog(LOG_CRIT, " filtered at %lf, @ %lf FPS", (fnow - fstart), (double)(filter_framecnt) / (fnow - fstart));
    }

    return filter_framecnt;
}

// This is the main loop for the sequential version of the program, which reads frames from the camera, processes them, and saves them to disk in a sequential manner.
// The main loop will continue to run until the specified number of frames have been acquired, processed, and saved.
// The main loop also includes a delay between each frame acquisition to control the frame rate of the program.
static void mainloop(void)
{
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;

    // default is false because we want to use the ring buffer for processing and saving frames for the threading case. Call of mainloop is only used for the sequential case
    is_scratchpad_buffer_in_use = true;

    // Replace this with a delay designed for your rate
    // of frame acquitision and storage.
    //
#if (FRAMES_PER_SEC == 1)
    printf("Running at 1 frame/sec\n");
    read_delay.tv_sec = 0;
    read_delay.tv_nsec = 920000000;
#elif (FRAMES_PER_SEC == 10)
    printf("Running at 10 frames/sec\n");
    read_delay.tv_sec = 0;
    read_delay.tv_nsec = 100000000;
#elif (FRAMES_PER_SEC == 20)
    printf("Running at 20 frames/sec\n");
    read_delay.tv_sec = 0;
    read_delay.tv_nsec = 50000000;
#elif (FRAMES_PER_SEC == 25)
    printf("Running at 25 frames/sec\n");
    read_delay.tv_sec = 0;
    read_delay.tv_nsec = 40000000;
#elif (FRAMES_PER_SEC == 30)
    printf("Running at 30 frames/sec\n");
    read_delay.tv_sec = 0;
    read_delay.tv_nsec = 33333333;
#else
    printf("Running at 1 frame/sec\n");
    read_delay.tv_sec = 1;
    read_delay.tv_nsec = 0;
#endif

    count = FRAMES_TO_ACQUIRE;

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int rc;

            FD_ZERO(&fds);
            FD_SET(camera_device_fd, &fds);

            /* Timeout */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            rc = select(camera_device_fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == rc)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == rc)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame())
            {
                if (nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                else
                {
                    clock_gettime(CLOCK_MONOTONIC, &time_now);
                    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;

                    if (read_framecnt > 1)
                    {
                        printf(" read at %lf, @ %lf FPS\n", (fnow - fstart), (double)(read_framecnt) / (fnow - fstart));

                        memcpy((void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]), buffers[frame_buf.index].start, frame_buf.bytesused);
                        // syslog(LOG_CRIT, "memcpy to rb.tail=%d, rb.head=%d, ptr=%p", ring_buffer.tail_idx, ring_buffer.head_idx, (void *)&(ring_buffer.save_frame[ring_buffer.tail_idx].frame[0]));

                        // advance ring buffer for next read
                        ring_buffer.tail_idx = (ring_buffer.tail_idx + 1) % ring_buffer.ring_size;
                        ring_buffer.count++;

                        process_image((void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]), HRES * VRES * PIXEL_SIZE);
                        // process_image(buffers[frame_buf.index].start, frame_buf.bytesused);
                        printf("bytesused=%d, hxvxp=%d\n", frame_buf.bytesused, HRES * VRES * PIXEL_SIZE);

                        printf("process from rb.tail=%d, rb.head=%d, ptr=%p\n", ring_buffer.tail_idx, ring_buffer.head_idx, (void *)&(ring_buffer.save_frame[ring_buffer.head_idx].frame[0]));
                        save_image(scratchpad_buffer, HRES * VRES * PIXEL_SIZE, &time_now);

                        // advance ring buffer for next write
                        ring_buffer.head_idx = (ring_buffer.head_idx + 1) % ring_buffer.ring_size;
                        ring_buffer.count--;
                    }
                    else
                    {
                        printf("at %lf\n", (fnow - fstart));
                    }
                }

                if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &frame_buf))
                    errno_exit("VIDIOC_QBUF");
                count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if (count <= 0)
                break;
        }

        if (count <= 0)
            break;
    }
}

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    clock_gettime(CLOCK_MONOTONIC, &time_stop);
    fstop = (double)time_stop.tv_sec + (double)time_stop.tv_nsec / 1000000000.0;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(camera_device_fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");

    printf("capture stopped\n");
}

static void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    printf("will capture to %d buffers\n", n_buffers);

    for (i = 0; i < n_buffers; ++i)
    {
        printf("allocated buffer %d\n", i);

        CLEAR(frame_buf);
        frame_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        frame_buf.memory = V4L2_MEMORY_MMAP;
        frame_buf.index = i;

        if (-1 == xioctl(camera_device_fd, VIDIOC_QBUF, &frame_buf))
            errno_exit("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(camera_device_fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
}

static void uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].start, buffers[i].length))
            errno_exit("munmap");

    free(buffers);
}

static void init_output_buf(void)
{
    // for output buffer, we will just use a single buffer that we will write to and then save to disk, so we don't need to worry about mmap or anything like that, we can just use malloc to allocate a buffer for the processed frame that we will save to disk
    // if we wanted to be more sophisticated, we could set up a ring buffer for the processed frames as well, but for simplicity, we will just use a single buffer for the processed frame that we will save to disk
    // this is because the processing and saving of frames is much slower than the acquisition of frames, so we don't need to worry about the processed frames being overwritten by the acquisition loop while we are processing or saving a frame
    // if we wanted to be more sophisticated, we could set up a separate thread for processing and saving frames, and then use a mutex to protect access to the processed frame buffer, but for simplicity, we will just use a single buffer for the processed frame that we will save to disk

    printf("Initializing output buffer for processed frames\n");
    ring_output_buffer.head_idx = 0;
    ring_output_buffer.tail_idx = 0;
    ring_output_buffer.count = 0;
    ring_output_buffer.ring_size = RING_OUTPUT_BUFFER_SIZE;
    for (int i = 0; i < ring_output_buffer.ring_size; i++)
    {
        ring_output_buffer.save_out_frame[i].is_filter_applied = false;
        ring_output_buffer.save_out_frame[i].is_ready_to_save = false;
        memset(&ring_output_buffer.save_out_frame[i].time_stamp, 0, sizeof(ring_output_buffer.save_out_frame[i].time_stamp));
        memset(ring_output_buffer.save_out_frame[i].identifier_str, 0, sizeof(ring_output_buffer.save_out_frame[i].identifier_str));
    }
}

static void init_mmap(char *dev_name)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = DRIVER_MMAP_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    printf("init_mmap req.count=%d\n", req.count);

    ring_buffer.tail_idx = 0;
    ring_buffer.head_idx = 0;
    ring_buffer.count = 0;
    ring_buffer.ring_size = FRAMES_PER_ACQ_PERIOD;
    for (int i = 0; i < ring_buffer.ring_size; i++)
    {
        ring_buffer.save_frame[i].is_different_from_previous = false;
        ring_buffer.save_frame[i].is_selected_to_save = false;
        memset(&ring_buffer.save_frame[i].time_stamp, 0, sizeof(ring_buffer.save_frame[i].time_stamp));
        memset(ring_buffer.save_frame[i].identifier_str, 0, sizeof(ring_buffer.save_frame[i].identifier_str));
    }

    if (-1 == xioctl(camera_device_fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                            "memory mapping\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2)
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("Device supports %d mmap buffers\n", req.count);

        // allocate tracking buffers array for those that are mapped
        buffers = calloc(req.count, sizeof(*buffers));

        // set up double buffer for frames to be safe with one time malloc her or just declare
    }

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        CLEAR(frame_buf);

        frame_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        frame_buf.memory = V4L2_MEMORY_MMAP;
        frame_buf.index = n_buffers;

        if (-1 == xioctl(camera_device_fd, VIDIOC_QUERYBUF, &frame_buf))
            errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = frame_buf.length;
        buffers[n_buffers].start =
            mmap(NULL /* start anywhere */,
                 frame_buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 camera_device_fd, frame_buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            errno_exit("mmap");

        printf("mappped buffer %d\n", n_buffers);
    }
}

static void init_device(char *dev_name)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(camera_device_fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(camera_device_fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(camera_device_fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width = HRES;
        fmt.fmt.pix.height = VRES;

        // Specify the Pixel Coding Formate here

        // This one works for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        // fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(camera_device_fd, VIDIOC_S_FMT, &fmt))
            errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(camera_device_fd, VIDIOC_G_FMT, &fmt))
            errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    init_mmap(dev_name);
}

static void close_device(void)
{
    if (-1 == close(camera_device_fd))
        errno_exit("close");

    camera_device_fd = -1;
}

static void open_device(char *dev_name)
{
    struct stat st;

    if (-1 == stat(dev_name, &st))
    {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    camera_device_fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == camera_device_fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

// Sequencial case. Note that it is not called from seqv4l2, where threading is used.
int v4l2_frame_acquisition_loop(char *dev_name)
{

    // initialization of V4L2
    open_device(dev_name);
    init_device(dev_name);

    start_capturing();

    // service loop frame read
    mainloop();

    // shutdown of frame acquisition service
    stop_capturing();

    printf("Total capture time=%lf, for %d frames, %lf FPS\n", (fstop - fstart), read_framecnt, ((double)read_framecnt / (fstop - fstart)));

    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}

int v4l2_frame_acquisition_initialization(char *dev_name)
{
    init_output_buf();
    // initialization of V4L2
    open_device(dev_name);
    init_device(dev_name);

    start_capturing();
}

int v4l2_frame_acquisition_shutdown(void)
{
    // shutdown of frame acquisition service
    stop_capturing();

    printf("Total capture time=%lf, for %d frames, %lf FPS\n", (fstop - fstart), read_framecnt, ((double)read_framecnt / (fstop - fstart)));

    uninit_device();
    enable_autofocus(); /* restore camera defaults before closing fd */
                        // enable_auto_exposure();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This is a simple frame difference function that sums the absolute value of the difference in Y channel for each pixel
unsigned int frame_diff_yuyv(const unsigned char *prev,
                             const unsigned char *curr,
                             int width, int height)
{
    unsigned int diffsum = 0;
    int i, npixels = width * height;
    for (i = 0; i < npixels; i++)
    {
        int d = (int)curr[i * 2] - (int)prev[i * 2]; /* even bytes = Y channel */
        diffsum += (d < 0) ? -d : d;
    }
    return diffsum;
}

// Frame difference considering Y, U, and V channels in YUYV format.
// YUYV packs pixels as [Y0][U][Y1][V] per 4-byte macropixel (U and V are
// shared between adjacent pixel pairs). Each channel is weighted equally.
unsigned int frame_diff_yuyv_color(const unsigned char *prev,
                                   const unsigned char *curr,
                                   int width, int height)
{
    unsigned int diffsum = 0;
    int m, nmacropixels = (width * height) / 2; /* 1 macropixel = 2 pixels = 4 bytes */
    for (m = 0; m < nmacropixels; m++)
    {
        int base = m * 4;
        int dy0 = (int)curr[base + 0] - (int)prev[base + 0]; /* Y0 */
        int du = (int)curr[base + 1] - (int)prev[base + 1];  /* U  */
        int dy1 = (int)curr[base + 2] - (int)prev[base + 2]; /* Y1 */
        int dv = (int)curr[base + 3] - (int)prev[base + 3];  /* V  */
        diffsum += (dy0 < 0) ? -dy0 : dy0;
        diffsum += (du < 0) ? -du : du;
        diffsum += (dy1 < 0) ? -dy1 : dy1;
        diffsum += (dv < 0) ? -dv : dv;
    }
    return diffsum;
}

// This function converts the raw difference sum to a percentage of the maximum possible difference for the frame.
// use_color=0: Y-only mode (max = width*height*255);
// use_color=1: Y+UV mode  (max = width*height*255*2, matching frame_diff_yuyv_color)
double frame_percent_diff(unsigned int diffsum, int width, int height, bool use_color)
{
    unsigned int maxdiff = (unsigned int)width * height * 255 * (use_color ? 2 : 1);
    return ((double)diffsum / (double)maxdiff) * 100.0;
}

/* Convert one YUYV frame (w*h*2 bytes) to packed RGB24 (w*h*3 bytes).
   Reuses the existing integer yuv2rgb() for consistency with process_image(). */
static void yuyv_frame_to_rgb(const unsigned char *yuyv,
                              unsigned char *rgb, int w, int h)
{
    int i, ni;
    int y0, u, y1, v;
    for (i = 0, ni = 0; i < w * h * 2; i += 4, ni += 6)
    {
        y0 = (int)yuyv[i];
        u = (int)yuyv[i + 1];
        y1 = (int)yuyv[i + 2];
        v = (int)yuyv[i + 3];
        yuv2rgb(y0, u, v, &rgb[ni], &rgb[ni + 1], &rgb[ni + 2]);
        yuv2rgb(y1, u, v, &rgb[ni + 3], &rgb[ni + 4], &rgb[ni + 5]);
    }
}

/* Fill curr_rgb and prev_rgb with the two most recently completed ring-buffer
   slots converted to RGB24.  Returns the current tail_idx (>= 0) on success,
   or -1 when fewer than 2 frames have been acquired.
   Called by Service_4_frame_display(); ring_buffer is process-local so no IPC
   is needed. Service_1 (higher priority) has already advanced tail past both
   slots read here — with ring_size=3 there are 3 write slots of gap before
   the writer could overwrite ci or pi. */
int seq_frame_get_for_display(unsigned char *curr_rgb,
                              unsigned char *prev_rgb,
                              int *diff_amplify,
                              int *is_motion,
                              int *is_saved,
                              int *frame_num,
                              double *diff_pct)
{
    int t, ci, pi;
    unsigned int diffsum;

    (void)diff_amplify; /* caller owns this value; parameter reserved for future use */

    t = ring_buffer.count;
    if (t < 3)
        return -1;

    ci = (t - 1 + (int)ring_buffer.ring_size) % (int)ring_buffer.ring_size;
    pi = (t - 2 + (int)ring_buffer.ring_size) % (int)ring_buffer.ring_size;

    yuyv_frame_to_rgb(ring_buffer.save_frame[ci].frame, curr_rgb, HRES, VRES);
    yuyv_frame_to_rgb(ring_buffer.save_frame[pi].frame, prev_rgb, HRES, VRES);

    diffsum = frame_diff_yuyv(ring_buffer.save_frame[pi].frame,
                              ring_buffer.save_frame[ci].frame,
                              HRES, VRES);
    *diff_pct = frame_percent_diff(diffsum, HRES, VRES, false);

    *is_motion = (int)ring_buffer.save_frame[ci].is_different_from_previous;
    *is_saved = (int)ring_buffer.save_frame[ci].is_selected_to_save;
    *frame_num = process_framecnt;
    return t;
}
