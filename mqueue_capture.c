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
 *  Refactored by Vladimir Zdravkov to use two POSIX threads communicating via mqueue:
 *    - capture thread (producer): V4L2 dequeue, YUYV->RGB conversion, mq_send
 *    - writer thread (consumer):  mq_receive, dump PPM/PGM to disk, free heap
 *  Pattern follows heap_mq.c from siewertsmooc/RTES-ECEE-5623.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <errno.h>

#include <linux/videodev2.h>

#include <time.h>
#include <pthread.h>
#include <mqueue.h>
#include <sched.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 640//320
#define VRES 480//240
#define HRES_STR "640"
#define VRES_STR "480"


#define NSEC_PER_SEC (1000000000)
#define ERROR (-1)
#define OK (0)


/* ---- mqueue / thread configuration ---- */
#define FRAME_MQ_NAME  "/capture_frames"
#define FRAME_MQ_DEPTH 30                       /* max in-flight frames     */
#define FRAME_MQ_SIZE  sizeof(struct frame_msg) /* tiny: pointer+metadata  */

static struct timespec rtclk_dt = {0, 0};
static struct timespec rtclk_start_time = {0, 0};
static struct timespec rtclk_stop_time = {0, 0};

static struct timespec rtclk_start_write_time = {0, 0};
static struct timespec rtclk_stop_write_time = {0, 0};

/* Message sent through the queue.  Only the pointer travels via mqueue;
   the pixel data stays on the heap — zero extra copy. */
struct frame_msg
{
    unsigned char  *data;       /* heap buffer with converted pixels; NULL = sentinel */
    int             size;       /* byte count of data                                 */
    unsigned int    frame_num;  /* sequence number used for filename                  */
    struct timespec timestamp;  /* CLOCK_REALTIME at capture                          */
    int             is_ppm;     /* 1 = PPM (colour), 0 = PGM (grey)                  */
};

/* ---- globals ---- */

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

enum io_method
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer
{
        void   *start;
        size_t  length;
};

static char            *dev_name;
//static enum io_method   io = IO_METHOD_USERPTR;
//static enum io_method   io = IO_METHOD_READ;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 180;  /* number of frames to capture, then exit.  Set to 0 for infinite. */

/* mqueue and thread handles — set up in main, used by both threads */
static mqd_t            frame_mq = (mqd_t)-1;  // initialized to -1 to indicate not open, so we can check before close in case of early exit due to error
static struct mq_attr   mq_attr;               // mqueue attributes, set in init_mqueue and used in mq_open; made global so it can be accessed in both init_mqueue and main for mq_open, and also to read original msg_max value and restore on exit
static int original_msg_max;                   // to store original mqueue msg_max value read from /proc/sys/fs/mqueue/msg_max, so we can restore it on exit if we changed it, to avoid affecting other programs that use mqueues

static pthread_t        th_capture, th_writer;  // thread handles, set in main and used in main for pthread_join; made global so they can be accessed in main for pthread_join, and also in setup_reader_writer_threads for setting thread attributes
static pthread_attr_t   attr_capture, attr_writer;
static struct sched_param param_capture, param_writer;

unsigned int framecnt = 0;

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}



static int setup_reader_writer_threads(void)
{
    int rt_max_prio, rt_min_prio;
    int rc;
    /* ---- thread priority setup (following heap_mq.c pattern) ----
     * Capture thread: SCHED_FIFO max priority — must not be preempted
     *                 while holding a MMAP buffer.
     * Writer thread:  SCHED_FIFO min priority — disk I/O, can wait.  */
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);
    /* Writer thread: SCHED_FIFO with min priority  */
    rc = pthread_attr_init(&attr_writer);
    rc = pthread_attr_setinheritsched(&attr_writer, PTHREAD_EXPLICIT_SCHED);
    rc = pthread_attr_setschedpolicy(&attr_writer, SCHED_FIFO);
    param_writer.sched_priority = rt_min_prio;
    pthread_attr_setschedparam(&attr_writer, &param_writer);
     /* Capture thread: SCHED_FIFO with MAX priority - 1. */
    rc = pthread_attr_init(&attr_capture);
    rc = pthread_attr_setinheritsched(&attr_capture, PTHREAD_EXPLICIT_SCHED);
    rc = pthread_attr_setschedpolicy(&attr_capture, SCHED_FIFO);
    param_capture.sched_priority = rt_max_prio -1;  /* MAX-1 to ensure writer can run when capture is blocked on MMAP buffer */
    pthread_attr_setschedparam(&attr_capture, &param_capture);
    return rc;
}

static int read_mqueue_msg_max(void)
{
    char buf[16];
    int fd, n;

    fd = open("/proc/sys/fs/mqueue/msg_max", O_RDONLY);
    if (fd == -1) { perror("read msg_max"); return -1; }
    n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    buf[n] = '\0';
    return atoi(buf);
}

static void set_mqueue_msg_max(int value)
{
    char buf[16];
    int fd;

    fd = open("/proc/sys/fs/mqueue/msg_max", O_WRONLY);
    if (fd == -1)
    {
        perror("open /proc/sys/fs/mqueue/msg_max (need root)");
        exit(EXIT_FAILURE);
    }

    snprintf(buf, sizeof(buf), "%d", value);
    if (write(fd, buf, strlen(buf)) == -1)
        perror("write msg_max");

    close(fd);
}

static void init_mqueue(void)
{
    /* Set mqueue msg_max to our desired depth, so that mq_send blocks when queue is full, providing natural backpressure. */
    // set_mqueue_msg_max(FRAME_MQ_DEPTH);

      /* ---- mqueue setup (following heap_mq.c pattern) ---- */
    mq_attr.mq_maxmsg  = FRAME_MQ_DEPTH;
    mq_attr.mq_msgsize = FRAME_MQ_SIZE;
    mq_attr.mq_flags   = 0;

    original_msg_max = read_mqueue_msg_max();
    printf("original mqueue msg_max: %d\n", original_msg_max);
    if (original_msg_max < 0)
    {
        fprintf(stderr, "Failed to read original mqueue msg_max\n");
        exit(EXIT_FAILURE);
    }
    set_mqueue_msg_max(frame_count);   // raise to 30 to ensure queue can hold all frames if writer is too slow, but restore on exit
    if(frame_count > FRAME_MQ_DEPTH)
        fprintf(stderr, "Warning: frame_count %d exceeds FRAME_MQ_DEPTH %d, may cause blocking if writer is too slow\n", frame_count, FRAME_MQ_DEPTH);  
        
    mq_unlink(FRAME_MQ_NAME);   /* remove stale queue from previous crash */
    frame_mq = mq_open(FRAME_MQ_NAME, O_CREAT | O_RDWR, S_IRWXU, &mq_attr);
    if (frame_mq == (mqd_t)-1)
        errno_exit("mq_open");
}


/* ------------------------------------------------------------------ */

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}

/* ---- dump_ppm / dump_pgm ----------------------------------------- */
/* Header built into a local buffer — no global string mutation.       */
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    char header[64];
    char filename[32];
    int written, total, dumpfd;

    snprintf(filename, sizeof(filename), "test%08d.ppm", tag);
    dumpfd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 00666);
    if (dumpfd < 0) { perror("open ppm"); return; }

    snprintf(header, sizeof(header),
             "P6\n#%010d sec %03d msec\n%s %s\n255\n",
             (int)time->tv_sec,
             (int)(time->tv_nsec / 1000000),
             HRES_STR, VRES_STR);

    write(dumpfd, header, strlen(header));

    total = 0;
    do
    {
        written = write(dumpfd, (const char *)p + total, size - total);
        if (written <= 0) break;
        total += written;
    } while (total < size);

    printf("wrote %d bytes to %s\n", total, filename);
    close(dumpfd);
}

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    char header[64];
    char filename[32];
    int written, total, dumpfd;

    snprintf(filename, sizeof(filename), "test%08d.pgm", tag);
    dumpfd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 00666);
    if (dumpfd < 0) { perror("open pgm"); return; }

    snprintf(header, sizeof(header),
             "P5\n#%010d sec %03d msec\n%s %s\n255\n",
             (int)time->tv_sec,
             (int)(time->tv_nsec / 1000000),
             HRES_STR, VRES_STR);

    write(dumpfd, header, strlen(header));

    total = 0;
    do
    {
        written = write(dumpfd, (const char *)p + total, size - total);
        if (written <= 0) break;
        total += written;
    } while (total < size);

    printf("wrote %d bytes to %s\n", total, filename);
    close(dumpfd);
}

/* Calculate the difference between two timespec values */
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  //printf("\ndt calcuation\n");

  // case 1 - less than a second of change
  if(dt_sec == 0)
  {
	  //printf("dt less than 1 second\n");

	  if(dt_nsec >= 0 && dt_nsec < NSEC_PER_SEC)
	  {
	          //printf("nanosec greater at stop than start\n");
		  delta_t->tv_sec = 0;
		  delta_t->tv_nsec = dt_nsec;
	  }

	  else if(dt_nsec > NSEC_PER_SEC)
	  {
	          //printf("nanosec overflow\n");
		  delta_t->tv_sec = 1;
		  delta_t->tv_nsec = dt_nsec-NSEC_PER_SEC;
	  }

	  else // dt_nsec < 0 means stop is earlier than start
	  {
	         printf("stop is earlier than start\n");
		 return(ERROR);  
	  }
  }

  // case 2 - more than a second of change, check for roll-over
  else if(dt_sec > 0)
  {
	  //printf("dt more than 1 second\n");

	  if(dt_nsec >= 0 && dt_nsec < NSEC_PER_SEC)
	  {
	          //printf("nanosec greater at stop than start\n");
		  delta_t->tv_sec = dt_sec;
		  delta_t->tv_nsec = dt_nsec;
	  }

	  else if(dt_nsec > NSEC_PER_SEC)
	  {
	          //printf("nanosec overflow\n");
		  delta_t->tv_sec = delta_t->tv_sec + 1;
		  delta_t->tv_nsec = dt_nsec-NSEC_PER_SEC;
	  }

	  else // dt_nsec < 0 means roll over
	  {
	          //printf("nanosec roll over\n");
		  delta_t->tv_sec = dt_sec-1;
		  delta_t->tv_nsec = NSEC_PER_SEC + dt_nsec;
	  }
  }

  return(OK);
}

/* ---- YUV conversion helpers -------------------------------------- */
void yuv2rgb_float(float y, float u, float v,
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
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
   int c = y-16, d = u - 128, e = v - 128;

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}


/* ---- convert_image -----------------------------------------------
 * Called from the capture thread (via read_frame).
 * Converts the raw MMAP frame into a heap buffer, then sends a
 * frame_msg pointer through the mqueue.  The MMAP buffer is returned
 * to the driver by read_frame() immediately after this returns.
 */
static void convert_image(const void *p, int size)
{
    int i, newi;
    struct frame_msg msg;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    clock_gettime(CLOCK_REALTIME, &msg.timestamp);

    framecnt++;
    msg.frame_num = framecnt;

    /* Instantaneous FPS: 1 / time since previous frame.
       Frame 1 has no previous timestamp so fps is reported as 0. */
    {
        static struct timespec prev_frame_time = {0, 0};
        double fps = 0.0;

        if (prev_frame_time.tv_sec != 0)
        {
            struct timespec ift = {0, 0};   /* inter-frame time */
            delta_t(&msg.timestamp, &prev_frame_time, &ift);
            double ift_sec = (double)ift.tv_sec + (double)ift.tv_nsec * 1e-9;
            if (ift_sec > 0.0)
                fps = 1.0 / ift_sec;
        }
        prev_frame_time = msg.timestamp;

        syslog(LOG_INFO, "capture: frame %u  fps=%.2f", framecnt, fps);
        printf("frame %u fps=%.2f: ", framecnt, fps);
    }

    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("GREY size %d\n", size);
        msg.size   = size;
        msg.is_ppm = 0;
        msg.data   = malloc(size);
        if (!msg.data) { perror("malloc grey"); return; }
        memcpy(msg.data, p, size);
    }
    else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {
#if defined(COLOR_CONVERT)
        printf("YUYV->RGB size %d\n", size);
        msg.size   = (size * 6) / 4;   /* 4 bytes YUYV -> 6 bytes RGB */
        msg.is_ppm = 1;
        msg.data   = malloc(msg.size);
        if (!msg.data) { perror("malloc yuyv->rgb"); return; }

        for (i = 0, newi = 0; i < size; i += 4, newi += 6)
        {
            y_temp  = (int)pptr[i];   u_temp = (int)pptr[i+1];
            y2_temp = (int)pptr[i+2]; v_temp = (int)pptr[i+3];
            yuv2rgb(y_temp,  u_temp, v_temp,
                    &msg.data[newi],   &msg.data[newi+1], &msg.data[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp,
                    &msg.data[newi+3], &msg.data[newi+4], &msg.data[newi+5]);
        }
#else
        printf("YUYV->Y size %d\n", size);
        msg.size   = size / 2;         /* 4 bytes YUYV -> 2 bytes YY  */
        msg.is_ppm = 0;
        msg.data   = malloc(msg.size);
        if (!msg.data) { perror("malloc yuyv->y"); return; }

        for (i = 0, newi = 0; i < size; i += 4, newi += 2)
        {
            msg.data[newi]   = pptr[i];     /* Y1 */
            msg.data[newi+1] = pptr[i+2];   /* Y2 */
        }
#endif
    }
    else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("RGB24 size %d\n", size);
        msg.size   = size;
        msg.is_ppm = 1;
        msg.data   = malloc(size);
        if (!msg.data) { perror("malloc rgb24"); return; }
        memcpy(msg.data, p, size);
    }
    else
    {
        printf("ERROR - unknown format\n");
        return;
    }


    /* Send pointer+metadata to the writer thread.  Blocks if queue full
       (natural backpressure: slows capture until writer catches up). */
    if (mq_send(frame_mq, (const char *)&msg, sizeof(msg), 30) == -1)
    {
        perror("mq_send");
        free(msg.data);   /* drop frame, do not leak */
    }
 
    fflush(stderr);
    fflush(stdout);
    clock_gettime(CLOCK_REALTIME, &rtclk_stop_time);
    delta_t(&rtclk_stop_time, &rtclk_start_time, &rtclk_dt);
    syslog(LOG_INFO, "reader: frame %u timestamp start=%ld.%09ld stop=%ld.%09ld delta=%ld.%09ld", msg.frame_num, rtclk_start_time.tv_sec, rtclk_start_time.tv_nsec, rtclk_stop_time.tv_sec, rtclk_stop_time.tv_nsec, rtclk_dt.tv_sec, rtclk_dt.tv_nsec);
}


/* ---- read_frame -------------------------------------------------- */

static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;
    clock_gettime(CLOCK_REALTIME, &rtclk_start_time); // save for delta_t calculation after capture, before conversion and mqueue send
    switch (io)
    {

        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length))
            {
                switch (errno)
                {

                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            convert_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))  // blocks until a frame is captured and available, or returns -1 with EAGAIN if non-blocking and no frame available
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

            assert(buf.index < n_buffers);

            /* Convert into heap buffer, then immediately return MMAP buffer
               to the driver — disk write happens later in the writer thread. */
            convert_image(buffers[buf.index].start, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))  // return buffer to driver immediately after conversion, so it's available for next capture while writer is doing disk I/O
                    errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            convert_image((void *)buf.m.userptr, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;
    }

    return 1;
}


/* ---- capture_thread_fn (producer) -------------------------------- */
/* Mirrors mainloop() but runs as a SCHED_FIFO thread.
   Sends a NULL sentinel to the writer after all frames are captured. */

void *capture_thread_fn(void *arg)
{
    unsigned int count;
    struct timespec read_delay;
    // struct timespec time_error;
    struct frame_msg sentinel;

    CLEAR(sentinel);   /* sentinel.data == NULL signals writer to exit */

    /* Target capture rate: 3 fps = 333,333,333 ns period.
       Using TIMER_ABSTIME so jitter never accumulates across frames. */
    read_delay.tv_sec  = 0;
    read_delay.tv_nsec = 333333333;   /* 1/3 second */

    count = frame_count;

    /* Initialized to zero; anchored to first captured frame timestamp
       so the first deadline is exactly one period after that frame,
       not thread start time. */
    struct timespec next_capture = {0, 0};

    printf("capture thread started\n");

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec  = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame())
            {
                /* Anchor to first frame on initial call */
                if (next_capture.tv_sec == 0)
                    clock_gettime(CLOCK_REALTIME, &next_capture);

                /* Advance absolute deadline by one period */
                next_capture.tv_nsec += read_delay.tv_nsec;
                if (next_capture.tv_nsec >= 1000000000L)
                {
                    next_capture.tv_sec++;
                    next_capture.tv_nsec -= 1000000000L;
                }

                /* Sleep until next absolute deadline — drift never accumulates */
                if (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME,
                                    &next_capture, NULL) != 0)
                    if (errno != EINTR)
                        perror("clock_nanosleep");

                count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if (count <= 0) break;
        }

        if (count <= 0) break;
    }

    /* Signal writer to exit. POSIX mqueue FIFO order guarantees all real
       frames are dequeued before the writer sees this sentinel. */
    printf("capture: all frames done, sending sentinel\n");
    if (mq_send(frame_mq, (const char *)&sentinel, sizeof(sentinel), 0) == -1)
        perror("mq_send sentinel");

    return NULL;
}


/* ---- writer_thread_fn (consumer) --------------------------------- */
/* Receives frame_msg pointers, writes file to disk, frees heap.
   Exits when it receives a sentinel (msg.data == NULL). */

void *writer_thread_fn(void *arg)
{
    struct frame_msg msg;
    unsigned int prio;
    int rc;
    // artificial delay of 1 second added to each write to simulate slow disk
    // and demonstrate backpressure on capture thread when mqueue is full.  
    // This is in addition to the actual time taken by the file write, which is measured and logged without the artificial delay.
    struct timespec write_delay;
    write_delay.tv_sec  = 1;
    write_delay.tv_nsec = 0000000000;   /* 1 second */

    struct timespec next_write = {0, 0};
    printf("writer thread started\n");

    do
    {
        // clock_gettime(CLOCK_REALTIME, &rtclk_start_write_time);  write + mq_receive time is measured in the writer thread, 
        // so start timestamp is taken here, just before mq_receive, and stop timestamp is taken after the file write, just before free.  
        // This captures the time spent in mq_receive (including any blocking if queue is empty) and the file write, but not the time spent in free, 
        // which should be minimal.
       
        rc = mq_receive(frame_mq, (char *)&msg, sizeof(msg), &prio);
        if (rc == -1)
        {
            if (errno == EINTR) continue;
            perror("mq_receive");
            break;
        }

        if (msg.data == NULL)
        {
            printf("writer: sentinel received, exiting\n");
            break;
        }
        clock_gettime(CLOCK_REALTIME, &rtclk_start_write_time); // start timestamp taken here, just after mq_receive, to exclude any blocking time if queue is empty
        syslog(LOG_INFO, "writer: frame %u prio=%u ", msg.frame_num, prio);
        
        if (msg.is_ppm)
            dump_ppm(msg.data, msg.size, msg.frame_num, &msg.timestamp);
        else
            dump_pgm(msg.data, msg.size, msg.frame_num, &msg.timestamp);
        clock_gettime(CLOCK_REALTIME, &rtclk_stop_write_time);
        delta_t(&rtclk_stop_write_time, &rtclk_start_write_time, &rtclk_dt);
        syslog(LOG_INFO, "writer: frame %u prio=%u timestamp start=%ld.%09ld stop=%ld.%09ld delta=%ld.%09ld", msg.frame_num, prio, rtclk_start_write_time.tv_sec, rtclk_start_write_time.tv_nsec, rtclk_stop_write_time.tv_sec, rtclk_stop_write_time.tv_nsec, rtclk_dt.tv_sec, rtclk_dt.tv_nsec);
        free(msg.data);

        /* Anchor deadline to current write time on first frame */
        if (next_write.tv_sec == 0)
            clock_gettime(CLOCK_REALTIME, &next_write);

        /* Advance absolute deadline by one period */
        next_write.tv_sec  += write_delay.tv_sec;
        next_write.tv_nsec += write_delay.tv_nsec;
        if (next_write.tv_nsec >= 1000000000L)
        {
            next_write.tv_sec++;
            next_write.tv_nsec -= 1000000000L;
        }

        if (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME,
                            &next_write, NULL) != 0)
            if (errno != EINTR)
                perror("clock_nanosleep");


    } while (1);

    return NULL;
}


/* ---- V4L2 lifecycle (unchanged) ---------------------------------- */

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io)
        {

        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                {
                        printf("allocated buffer %d\n", i);
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

static void uninit_device(void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }

        free(buffers);
}

static void init_read(unsigned int buffer_size)
{
        buffers = calloc(1, sizeof(*buffers));

        if (!buffers)
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start)
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 6;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
        {
                if (EINVAL == errno)
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2)
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = calloc(req.count, sizeof(*buffers));

        if (!buffers)
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc(4, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

static void init_device(void)
{
   /*  struct v4l2_streamparm parm;
    CLEAR(parm);
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = 30;
    xioctl(fd, VIDIOC_S_PARM, &parm);
    printf("granted: %u fps\n",
       parm.parm.capture.timeperframe.denominator); */

    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
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

    switch (io)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                fprintf(stderr, "%s does not support read i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                fprintf(stderr, "%s does not support streaming i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
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
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    switch (io)
    {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}

static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 GREY\n"
                 "-c | --count         Number of frames to grab [%i]\n"
                 "",
                 argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    int rc;
   // int rt_max_prio, rt_min_prio;

    dev_name = "/dev/video0";

    openlog("capture", LOG_PERROR | LOG_CONS, LOG_USER);

    for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                    short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                syslog(LOG_INFO, "selected IO_METHOD_MMAP");
                break;

            case 'r':
                io = IO_METHOD_READ;
                syslog(LOG_INFO, "selected IO_METHOD_READ");
                break;

            case 'u':
                io = IO_METHOD_USERPTR;
                syslog(LOG_INFO, "selected IO_METHOD_USERPTR");
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                        errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    /* ---- mqueue setup ---- */
    init_mqueue();

    /* ---- V4L2 setup ---- */
    open_device();
    init_device();
    start_capturing();

    setup_reader_writer_threads();

    /* Start writer first so it is ready before capture produces frames */
    if ((rc = pthread_create(&th_writer, &attr_writer, writer_thread_fn, NULL)) != 0)
    {
        perror("pthread_create writer");
        printf("rc=%d\n", rc);
        exit(EXIT_FAILURE);
    }
    printf("writer thread created\n");

    if ((rc = pthread_create(&th_capture, &attr_capture, capture_thread_fn, NULL)) != 0)
    {
        perror("pthread_create capture");
        printf("rc=%d\n", rc);
        exit(EXIT_FAILURE);
    }
    printf("capture thread created\n");

    pthread_join(th_capture, NULL);
    printf("capture thread joined\n");

    pthread_join(th_writer, NULL);
    printf("writer thread joined\n");

    /* ---- teardown ---- */
    stop_capturing();
    uninit_device();
    close_device();

    mq_close(frame_mq);  // close mqueue after threads have joined and are no longer using it
    mq_unlink(FRAME_MQ_NAME);  // unlink mqueue after close, to remove it from the system and avoid affecting other programs that use mqueues with the same name
    set_mqueue_msg_max(original_msg_max);  // restore original mqueue msg max in case it was changed by this program and is different from the default, to avoid affecting other programs that use mqueues

    fprintf(stderr, "\n");
    closelog();
    return 0;
}
