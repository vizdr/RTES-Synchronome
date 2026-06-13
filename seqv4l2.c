// Sam Siewert, December 2020
//
// Sequencer Generic Demonstration
//
// Sequencer - 100 Hz
//                   [gives semaphores to all other services]
// Service_1 - 25 Hz, every 4th Sequencer loop reads a V4L2 video frame
// Service_2 -  1 Hz, every 100th Sequencer loop writes out the current video frame
//
// With the above, priorities by RM policy would be:
//
// Sequencer = RT_MAX	@ 100 Hz
// Servcie_1 = RT_MAX-1	@ 25  Hz
// Service_2 = RT_MIN	@ 1   Hz
//

// This is necessary for CPU affinity macros in Linux
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <errno.h>
#include <sys/mman.h> // for mlockall
#include <signal.h>
#include <stdatomic.h>
#include <termios.h>     // for keyboard input without blocking
#include <sys/utsname.h> // for uname

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (4)
#define TRUE (1)
#define FALSE (0)

#define SEQ_CORE (1)
#define RT_CORE (2)

#define NUM_THREADS (4) /* number of service threads, not including viewer thread and key read thread */

// Of the available user space clocks, CLOCK_MONONTONIC_RAW is typically most precise and not subject to
// updates from external timer adjustments
//
// However, some POSIX functions like clock_nanosleep can only use adjusted CLOCK_MONOTONIC or CLOCK_REALTIME
//
// #define MY_CLOCK_TYPE CLOCK_REALTIME
// #define MY_CLOCK_TYPE CLOCK_MONOTONIC
#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW
// #define MY_CLOCK_TYPE CLOCK_REALTIME_COARSE
// #define MY_CLOCK_TYPE CLOCK_MONTONIC_COARSE

/* ── Viewer / Service_4 configuration ─────────────────────────── */
#define VIEWER_ENABLE 1        /* 0 = compile out all SDL2 code   */
#define VIEWER_SHOW_CURRENT 1  /* Window 0: latest frame          */
#define VIEWER_SHOW_PREVIOUS 1 /* Window 1: previous frame        */
#define VIEWER_SHOW_DIFF 1     /* Window 2: |curr-prev| x amplify */
#define VIEWER_WIN_W 640       /* display window width  (pixels)  */
#define VIEWER_WIN_H 480       /* display window height (pixels)  */
#define VIEWER_WIN_SPACING 20  /* gap between windows (pixels)    */
#define VIEWER_WIN_Y 50        /* initial Y of all windows        */
#define VIEWER_DIFF_AMPLIFY 4  /* diff brightness multiplier      */
#define VIEWER_CORE 3          /* CPU core for Service_4          */
/* ─────────────────────────────────────────────────────────────── */

#if VIEWER_ENABLE
#include <SDL2/SDL.h>
#endif

int abortTest = FALSE;
atomic_int abortS1 = FALSE, abortS2 = FALSE, abortS3 = FALSE, abortS5 = FALSE;
sem_t semS1, semS2, semS3, semS5, semS6;

#if VIEWER_ENABLE
int abortS4 = FALSE;
sem_t semS4;
#endif

struct timespec start_time_val;
double start_realtime;

static timer_t timer_1;
static struct itimerspec itime = {{1, 0}, {1, 0}};
static struct itimerspec last_itime;

static unsigned long long seqCnt = 0;

atomic_int skip_filter_requested = 0;

typedef struct
{
    int threadIdx;
} threadParams_t;

void Sequencer(int id);

void *Service_1_frame_acquisition(void *threadp);
void *Service_2_frame_process(void *threadp);
void *Service_3_frame_storage(void *threadp);
void *Service_5_frame_filter(void *threadp);
void *Service_6_keyboard_reader(void *threadp);

#if VIEWER_ENABLE
void *Service_4_frame_display(void *threadp);
int seq_frame_get_for_display(unsigned char *curr_rgb,
                              unsigned char *prev_rgb,
                              int *diff_amplify,
                              int *is_motion,
                              int *is_saved,
                              int *frame_num,
                              double *diff_pct);
#endif

int seq_frame_read(void);
int seq_frame_process(void);
int seq_frame_store(void);
int seq_frame_filter(void);

double getTimeMsec(void);
double realtime(struct timespec *tsptr);
void print_scheduler(void);

int v4l2_frame_acquisition_initialization(char *dev_name);
int v4l2_frame_acquisition_shutdown(void);
int v4l2_frame_acquisition_loop(char *dev_name);

static void skip_filter_handler(int sig, siginfo_t *si, void *ctx)
{
    int key = si->si_value.sival_int;
    switch (key)
    {
    case 's':
    case 'S':
        atomic_store_explicit(&skip_filter_requested, 1, memory_order_relaxed);
        ;
        break;
    case 'r':
    case 'R':
        atomic_store_explicit(&skip_filter_requested, 0, memory_order_relaxed);
        ;
        break;
    default:
        printf("Received unknown signal with key %d\n", key);
        return;
    }
}

// called to avoid minor page fault
static void stack_prefault(void)
{
    volatile unsigned char dummy[16384]; // touch 16 KB of stack depth
    memset((void *)dummy, 0, sizeof(dummy));
}

void main(void)
{
    struct timespec current_time_val, current_time_res;
    double current_realtime, current_realtime_res;

    char *dev_name = "/dev/video0";

    int i, rc, scope, flags = 0;

    cpu_set_t threadcpu;
    cpu_set_t seqcpu;
    cpu_set_t allcpuset;

    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio, cpuidx;

    struct utsname uts;
    char uname_buf[512];
    // open connection to syslog with specified options and facility.
    // LOG_CONS: write to console if syslog is not available, LOG_PERROR: also print to stderr, LOG_LOCAL1: use local1 facility
    openlog("seqv4l2", LOG_CONS | LOG_PERROR, LOG_LOCAL1);
    uname(&uts); // syscall to get system information and fill the uts structure
    snprintf(uname_buf, sizeof(uname_buf), "%s %s %s %s %s GNU/Linux",
             uts.sysname, uts.nodename, uts.release, uts.version, uts.machine); // pre-build string with system info
    syslog(LOG_CRIT, uname_buf);                                                // open a connection to the system logger for logging messages related to this program

#if VIEWER_ENABLE
    pthread_t viewer_thread;
    threadParams_t threadParams_viewer;
#endif
    pthread_t key_reader_thread;
    threadParams_t threadParams_key_reader;

    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;

    pthread_attr_t main_attr;
    pid_t mainpid;

    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;

    sa.sa_sigaction = skip_filter_handler;

    v4l2_frame_acquisition_initialization(dev_name);

    // lock memory to prevent paging in this demo application which is not handling page faults in real-time safe manner
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("mlockall");

    // required to get camera initialized and ready
    seq_frame_read();

    printf("Starting High Rate Sequencer Demo\n");
    clock_gettime(MY_CLOCK_TYPE, &start_time_val);
    start_realtime = realtime(&start_time_val);
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    clock_getres(MY_CLOCK_TYPE, &current_time_res);
    current_realtime_res = realtime(&current_time_res);

    // syslog(LOG_CRIT, "START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf", (current_realtime - start_realtime), current_realtime_res);

    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

    CPU_ZERO(&allcpuset);

    for (i = 0; i < NUM_CPU_CORES; i++)
        CPU_SET(i, &allcpuset);

    printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));

    // initialize the sequencer semaphores
    //
    if (sem_init(&semS1, 0, 0))
    {
        printf("Failed to initialize S1 semaphore\n");
        exit(-1);
    }
    if (sem_init(&semS2, 0, 0))
    {
        printf("Failed to initialize S2 semaphore\n");
        exit(-1);
    }
    if (sem_init(&semS3, 0, 0))
    {
        printf("Failed to initialize S2 semaphore\n");
        exit(-1);
    }
#if VIEWER_ENABLE
    if (sem_init(&semS4, 0, 0))
    {
        printf("Failed to initialize S4 semaphore\n");
        exit(-1);
    }
#endif
    if (sem_init(&semS5, 0, 0))
    {
        printf("Failed to initialize S5 semaphore\n");
        exit(-1);
    }
    if (sem_init(&semS6, 0, 0))
    {
        printf("Failed to initialize S6 semaphore\n");
        exit(-1);
    }
    mainpid = getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc = sched_getparam(mainpid, &main_param);
    main_param.sched_priority = rt_max_prio;
    rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if (rc < 0)
        perror("main_param");
    print_scheduler();

    pthread_attr_getscope(&main_attr, &scope);

    if (scope == PTHREAD_SCOPE_SYSTEM)
        printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
        printf("PTHREAD SCOPE PROCESS\n");
    else
        printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    CPU_ZERO(&seqcpu);
    CPU_SET(SEQ_CORE, &seqcpu);
    sched_setaffinity(getpid(), sizeof(cpu_set_t), &seqcpu);
    printf("\nSequencer (main) thread running on CPU=%d from %d available CPUs\n", sched_getcpu(), CPU_COUNT(&threadcpu));

    for (i = 0; i < (NUM_THREADS); i++)
    {

        // run ALL threads on core RT_CORE
        CPU_ZERO(&threadcpu);
        cpuidx = (RT_CORE);
        CPU_SET(cpuidx, &threadcpu);
        if (i == 0)
            printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

        rc = pthread_attr_init(&rt_sched_attr[i]);
        rc = pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        rc = pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        rc = pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

        rt_param[i].sched_priority = rt_max_prio - i;
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

        threadParams[i].threadIdx = i;
    }

    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1 = RT_MAX-1	@ 25 Hz
    //
    rt_param[0].sched_priority = rt_max_prio - 1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc = pthread_create(&threads[0],       // pointer to thread descriptor
                        &rt_sched_attr[0], // use specific attributes
                        //(void *)0,               // default attributes
                        Service_1_frame_acquisition, // thread function entry point
                        (void *)&(threadParams[0])   // parameters to pass in
    );
    if (rc < 0)
        perror("pthread_create for service 1 - V4L2 video frame acquisition");
    else
        printf("pthread_create successful for service 1\n");

    // Service_2 = RT_MAX-2	@ 1 Hz
    //
    rt_param[1].sched_priority = rt_max_prio - 2;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc = pthread_create(&threads[1], &rt_sched_attr[1], Service_2_frame_process, (void *)&(threadParams[1]));
    if (rc < 0)
        perror("pthread_create for service 2 - flash frame process");
    else
        printf("pthread_create successful for service 2\n");

    // Service_3 = RT_MAX-3	@ 1 Hz
    //
    rt_param[2].sched_priority = rt_max_prio - 3;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc = pthread_create(&threads[2], &rt_sched_attr[2], Service_3_frame_storage, (void *)&(threadParams[2]));
    if (rc < 0)
        perror("pthread_create for service 3 - flash frame storage");
    else
        printf("pthread_create successful for service 3\n");

#if VIEWER_ENABLE
    // Service_4 = SCHED_OTHER prio=0 @ 5 Hz, pinned to VIEWER_CORE.
    // SDL2 is not RT-safe; a dedicated non-RT core isolates display stalls
    // from the RT capture pipeline on RT_CORE.
    {
        pthread_attr_t viewer_attr;
        struct sched_param viewer_param;
        cpu_set_t viewercpu;

        viewer_param.sched_priority = 0;
        CPU_ZERO(&viewercpu);
        CPU_SET(VIEWER_CORE, &viewercpu);

        pthread_attr_init(&viewer_attr);
        pthread_attr_setinheritsched(&viewer_attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&viewer_attr, SCHED_OTHER);
        pthread_attr_setschedparam(&viewer_attr, &viewer_param);
        pthread_attr_setaffinity_np(&viewer_attr, sizeof(cpu_set_t), &viewercpu);

#ifdef VIEWER_ENABLE
        threadParams_viewer.threadIdx = (sizeof(threadParams) / sizeof(threadParams[0])) + 1; /* index after last RT service thread */
#endif

        rc = pthread_create(&viewer_thread, &viewer_attr,
                            Service_4_frame_display, (void *)&(threadParams_viewer));
        if (rc < 0)
            perror("pthread_create for service 4 - frame display");
        else
            printf("pthread_create successful for service 4\n");

        pthread_attr_destroy(&viewer_attr);
    }
#endif

    // Service_5 = RT_MAX-3	@ 1 Hz
    //
    rt_param[3].sched_priority = rt_max_prio - 4;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc = pthread_create(&threads[3], &rt_sched_attr[3], Service_5_frame_filter, (void *)&(threadParams[3]));
    if (rc < 0)
        perror("pthread_create for service 5 - filter frame");
    else
        printf("pthread_create successful for service 5\n");

    // Wait for service threads to initialize and await relese by sequencer.
    //
    // Note that the sleep is not necessary of RT service threads are created with
    // correct POSIX SCHED_FIFO priorities compared to non-RT priority of this main
    // program.
    //
    // sleep(1);

    // Service_6 SCHED_OTHER prio=0 @ 1 Hz, for keyboard reading to trigger synchronous abort of the test.
    pthread_attr_t key_read_attr;
    struct sched_param key_read_param;
    cpu_set_t viewercpu;

    key_read_param.sched_priority = 0;
    pthread_attr_init(&key_read_attr);
    pthread_attr_setinheritsched(&key_read_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&key_read_attr, SCHED_OTHER);
    pthread_attr_setschedparam(&key_read_attr, &key_read_param);
    CPU_ZERO(&viewercpu);
    CPU_SET(VIEWER_CORE, &viewercpu);
    pthread_attr_setaffinity_np(&key_read_attr, sizeof(cpu_set_t), &viewercpu);

    threadParams_key_reader.threadIdx = (sizeof(threadParams) / sizeof(threadParams[0])) + 2; /* index after last RT service thread and viewer thread */
    rc = pthread_create(&key_reader_thread, &key_read_attr,
                        Service_6_keyboard_reader, (void *)&(threadParams_key_reader));
    if (rc != 0)
        fprintf(stderr, "pthread_create for service 6 - keyboard reader: %s\n", strerror(rc));
    else
        printf("pthread_create successful for service 6\n");

    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");

    // Sequencer = RT_MAX	@ 100 Hz
    //
    /* set up to signal SIGALRM if timer expires */
    struct sigevent sev;
    sev.sigev_notify = SIGEV_THREAD_ID;
    sev.sigev_signo = SIGALRM;
    sev._sigev_un._tid = mainpid; // main thread TID
    sev.sigev_value.sival_ptr = &timer_1;
    timer_create(CLOCK_REALTIME, &sev, &timer_1);

    signal(SIGALRM, (void (*)())Sequencer);

    /* arm the interval timer */
    itime.it_interval.tv_sec = 0;
    itime.it_interval.tv_nsec = 10000000;
    itime.it_value.tv_sec = 0;
    itime.it_value.tv_nsec = 10000000;
    // itime.it_interval.tv_sec = 1;
    // itime.it_interval.tv_nsec = 0;
    // itime.it_value.tv_sec = 1;
    // itime.it_value.tv_nsec = 0;

    timer_settime(timer_1, flags, &itime, &last_itime);

    for (i = 0; i < (NUM_THREADS); i++)
    {
        if ((rc = pthread_join(threads[i], NULL)) < 0)
            perror("main pthread_join");
        else
            printf("joined thread %d\n", i);
    }

    if ((rc = pthread_create(&key_reader_thread, &key_read_attr,
                             Service_6_keyboard_reader, (void *)&(threadParams_key_reader))) != 0)
        fprintf(stderr, "pthread_create for service 6 - keyboard reader: %s\n", strerror(rc));
    else
        printf("pthread_create successful for service 6\n");

#if VIEWER_ENABLE
    if ((rc = pthread_join(viewer_thread, NULL)) < 0)
        perror("main pthread_join for viewer thread");
    else
        printf("joined viewer thread\n");
#endif

    v4l2_frame_acquisition_shutdown();

    printf("\nTEST COMPLETE\n");
}

void Sequencer(int id)
{
    struct timespec current_time_val;
    double current_realtime;
    int rc, flags = 0;

    // received interval timer signal
    if (abortTest)
    {
        // disable interval timer
        itime.it_interval.tv_sec = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec = 0;
        itime.it_value.tv_nsec = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);
        printf("Disabling sequencer interval timer with abort=%d and %llu\n", abortTest, seqCnt);

        // shutdown all services
        abortS1 = TRUE;
        abortS2 = TRUE;
        abortS3 = TRUE;
        abortS5 = TRUE;
#if VIEWER_ENABLE
        abortS4 = TRUE;
#endif
        sem_post(&semS1);
        sem_post(&semS2);
        sem_post(&semS3);
        sem_post(&semS5);
        sem_post(&semS6);
#if VIEWER_ENABLE
        sem_post(&semS4);
#endif
    }

    seqCnt++;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    // printf("Sequencer on core %d for cycle %llu @ sec=%6.9lf\n", sched_getcpu(), seqCnt, current_realtime-start_realtime);
    // syslog(LOG_CRIT, "Sequencer on core %d for cycle %llu @ sec=%6.9lf", sched_getcpu(), seqCnt, current_realtime-start_realtime);

    // Release each service at a sub-rate of the generic sequencer rate

    // Servcie_1 @ 5 Hz
    if ((seqCnt % 20) == 0)
        sem_post(&semS1);

    // Service_2 @ 1 Hz
    if ((seqCnt % 100) == 0)
        sem_post(&semS2);

    // Service_3 @ 1 Hz
    if ((seqCnt % 100) == 0)
        sem_post(&semS3);

#if VIEWER_ENABLE
    // Service_4 (Viewer) @ 5 Hz — same cadence as Service_1 (frame acquisition).
    // Service_1 is higher priority and runs on RT_CORE first; by the time Service_4
    // wakes on VIEWER_CORE the ring buffer slot it reads is already complete.
    if ((seqCnt % 20) == 0)
        sem_post(&semS4);
#endif

    // Service_5 @ 1 Hz
    if ((seqCnt % 100) == 0)
        sem_post(&semS5);

    // Service_6 @ 1 Hz
    if ((seqCnt % 100) == 0)
        sem_post(&semS6);
}

void *Service_1_frame_acquisition(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S1Cnt = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    printf("\nFrame acquisitation thread running on CPU=%d \n", sched_getcpu());
    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    // syslog(LOG_CRIT, "S1 thread @ sec=%6.9lf", current_realtime - start_realtime);

    // to avoid zero-fill allocation
    stack_prefault();

    while (!abortS1) // check for synchronous abort request
    {
        // wait for service request from the sequencer, a signal handler or ISR in kernel
        sem_wait(&semS1);

        if (abortS1)
            break;
        S1Cnt++;

        // DO WORK - acquire V4L2 frame here or OpenCV frame here
        seq_frame_read();

        // on order of up to milliseconds of latency to get time
        clock_gettime(MY_CLOCK_TYPE, &current_time_val);
        current_realtime = realtime(&current_time_val);
        // syslog(LOG_CRIT, "S1 at 5 Hz on core %d for release %llu @ sec=%6.9lf", sched_getcpu(), S1Cnt, current_realtime - start_realtime);

        if (S1Cnt > 2500)
        {
            abortTest = TRUE;
        };
    }

    // Resource shutdown here
    //
    pthread_exit((void *)0);
}

void *Service_2_frame_process(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S2Cnt = 0;
    int process_cnt;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    printf("\nFrame processing thread running on CPU=%d \n", sched_getcpu());
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    // syslog(LOG_CRIT, "S2 thread @ sec=%6.9lf", current_realtime - start_realtime);

    // to avoid zero-fill allocation
    stack_prefault();

    while (!abortS2)
    {
        sem_wait(&semS2);

        if (abortS2)
            break;
        S2Cnt++;

        // DO WORK - transform frame

        process_cnt = seq_frame_process();

        clock_gettime(MY_CLOCK_TYPE, &current_time_val);
        current_realtime = realtime(&current_time_val);
        // syslog(LOG_CRIT, "S2 at 1 Hz on core %d for release %llu @ sec=%6.9lf", sched_getcpu(), S2Cnt, current_realtime - start_realtime);
    }

    pthread_exit((void *)0);
}

void *Service_3_frame_storage(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S3Cnt = 0;
    int store_cnt;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    printf("\nFrame storage thread running on CPU=%d \n", sched_getcpu());
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    // syslog(LOG_CRIT, "S3 thread @ sec=%6.9lf\n", current_realtime - start_realtime);

    // to avoid zero-fill allocation
    stack_prefault();

    while (!abortS3)
    {
        sem_wait(&semS3);

        if (abortS3)
            break;
        S3Cnt++;

        // DO WORK - store frame
        store_cnt = seq_frame_store();

        clock_gettime(MY_CLOCK_TYPE, &current_time_val);
        current_realtime = realtime(&current_time_val);
        // syslog(LOG_CRIT, "S3 at 1 Hz on core %d for release %llu @ sec=%6.9lf", sched_getcpu(), S3Cnt, current_realtime - start_realtime);

        // after last write, set synchronous abort
        if (store_cnt == 61)
        {
            abortTest = TRUE;
        };
    }

    pthread_exit((void *)0);
}

// Service_4 is the viewer thread which runs on a separate non-RT core and is triggered at the same cadence as Service_1 (frame acquisition).
// Thread function is defined after Service_5 to keep all RT services together

// Service_5 for applying a filter to the current frame
void *Service_5_frame_filter(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S5Cnt = 0;
    int filter_cnt = 0;
    int filter_cnt_prev = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    printf("\nFrame filter thread running on CPU=%d \n", sched_getcpu());
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    // syslog(LOG_CRIT, "S5 thread @ sec=%6.9lf", current_realtime - start_realtime);

    // to avoid zero-fill allocation
    stack_prefault();

    while (!abortS5)
    {
        sem_wait(&semS5);

        if (abortS5)
            break;
        S5Cnt++;

        // DO WORK - apply filter to frame
        // filter_cnt = seq_frame_filter();
        if (skip_filter_requested > 0)
        {
            printf("S5: skipping filter work due to user input\n");
        }
        else
        {
            filter_cnt_prev = filter_cnt;
            filter_cnt = seq_frame_filter();
            if (filter_cnt_prev == filter_cnt)
            {
                // syslog(LOG_CRIT, "S5: filter was not applied for release %llu, filter count %d", S5Cnt, filter_cnt);
            }
        }

        clock_gettime(MY_CLOCK_TYPE, &current_time_val);
        current_realtime = realtime(&current_time_val);
        // syslog(LOG_CRIT, "S5 at 1 Hz on core %d for release %llu @ sec=%6.9lf", sched_getcpu(), S5Cnt, current_realtime - start_realtime);

        // after last write, set synchronous abort
        if (filter_cnt == 1802)
        {
            abortTest = TRUE;
        };
    }

    pthread_exit((void *)0);
}

double getTimeMsec(void)
{
    struct timespec event_ts = {0, 0};

    clock_gettime(MY_CLOCK_TYPE, &event_ts);
    return ((event_ts.tv_sec) * 1000.0) + ((event_ts.tv_nsec) / 1000000.0);
}

double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec) / 1000000000.0));
}

void print_scheduler(void)
{
    int schedType;

    schedType = sched_getscheduler(getpid());

    switch (schedType)
    {
    case SCHED_FIFO:
        printf("Pthread Policy is SCHED_FIFO\n");
        break;
    case SCHED_OTHER:
        printf("Pthread Policy is SCHED_OTHER\n");
        exit(-1);
        break;
    case SCHED_RR:
        printf("Pthread Policy is SCHED_RR\n");
        exit(-1);
        break;
    // case SCHED_DEADLINE:
    //     printf("Pthread Policy is SCHED_DEADLINE\n"); exit(-1);
    //     break;
    default:
        printf("Pthread Policy is UNKNOWN\n");
        exit(-1);
    }
}

#if VIEWER_ENABLE
void *Service_4_frame_display(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S4Cnt = 0;
    int nwins = 0;
    int curr_slot = -1, prev_slot = -1, diff_slot = -1;
    int i, frame_bytes;
    int is_motion = 0, is_saved = 0, fnum = 0, diff_amp;
    double diff_pct = 0.0;
    char title[80];
    SDL_Event ev;
    int xpos = 0;
    printf("\nFrame display thread running on CPU=%d \n", sched_getcpu());
    static unsigned char curr_rgb[VIEWER_WIN_W * VIEWER_WIN_H * 3];
    static unsigned char prev_rgb[VIEWER_WIN_W * VIEWER_WIN_H * 3];
    static unsigned char diff_buf[VIEWER_WIN_W * VIEWER_WIN_H * 3];

    SDL_Window *win[3] = {NULL, NULL, NULL};
    SDL_Renderer *ren[3] = {NULL, NULL, NULL};
    SDL_Texture *tex[3] = {NULL, NULL, NULL};

    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        pthread_exit((void *)0);
    }

/* Create one SDL window + renderer + streaming RGB24 texture at xpos, advance xpos. */
#define MAKE_WINDOW(label, slot_var)                                                           \
    do                                                                                         \
    {                                                                                          \
        (slot_var) = nwins;                                                                    \
        win[nwins] = SDL_CreateWindow((label), xpos, VIEWER_WIN_Y,                             \
                                      VIEWER_WIN_W, VIEWER_WIN_H, SDL_WINDOW_RESIZABLE);       \
        ren[nwins] = SDL_CreateRenderer(win[nwins], -1,                                        \
                                        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC); \
        tex[nwins] = SDL_CreateTexture(ren[nwins], SDL_PIXELFORMAT_RGB24,                      \
                                       SDL_TEXTUREACCESS_STREAMING,                            \
                                       VIEWER_WIN_W, VIEWER_WIN_H);                            \
        xpos += VIEWER_WIN_W + VIEWER_WIN_SPACING;                                             \
        nwins++;                                                                               \
    } while (0)

#if VIEWER_SHOW_CURRENT
    MAKE_WINDOW("Current Frame", curr_slot);
#endif
#if VIEWER_SHOW_PREVIOUS
    MAKE_WINDOW("Previous Frame", prev_slot);
#endif
#if VIEWER_SHOW_DIFF
    MAKE_WINDOW("Diff", diff_slot);
#endif

#undef MAKE_WINDOW

    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    // syslog(LOG_CRIT, "S4 thread @ sec=%6.9lf", current_realtime - start_realtime);
    // printf("S4 thread @ sec=%6.9lf\n", current_realtime - start_realtime);

    while (!abortS4)
    {
        sem_wait(&semS4);
        if (abortS4)
            break;
        S4Cnt++;

        /* to avoid further work on outdated frames we drain queued posts so we always render the most recent frame */
        while (sem_trywait(&semS4) == 0)
        {
        }

        /* SDL event pump — must run on the thread that owns the windows */
        while (SDL_PollEvent(&ev))
        {
            if (ev.type == SDL_QUIT ||
                (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE))
                abortTest = TRUE;
        }

        if (abortTest)
        {
            abortS4 = TRUE;
            break;
        }

        diff_amp = VIEWER_DIFF_AMPLIFY;
        if (seq_frame_get_for_display(curr_rgb, prev_rgb,
                                      &diff_amp, &is_motion, &is_saved, &fnum,
                                      &diff_pct) < 0)
            continue; /* fewer than 2 frames acquired yet */

        /* per-pixel amplified absolute diff across all RGB channels */
        frame_bytes = VIEWER_WIN_W * VIEWER_WIN_H * 3;
        for (i = 0; i < frame_bytes; i++)
        {
            int d = abs((int)curr_rgb[i] - (int)prev_rgb[i]) * diff_amp;
            diff_buf[i] = (unsigned char)(d > 255 ? 255 : d);
        }

#if VIEWER_SHOW_CURRENT
        snprintf(title, sizeof(title), "Current Frame #%d%s%s",
                 fnum, is_motion ? " [MOTION]" : "", is_saved ? " [SAVED]" : "");
        SDL_SetWindowTitle(win[curr_slot], title);
        SDL_UpdateTexture(tex[curr_slot], NULL, curr_rgb, VIEWER_WIN_W * 3);
        SDL_RenderClear(ren[curr_slot]);
        SDL_RenderCopy(ren[curr_slot], tex[curr_slot], NULL, NULL);
        SDL_RenderPresent(ren[curr_slot]);
#endif
#if VIEWER_SHOW_PREVIOUS
        SDL_UpdateTexture(tex[prev_slot], NULL, prev_rgb, VIEWER_WIN_W * 3);
        SDL_RenderClear(ren[prev_slot]);
        SDL_RenderCopy(ren[prev_slot], tex[prev_slot], NULL, NULL);
        SDL_RenderPresent(ren[prev_slot]);
#endif
#if VIEWER_SHOW_DIFF
        snprintf(title, sizeof(title), "Diff (x%d) %.2f%%", diff_amp, diff_pct);
        SDL_SetWindowTitle(win[diff_slot], title);
        SDL_UpdateTexture(tex[diff_slot], NULL, diff_buf, VIEWER_WIN_W * 3);
        SDL_RenderClear(ren[diff_slot]);
        SDL_RenderCopy(ren[diff_slot], tex[diff_slot], NULL, NULL);
        SDL_RenderPresent(ren[diff_slot]);
#endif

        clock_gettime(MY_CLOCK_TYPE, &current_time_val);
        current_realtime = realtime(&current_time_val);
        // syslog(LOG_CRIT, "S4 display on core %d for release %llu @ sec=%6.9lf", sched_getcpu(), S4Cnt, current_realtime - start_realtime);
    }

    for (i = 0; i < nwins; i++)
    {
        SDL_DestroyTexture(tex[i]);
        SDL_DestroyRenderer(ren[i]);
        SDL_DestroyWindow(win[i]);
    }
    SDL_Quit();
    pthread_exit((void *)0);
}
#endif /* VIEWER_ENABLE */

void *Service_6_keyboard_reader(void *threadp)
{
    struct termios orig_termios, raw_termios; // for non-blocking keyboard input
    fd_set readfds;
    struct timeval timeout;
    char ch;
    int rv;
    printf("\nKeyboard Reader thread running on CPU=%d \n", sched_getcpu());
    tcgetattr(STDIN_FILENO, &orig_termios);         // get current terminal settings
    raw_termios = orig_termios;                     // modify for raw input: non-canonical mode, no echo, and non-blocking read
    raw_termios.c_lflag &= ~(ICANON | ECHO);        // disable canonical mode and echo
    raw_termios.c_cc[VMIN] = 0;                     // return immediately from read() if no input
    raw_termios.c_cc[VTIME] = 0;                    // no blocking timeout — we will use select() for timing instead
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_termios); // apply raw settings immediately

    while (!abortTest)
    {
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 1000000; // 1000 ms poll period — reset every iteration

        rv = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout); // wait for input or timeout

        if (rv > 0 && FD_ISSET(STDIN_FILENO, &readfds))
        {
            if (read(STDIN_FILENO, &ch, 1) == 1)
            {
                if (ch == 's' || ch == 'S' || ch == 'r' || ch == 'R')
                {
                    union sigval sv = {.sival_int = (int)ch};
                    sigqueue(getpid(), SIGRTMIN + 1, sv); // send signal to self to request filter skip or resume
                }
                // any other key: just ignore it, no abort
            }
            else
            {
                printf("Read error or EOF on stdin\n");
                abortTest = TRUE;
            }
        }
        else if (rv < 0 && errno != EINTR)
        {
            break; // genuine select() error
        }
        // rv == 0: timeout — loop back and re-check abortTest
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    pthread_exit((void *)0);
}
