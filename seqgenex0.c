// Sam Siewert, December 2017
// Modified by Vladimir Zdravkov, March - April 2026
//
// Sequencer Generic
//
// The purpose of this code is to provide an example for how to best
// sequence a set of periodic services for problems similar to and including
// the final project in real-time systems.
//
// For example: Service_1 for camera frame aquisition
//              Service_2 for image analysis and timestamping
//              Service_3 for image processing (difference images)
//              Service_4 for save time-stamped image to file service
//              Service_5 for save processed image to file service
//              Service_6 for send image to remote server to save copy
//              Service_7 for elapsed time in syslog each minute for debug
//
// Sequencer Generic to emulate Example A1 assuming millisec time resolution
// A1:
// Service_1, S1, T1=2,  C1=1, D=T
// Service_2, S2, T2=10, C2=1, D=T
// Service_3, S3, T3=15, C3=2, D=T
// A1:
// Sequencer - 100 Hz [gives semaphores to all other services]
// Service_1 - 50 Hz, every other Sequencer loop
// Service_2 - 10 Hz, every 10th Sequencer loop
// Service_3 - 6.67 Hz, every 15th Sequencer loop
//
// With the above, priorities by RM policy would be for A1-A7:
//
//                                           A1    A2    A3    A4   A5    A6   A7
// Sequencer = RT_MAX	@ 100 Hz,            T= 1  T= 1  T=1   T=1   T=1  T=1  T=1
// Servcie_1 = RT_MAX-1	@ 50, 33.33 Hz,      T= 2  T= 2  T=2   T=2   T=2  T=2  T=3
// Service_2 = RT_MAX-2	@ 10,20,16.67 Hz,    T=10  T= 5  T=5   T=5   T=5  T=5  T=6
// Service_3 = RT_MAX-3	@ 6.67,10,14.29,1.11 T=15  T=15  T=10  T=7   T=10 T=7  T=9
// Service_4 = RT_MAX-4	@ 5,7.69 Hz          ---   ---   T=20  T=13  ---  T=13 ---
// Note: the service periods  are based on the assumption that the sequencer is running at 100 Hz, which is what the default configuration of this code is.  If you change the sequencer rate, then you will need to adjust the service release logic in the sequencer thread function and the service periods in the edf_thread_params structure accordingly.
// Here are a few hardware/platform configuration settings
// that you should also check before running this code:
//
// 1) Check to ensure all your CPU cores on in an online state.
//
// 2) Check /sys/devices/system/cpu or do lscpu.
//
//    echo 1 > /sys/devices/system/cpu/cpu1/online
//    echo 1 > /sys/devices/system/cpu/cpu2/online
//    echo 1 > /sys/devices/system/cpu/cpu3/online
//
// 3) Check for precision time resolution and support with cat /proc/timer_list
//
// 4) Ideally all printf calls should be eliminated as they can interfere with
//    timing.  They should be replaced with an in-memory event logger or at
//    least calls to syslog.
//

// This is necessary for CPU affinity macros in Linux
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h> // for SCHED_DEADLINE and sched_attr. If glibc 2.41 not available, replace with sched_setattr #include <linux/sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>
#include <errno.h>
#include "seqgen.h"
#include <sys/sysinfo.h>
#include <sys/mman.h>    // for mlockall
#include <sys/utsname.h> // for uname

// #define EDF
#define ASS_NUM 7 /* select assignment: 4, 5, 6, or 7 */
#define _STRINGIFY(x) #x
#define STRINGIFY(x) _STRINGIFY(x)
#define ASS STRINGIFY(ASS_NUM)

#if defined(EDF)

#include <sys/syscall.h> // for syscall numbers

// not in use, reserved for the case sched_setattr from glibc 2.41 is not available. 
static inline int sys_sched_setattr(pid_t pid, struct sched_attr *attr, unsigned int flags)
{
    return syscall(SYS_sched_setattr, pid, attr, flags);
}

#endif

// adjacent string literals are concatenated at compile time, so we can use this to build our log message format string with the course and assignment number
#define COURSE_ASS_NUMBER(crs, nmb) "[COURSE:" crs "][ASSIGNMENT:" nmb "] %s"
#define COURSE_ASS_NUMBER_SCHED(crs, nmb) "[COURSE:" crs "][ASSIGNMENT:" nmb "]:Thread %d start %llu @ %lf on core %d"
#define COURSE "2"

#define MY_CLOCK CLOCK_MONOTONIC

// This is a simple Fibonacci calculation to provide some load for the service threads
static double start_time_fib = 0;
static double end_time_fib = 0;

unsigned int seqIterations = 14;     // 47 is the largest Fibonacci number that can be calculated without overflow in 32 bits, which is what we are using for the fib variables.  This gives us a reasonable amount of load to test the sequencer and drift control.  You can increase this for more load, but be aware of overflow issues with the fib variables.
unsigned int reqIterations = 100000; // 10 million iterations gives us about 1.5 seconds of load on the test machine for fib(47).  Adjust as needed for your machine and desired load level.
volatile unsigned int fib = 0, fib0 = 0, fib1 = 1;

int FIB_TEST(unsigned int seqCnt, unsigned int iterCnt, unsigned int serviceIdx)
{
    start_time_fib = getTimeMsec();
    // syslog(LOG_CRIT, "FIB_TEST: start fib(%u, %u) @ service %u launched at %lf sec\n", seqCnt, iterCnt, serviceIdx, start_time_fib);
    unsigned int idx = 0, jdx = 1;
    for (idx = 0; idx < iterCnt; idx++)
    {
        fib = fib0 + fib1;
        while (jdx < seqCnt)
        {
            fib0 = fib1;
            fib1 = fib;
            fib = fib0 + fib1;
            jdx++;
        }

        jdx = 0;
    }
    end_time_fib = getTimeMsec();
    // syslog(LOG_CRIT, "FIB_TEST: stop fib(%u, %u) = %u @ service %u durated %lf sec\n", seqCnt, iterCnt, idx, serviceIdx, (end_time_fib - start_time_fib));
    return idx;
}

#define ABS_DELAY
#define DRIFT_CONTROL
#define NUM_THREADS (4 + 1)

int abortTest = FALSE;
int abortS1 = FALSE, abortS2 = FALSE, abortS3 = FALSE, abortS4 = FALSE;
sem_t semS1, semS2, semS3, semS4;
static double start_time = 0;

pthread_t threads[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];
pthread_attr_t main_attr;
int rt_max_prio, rt_min_prio;
struct sched_param rt_param[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];
struct rm_seq_params
{
    __u64 divisorService_1; /* divisor for service 1 release relative to generic sequencer */
    __u64 divisorService_2; /* divisor for service 2 release relative to generic sequencer */
    __u64 divisorService_3; /* divisor for service 3 release relative to generic sequencer */
    __u64 divisorService_4; /* divisor for service 4 release relative to generic sequencer, if used in the assignment */
};

#if defined(EDF)
struct sched_attr edf_attr[NUM_THREADS]; // for EDF scheduling attributes if using EDF
#endif

struct edf_params
{
    __u64 runtime;  /* WCET in nanoseconds */
    __u64 deadline; /* relative deadline in nanoseconds */
    __u64 period;   /* period in nanoseconds */
};

/* Thread order: [0]=Sequencer, [1]=Service_1, [2]=Service_2, [3]=Service_3, [4]=Service_4
 * Periods come from the assignment table in the file header (ms -> ns).
 * Runtime = 10% of period as WCET estimate.
 * Service_4 is zeroed for assignments where it is not used (A5, A7). */
#if ASS_NUM == 4
static struct edf_params edf_thread_params[NUM_THREADS] = {
    {100000, 1000000, 1000000},    /* [0] Seq: T=1ms  */
    {200000, 2000000, 2000000},    /* [1] S1:  T=2ms  */
    {500000, 5000000, 5000000},    /* [2] S2:  T=5ms  */
    {700000, 7000000, 7000000},    /* [3] S3:  T=7ms  */
    {1300000, 13000000, 13000000}, /* [4] S4:  T=13ms */
};
static struct rm_seq_params rm_seq_divs = {
    .divisorService_1 = 2,
    .divisorService_2 = 5,
    .divisorService_3 = 7,
    .divisorService_4 = 13};
#elif ASS_NUM == 5
static struct edf_params edf_thread_params[NUM_THREADS] = {
    {100000, 1000000, 1000000},    /* [0] Seq: T=1ms  */
    {200000, 2000000, 2000000},    /* [1] S1:  T=2ms  */
    {500000, 5000000, 5000000},    /* [2] S2:  T=5ms  */
    {1000000, 10000000, 10000000}, /* [3] S3:  T=10ms */
    {0, 0, 0},                     /* [4] S4:  not used in A5 */
};
static struct rm_seq_params rm_seq_divs = {
    .divisorService_1 = 2,
    .divisorService_2 = 5,
    .divisorService_3 = 10,
    .divisorService_4 = 0};
#elif ASS_NUM == 6
static struct edf_params edf_thread_params[NUM_THREADS] = {
    {100000, 1000000, 1000000},    /* [0] Seq: T=1ms  */
    {200000, 2000000, 2000000},    /* [1] S1:  T=2ms  */
    {500000, 5000000, 5000000},    /* [2] S2:  T=5ms  */
    {700000, 7000000, 7000000},    /* [3] S3:  T=7ms  */
    {1300000, 13000000, 13000000}, /* [4] S4:  T=13ms */
};
static struct rm_seq_params rm_seq_divs = {
    .divisorService_1 = 2,
    .divisorService_2 = 5,
    .divisorService_3 = 7,
    .divisorService_4 = 13};
#elif ASS_NUM == 7
static struct edf_params edf_thread_params[NUM_THREADS] = {
    {100000, 1000000, 1000000}, /* [0] Seq: T=1ms  */
    {300000, 3000000, 3000000}, /* [1] S1:  T=3ms  */
    {600000, 6000000, 6000000}, /* [2] S2:  T=6ms  */
    {900000, 9000000, 9000000}, /* [3] S3:  T=9ms  */
    {0, 0, 0},                  /* [4] S4:  not used in A7 */
};
static struct rm_seq_params rm_seq_divs = {
    .divisorService_1 = 3,
    .divisorService_2 = 6,
    .divisorService_3 = 9,
    .divisorService_4 = 0};
#else
#error "ASS_NUM must be 4, 5, 6, or 7"
#endif

int main(void)
{
    double current_time;
    struct timespec rt_res, monotonic_res; // for checking clock resolution, but we are only using the realtime clock in this example
    int i, rc, cpuidx;
    cpu_set_t threadcpu;
    struct sched_param main_param;
    pid_t mainpid;

    struct utsname uts;
    char uname_buf[512];
    // open connection to syslog with specified options and facility.
    // LOG_CONS: write to console if syslog is not available, LOG_PERROR: also print to stderr, LOG_LOCAL1: use local1 facility
    openlog("seqgenex0", LOG_CONS | LOG_PERROR, LOG_LOCAL1);
    uname(&uts); // syscall to get system information and fill the uts structure
    snprintf(uname_buf, sizeof(uname_buf), "%s %s %s %s %s GNU/Linux",
             uts.sysname, uts.nodename, uts.release, uts.version, uts.machine); // pre-build string with system info
    syslog(LOG_CRIT, COURSE_ASS_NUMBER(COURSE, ASS), uname_buf);                // open a connection to the system logger for logging messages related to this program

    // Lock all current and future memory pages into RAM to prevent translation faults
    // during thread execution. Without this, lazy page allocation causes threads to
    // trigger page faults inside the kernel while holding RCU read locks, which blocks
    // RCU grace periods and causes rcu_preempt stall warnings.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("mlockall");

    start_time = getTimeMsec();

    // delay start for a second
    usleep(1000000);

    printf("Starting High Rate Sequencer Example\n");
    get_cpu_core_config();

    clock_getres(CLOCK_REALTIME, &rt_res);
    printf("RT clock resolution is %ld sec, %ld nsec\n", rt_res.tv_sec, rt_res.tv_nsec);

    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

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
        printf("Failed to initialize S3 semaphore\n");
        exit(-1);
    }
    if (sem_init(&semS4, 0, 0)) // Assignment 3 addition - add a semaphore for service 4
    {
        printf("Failed to initialize S4 semaphore\n");
        exit(-1);
    }

    // set up the main thread as the highest priority thread so that it can create the sequencer and service threads with the appropriate priorities based on RM policy, and then wait for them to complete.  If we did not do this, then the main thread would be at the default priority and could end up running after creating a service thread at a higher priority than the main thread, which could cause timing issues with the test.
    mainpid = getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc = sched_getparam(mainpid, &main_param);
    main_param.sched_priority = rt_max_prio;
    rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if (rc < 0)
        perror("main_param");

    print_scheduler();

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);
    ///////////////////////////////////////////////////////////////////////////////////
    // initialize the thread attributes for the service threads
    for (i = 0; i < NUM_THREADS; i++)
    {

        CPU_ZERO(&threadcpu);
        cpuidx = (3);
        CPU_SET(cpuidx, &threadcpu);

        rc = pthread_attr_init(&rt_sched_attr[i]);
        rc = pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        
// set the scheduling policy for the thread to SCHED_FIFO and assign priorities based on RM policy, with the sequencer having the highest priority
#if defined(EDF)
        // A DEADLINE thread must always be migratable 
        // across the entire root domain — the kernel enforces this at every point where affinity could be restricted.
        // The option with EDF is cpusets — create a root domain of your chosen CPUs before launching the program;
        // DEADLINE threads can then be restricted to that full subset
        edf_attr[i].size = sizeof(struct sched_attr);
        edf_attr[i].sched_policy = SCHED_DEADLINE;
        edf_attr[i].sched_runtime = edf_thread_params[i].runtime;
        edf_attr[i].sched_deadline = edf_thread_params[i].deadline;
        edf_attr[i].sched_period = edf_thread_params[i].period;
        edf_attr[i].sched_flags = SCHED_FLAG_RESET_ON_FORK; // reset to default scheduling attributes on fork to prevent child threads from inheriting the EDF scheduling attributes, which can
#else
        rc = pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);
        rc = pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        rt_param[i].sched_priority = rt_max_prio - i;
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
#endif

        threadParams[i].threadIdx = i;
    }

    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    current_time = getTimeMsec();
// syslog(LOG_CRIT, "RTMAIN: on cpu=%d @ sec=%lf, elapsed=%lf\n", sched_getcpu(), start_time, current_time);

// Create Service threads which will block awaiting release for:
//

// Servcie_1 = RT_MAX-1	@ 50 Hz
//
#ifndef EDF
    rt_param[1].sched_priority = rt_max_prio - 1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
#endif
    rc = pthread_create(&threads[1],       // pointer to thread descriptor
                        &rt_sched_attr[1], // use specific attributes
                        //(void *)0,               // default attributes
                        Service_1,                 // thread function entry point
                        (void *)&(threadParams[1]) // parameters to pass in
    );

    if (rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");

// Service_2 = RT_MAX-2	@ 10 Hz
//
#ifndef EDF
    rt_param[2].sched_priority = rt_max_prio - 2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
#endif
    rc = pthread_create(&threads[2],       // pointer to thread descriptor
                        &rt_sched_attr[2], // use specific attributes
                        //(void *)0,               // default attributes
                        Service_2,                 // thread function entry point
                        (void *)&(threadParams[2]) // parameters to pass in
    );
    if (rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");

    // Service_3 = RT_MAX-3	@ 6.67 Hz
    //
#ifndef EDF
    rt_param[3].sched_priority = rt_max_prio - 3;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
#endif
    rc = pthread_create(&threads[3],       // pointer to thread descriptor
                        &rt_sched_attr[3], // use specific attributes
                        //(void *)0,               // default attributes
                        Service_3,                 // thread function entry point
                        (void *)&(threadParams[3]) // parameters to pass in
    );
    if (rc < 0)
        perror("pthread_create for service 3");
    else
        printf("pthread_create successful for service 3\n");

    // Service_4 = RT_MAX-4 — only for assignments where T is defined (period != 0)
    if (edf_thread_params[4].period != 0)
    {
#ifndef EDF
        rt_param[4].sched_priority = rt_max_prio - 4;
        pthread_attr_setschedparam(&rt_sched_attr[4], &rt_param[4]);
#endif
        rc = pthread_create(&threads[4],       // pointer to thread descriptor
                            &rt_sched_attr[4], // use specific attributes
                            //(void *)0,               // default attributes
                            Service_4,                 // thread function entry point
                            (void *)&(threadParams[4]) // parameters to pass in
        );
        if (rc < 0)
            perror("pthread_create for service 4");
        else
            printf("pthread_create successful for service 4\n");
    }
    else
        printf("Service_4 not used in this assignment (T=---)\n");

    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");
    threadParams[0].sequencePeriods = RTSEQ_PERIODS;

    // Sequencer = RT_MAX	@ 1000 Hz
    //
#ifndef EDF
    rt_param[0].sched_priority = rt_max_prio - 1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
#endif
    rc = pthread_create(&threads[0],       // pointer to thread descriptor
                        &rt_sched_attr[0], // use specific attributes
                        //(void *)0,               // default attributes
                        Sequencer,                 // thread function entry point
                        (void *)&(threadParams[0]) // parameters to pass in
    );
    if (rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequeencer service 0\n");

    for (i = 0; i < NUM_THREADS; i++)
    {
        if (i == 4 && edf_thread_params[4].period == 0)
            continue;
        pthread_join(threads[i], NULL);
    }

    printf("\nTEST COMPLETE\n");
    return 0;
}

void *Sequencer(void *threadp)
{
    struct timespec delay_time = {0, RTSEQ_DELAY_NSEC};
    struct timespec std_delay_time = {0, RTSEQ_DELAY_NSEC};
    struct timespec current_time_val = {0, 0};

    // struct timespec remaining_time;  // for use if we want to check for early wake-up and adjust delay accordingly, but this adds overhead so we are not using it in this example
    double current_time, last_time, scaleDelay; // scaleDelay is the adjustment to apply to the delay to control drift, but calculating it and adjusting the delay adds overhead, so we are not using it in this example
    double delta_t = (RTSEQ_DELAY_NSEC / (double)NANOSEC_PER_SEC);
    double scale_dt; // scaled delay adjustment for drift control
    int rc, delay_cnt = 0;
    unsigned long long seqCnt = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

// pthread_attr has no parameters for EDF, so we need to use the sched_attr structure and sched_setattr syscall to set EDF parameters if using EDF scheduling policy
#if defined(EDF)
    if (sched_setattr(0, &edf_attr[threadParams->threadIdx], 0) < 0)
    {
        perror("sched_setattr");
        pthread_exit((void *)-1);
    }
#endif
    current_time = getTimeMsec();
    last_time = current_time - delta_t;

    // syslog(LOG_CRIT, "RTSEQ: start on cpu=%d @ sec=%lf after %lf with dt=%lf\n", sched_getcpu(), current_time, last_time, delta_t);

    do
    {
        current_time = getTimeMsec();
        delay_cnt = 0;

#ifdef DRIFT_CONTROL
        scale_dt = (current_time - last_time) - delta_t;
        delay_time.tv_nsec = std_delay_time.tv_nsec - (scale_dt * (NANOSEC_PER_SEC + DT_SCALING_UNCERTAINTY_NANOSEC)) - CLOCK_BIAS_NANOSEC;
        // syslog(LOG_CRIT, "RTSEQ: scale dt=%lf @ sec=%lf after=%lf with dt=%lf\n", scale_dt, current_time, last_time, delta_t);
#else
        delay_time = std_delay_time;
        scale_dt = delta_t;
#endif

#ifdef ABS_DELAY
        clock_gettime(MY_CLOCK, &current_time_val);
        delay_time.tv_sec = current_time_val.tv_sec;
        delay_time.tv_nsec = current_time_val.tv_nsec + delay_time.tv_nsec;

        if (delay_time.tv_nsec > NANOSEC_PER_SEC)
        {
            delay_time.tv_sec = delay_time.tv_sec + 1;
            delay_time.tv_nsec = delay_time.tv_nsec - NANOSEC_PER_SEC;
        }
        // syslog(LOG_CRIT, "RTSEQ: cycle %08llu delay for dt=%lf @ sec=%d, nsec=%d to sec=%d, nsec=%d\n", seqCnt, scale_dt, current_time_val.tv_sec, current_time_val.tv_nsec, delay_time.tv_sec, delay_time.tv_nsec);
#endif

        // Delay loop with check for early wake-up
        do
        {
#ifdef ABS_DELAY
            rc = clock_nanosleep(MY_CLOCK, TIMER_ABSTIME, &delay_time, (struct timespec *)0);
#else
            rc = clock_nanosleep(MY_CLOCK, 0, &delay_time, &remaining_time);
#endif

            if (rc == EINTR)
            {
                syslog(LOG_CRIT, "RTSEQ: EINTR @ sec=%lf\n", current_time);
                delay_cnt++;
            }
            else if (rc < 0)
            {
                perror("RTSEQ: nanosleep");
                exit(-1);
            }

            // syslog(LOG_CRIT, "RTSEQ: WOKE UP\n");

        } while (rc == EINTR);

        // syslog(LOG_CRIT, "RTSEQ: cycle %08llu @ sec=%lf, last=%lf, dt=%lf, sdt=%lf\n", seqCnt + 1U, current_time, last_time, (current_time-last_time), scale_dt);

        // Release each service at a sub-rate of the generic sequencer rate

        // Servcie_1 = RT_MAX-1	@  50 Hz, A7: 33.33 HZ;
        if (rm_seq_divs.divisorService_1 != 0)
            if ((seqCnt % rm_seq_divs.divisorService_1) == 0)
                sem_post(&semS1);

        // Service_2 = RT_MAX-2	@ A1: 10 Hz; A2, A3: 20 Hz, A7: 16.67 HZ;
        if (rm_seq_divs.divisorService_2 != 0)
            if ((seqCnt % rm_seq_divs.divisorService_2) == 0)
                sem_post(&semS2);

        // Service_3 = RT_MAX-3	@ 6.67 Hz; A3, A5: 10 Hz; A4: 14.29, A7: 1.11 Hz
        if (rm_seq_divs.divisorService_3 != 0)
            if ((seqCnt % rm_seq_divs.divisorService_3) == 0)
                sem_post(&semS3);

        // Service_4 = RT_MAX-4	@ 5 Hz; A4: 7.69 Hz
        if (rm_seq_divs.divisorService_4 != 0)
            if ((seqCnt % rm_seq_divs.divisorService_4) == 0)
                sem_post(&semS4); // Assignment 3+ addition - release service 4 at the appropriate sub-rate

        seqCnt++;
        last_time = current_time;

    } while (!abortTest && (seqCnt < threadParams->sequencePeriods)); // run for a set number of cycles or until abort flag is set

    sem_post(&semS1);
    sem_post(&semS2);
    sem_post(&semS3);
    sem_post(&semS4);
    abortS1 = TRUE;
    abortS2 = TRUE;
    abortS3 = TRUE;
    abortS4 = TRUE;

    pthread_exit((void *)0);
}

void *Service_1(void *threadp)
{
    double current_time;
    unsigned long long S1Cnt = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
#if defined(EDF)
    if (sched_setattr(0, &edf_attr[threadParams->threadIdx], 0) < 0)
    {
        perror("sched_setattr");
        pthread_exit((void *)-1);
    }
#endif
    current_time = getTimeMsec();
    // syslog(LOG_CRIT, "S1: start on cpu=%d @ sec=%lf\n", sched_getcpu(), current_time);

    while (!abortS1)
    {
        sem_wait(&semS1);
        S1Cnt++;

        if (abortS1)
            break; // check abort flag after wake-up in case it was set while waiting, eg final cycle of the test when the sequencer releases the services to allow them to exit
        current_time = getTimeMsec();
        syslog(LOG_CRIT, COURSE_ASS_NUMBER_SCHED(COURSE, ASS), threadParams->threadIdx, S1Cnt, current_time, sched_getcpu());
        // syslog(LOG_CRIT, "S1: release %llu @ sec=%lf\n", S1Cnt, current_time);
        FIB_TEST(seqIterations, reqIterations, 1U);
    }

    pthread_exit((void *)0);
}

void *Service_2(void *threadp)
{
    double current_time;
    unsigned long long S2Cnt = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

#if defined(EDF)
    if (sched_setattr(0, &edf_attr[threadParams->threadIdx], 0) < 0)
    {
        perror("sched_setattr");
        pthread_exit((void *)-1);
    }
#endif

    current_time = getTimeMsec();
    // syslog(LOG_CRIT, "S2: start on cpu=%d @ sec=%lf\n", sched_getcpu(), current_time);

    while (!abortS2)
    {
        sem_wait(&semS2);
        S2Cnt++;

        if (abortS2)
            break; // check abort flag after wake-up in case it was set while waiting, eg final cycle of the test when the sequencer releases the services to allow them to exit
        current_time = getTimeMsec();
        syslog(LOG_CRIT, COURSE_ASS_NUMBER_SCHED(COURSE, ASS), threadParams->threadIdx, S2Cnt, current_time, sched_getcpu());
        // syslog(LOG_CRIT, "S2: release %llu @ sec=%lf\n", S2Cnt, current_time);
        FIB_TEST(2 *seqIterations, reqIterations, 2U);
    }

    pthread_exit((void *)0);
}

void *Service_3(void *threadp)
{
    double current_time;
    unsigned long long S3Cnt = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

#if defined(EDF)
    if (sched_setattr(0, &edf_attr[threadParams->threadIdx], 0) < 0)
    {
        perror("sched_setattr");
        pthread_exit((void *)-1);
    }
#endif

    current_time = getTimeMsec();
    // syslog(LOG_CRIT, "S3: start on cpu=%d @ sec=%lf\n", sched_getcpu(), current_time);

    while (!abortS3)
    {
        sem_wait(&semS3);
        S3Cnt++;

        if (abortS3)
            break; // check abort flag after wake-up in case it was set while waiting, eg final cycle of the test when the sequencer releases the services to allow them to exit
        current_time = getTimeMsec();
        syslog(LOG_CRIT, COURSE_ASS_NUMBER_SCHED(COURSE, ASS), threadParams->threadIdx, S3Cnt, current_time, sched_getcpu());
        // syslog(LOG_CRIT, "S3: release %llu @ sec=%lf\n", S3Cnt, current_time);
        FIB_TEST(3 *seqIterations, reqIterations, 3U);
    }

    pthread_exit((void *)0);
}

// relevant to Assignment 3+
void *Service_4(void *threadp)
{
    double current_time;
    unsigned long long S4Cnt = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

#if defined(EDF)
    if (sched_setattr(0, &edf_attr[threadParams->threadIdx], 0) < 0)
    {
        perror("sched_setattr");
        pthread_exit((void *)-1);
    }
#endif

    current_time = getTimeMsec();
    // syslog(LOG_CRIT, "S4: start on cpu=%d @ sec=%lf\n", sched_getcpu(), current_time);

    while (!abortS4)
    {
        sem_wait(&semS4);
        S4Cnt++;

        if (abortS4)
            break; // check abort flag after wake-up in case it was set while waiting, eg final cycle of the test when the sequencer releases the services to allow them to exit
        current_time = getTimeMsec();
        syslog(LOG_CRIT, COURSE_ASS_NUMBER_SCHED(COURSE, ASS), threadParams->threadIdx, S4Cnt, current_time, sched_getcpu());
        // syslog(LOG_CRIT, "S4: release %llu @ sec=%lf\n", S4Cnt, current_time);
        FIB_TEST( seqIterations, reqIterations, 4U);
    }

    pthread_exit((void *)0);
}

// global start_time must be set on first call
double getTimeMsec(void)
{
    struct timespec event_ts = {0, 0};
    double event_time = 0;

    clock_gettime(CLOCK_REALTIME, &event_ts);
    event_time = ((event_ts.tv_sec) + ((event_ts.tv_nsec) / (double)NANOSEC_PER_SEC));
    return (event_time - start_time);
}

void print_scheduler(void)
{
    int schedType, scope;

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
    default:
        printf("Pthread Policy is UNKNOWN\n");
        exit(-1);
    }

    pthread_attr_getscope(&main_attr, &scope);

    if (scope == PTHREAD_SCOPE_SYSTEM)
        printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
        printf("PTHREAD SCOPE PROCESS\n");
    else
        printf("PTHREAD SCOPE UNKNOWN\n");
}

void get_cpu_core_config(void)
{
    cpu_set_t cpuset;
    pthread_t callingThread;
    int rc, idx;

    CPU_ZERO(&cpuset);

    // get affinity set for main thread
    callingThread = pthread_self();

    // Check the affinity mask assigned to the thread
    rc = pthread_getaffinity_np(callingThread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0)
        perror("pthread_getaffinity_np");
    else
    {
        printf("thread running on CPU=%d, CPUs =", sched_getcpu());

        for (idx = 0; idx < CPU_SETSIZE; idx++)
            if (CPU_ISSET(idx, &cpuset))
                printf(" %d", idx);

        printf("\n");
    }

    printf("Using CPUS=%d from total available.\n", CPU_COUNT(&cpuset));
}
