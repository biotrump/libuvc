#define _BSD_SOURCE	//implicit declaration of function usleep
#define _GNU_SOURCE             /* See feature_test_macros(7) */

#include <stdio.h>
#include <stdlib.h>
//#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>

#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"

#include "uvc_thd.h"

UVC_BUFFER_T bcvUVCBuffer;

/** @brief
 * ret=-1, errno=110, ETIMEDOUT==110
 */
int sem_timedwait_millsecs(sem_t *sem, long msecs)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    long secs = msecs/1000;
    msecs = msecs%1000;

    long add = 0;
    msecs = msecs*1000*1000 + ts.tv_nsec;
    add = msecs / (1000*1000*1000);
    ts.tv_sec += (add + secs);
    ts.tv_nsec = msecs%(1000*1000*1000);
	int ret=sem_timedwait(sem, &ts);
	//printf("%s:ret=%d, errno=%d, ETIMEDOUT=%d\n",__func__, ret, errno, ETIMEDOUT);
    return ret;
}

/** @brief
 * ret=-1, errno=110, ETIMEDOUT==110
 */
int sem_timedwait_secs(sem_t *sem, int secs)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += secs;

	int ret=sem_timedwait(sem, &ts);
	printf("%s:ret=%d, errno=%d, ETIMEDOUT=%d\n",__func__, ret, errno, ETIMEDOUT);
    return ret;
}

/** @brief
 * ret=-1, errno=110, ETIMEDOUT==110
 */
int pthread_mutex_timedlock_millsecs(pthread_mutex_t *mutex, long msecs)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    long secs = msecs/1000;
    msecs = msecs%1000;

    long add = 0;
    msecs = msecs*1000*1000 + ts.tv_nsec;
    add = msecs / (1000*1000*1000);
    ts.tv_sec += (add + secs);
    ts.tv_nsec = msecs%(1000*1000*1000);
	int ret=pthread_mutex_timedlock(mutex, &ts);

	//printf("%s:ret=%d, errno=%d, ETIMEDOUT=%d\n",__func__, ret, errno, ETIMEDOUT);
    return ret;
}

/** @brief
 * ret=-1, errno=110, ETIMEDOUT==110
 */
int pthread_mutex_timedlock_secs(pthread_mutex_t *mutex, int secs)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += secs;

	int ret=pthread_mutex_timedlock(mutex, &ts);

	printf("%s:ret=%d, errno=%d, ETIMEDOUT=%d\n",__func__, ret, errno, ETIMEDOUT);
    return ret;
}

struct timespec ts0;
struct timespec diff(struct timespec start, struct timespec end)
{
	struct timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

struct timeval diff_tval(struct timeval start, struct timeval end)
{
	struct timeval temp;
	if ((end.tv_usec-start.tv_usec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_usec = 1000000+end.tv_usec-start.tv_usec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_usec = end.tv_usec-start.tv_usec;
	}
	return temp;
}

#if 0
struct timeval {
	time_t		tv_sec;		/* seconds */
	suseconds_t	tv_usec;		/* microseconds */
};

 * enum uvc_frame_format frame_format;
  /** Number of bytes per horizontal line (undefined for compressed format) */
  size_t step;
  /** Frame number (may skip, but is strictly monotonically increasing) */
  uint32_t sequence;
  /** Estimate of system time when the device started capturing the image */
  struct timeval capture_time;
  
res = uvc_start_streaming(devh, &ctrl, cb, 12345, 0);
cb usr_ptr is from 12345 of uvc_start_streaming 

#endif

/**
 * 
 * 
 */

void callback(uvc_frame_t *frame, void *usr_ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;

//  	printf(">>%s:sequence=%d, tv_sec=%d, tv_nsec=%lu\n",__func__, 
//		   frame->sequence, frame->capture_time.tv_sec, 
//		  frame->capture_time.tv_nsec);

	static struct timespec tv0;
	struct timespec dt;
	float fps;
	if(tv0.tv_sec==0 && tv0.tv_nsec == 0){//init
		tv0 =frame->capture_time;
		memset(&dt,0,sizeof(dt));
		fps=0.0f;
	}else{
		dt = diff(tv0, frame->capture_time);
		fps = 1000000000.0f /(dt.tv_sec * 1000000000.0f + dt.tv_nsec);
		tv0=frame->capture_time;
	}
	printf(">>%s:delta tv_sec=%d, tv_nsec=%lu, fps=%.1f\n",__func__, 
		   dt.tv_sec , dt.tv_nsec, fps);
	
#if 0	
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
	struct timespec delt = diff(ts0, ts);
	ts0 = ts;
	printf("%s:tv_sec=%d, tv_nsec=%d, fps=%.1f\n",__func__, 
		   delt.tv_sec , delt.tv_nsec, 1.0E9/delt.tv_nsec);

#endif
  //printf("callback! length = %u, ptr = %d\n", frame->data_bytes, (int) ptr);
	uvcBufInsert(frame->data);

  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!");
    return;
  }

  ret = uvc_any2bgr(frame, bgr);
  if (ret) {
    uvc_perror(ret, "uvc_any2bgr");
    uvc_free_frame(bgr);
    return;
  }

  ShowFrame(bgr->data, bgr->width * 3);
  
  uvc_free_frame(bgr);
}

int uvcBufInit(PUVC_INFO_T my_uvc)
{
	uvc_device_handle_t *devh=my_uvc->devh;
	printf(">>%s:devh=%p\n",__func__, devh);
	if(devh==NULL){
		printf("%s:devh NULL\n",__func__);
		return -1;
	}
	if(devh->streams==NULL){
		printf("%s:devh->streams NULL\n",__func__);
		return -1;
	}
	memset(&bcvUVCBuffer, 0, sizeof(UVC_BUFFER_T) );
	bcvUVCBuffer.data = realloc( bcvUVCBuffer.data, UVC_CAPTURE_BUFFER_SIZE ) ;
	if(bcvUVCBuffer.data == NULL){
		printf("%s: malloc error @%s:line#%d\n",__func__,__FILE__, __LINE__);
		//assert(0);
	}
	bcvUVCBuffer.frame_size = my_uvc->frame_size;
	bcvUVCBuffer.frame_width = my_uvc->frame_width;
	bcvUVCBuffer.frame_height = my_uvc->frame_height;
	bcvUVCBuffer.max_frames = UVC_CAPTURE_BUFFER_SIZE / my_uvc->frame_size;
	bcvUVCBuffer.frame_stride=my_uvc->frame_stride;

	sem_init(&bcvUVCBuffer.occupied, 0, 0);
	sem_init(&bcvUVCBuffer.empty, 0, bcvUVCBuffer.max_frames);//video stream number
	pthread_mutex_init(&bcvUVCBuffer.mutex, NULL);

	//bcvUVCBuffer.tid = devh->streams->cb_thread;

	bcvUVCBuffer.magic=0x55aa;
	//printf("<<%s:tid=%d\n",__func__, bcvUVCBuffer.tid);
	return 0;
}

uint8_t *uvcBufInsert(uint8_t *in /*, int size*/)
{
	int at;
	int empty=0, occupied=0;

	if(bcvUVCBuffer.magic != 0x55aa){
		printf("!!!wrong bcvUVCBuffer.magic=0x%x \n", bcvUVCBuffer.magic);
		return NULL;
	}
	sem_getvalue(&bcvUVCBuffer.empty, &empty);
	sem_getvalue(&bcvUVCBuffer.occupied, &occupied);

	printf(">>%s:empty=%d, occupied=%d\n",__func__,empty,occupied);
	sem_wait(&bcvUVCBuffer.empty);//Semaphore P op, --bcvUVCBuffer->empty

	sem_getvalue(&bcvUVCBuffer.empty, &empty);
	sem_getvalue(&bcvUVCBuffer.occupied, &occupied);
	printf(">>>%s:empty=%d, occupied=%d\n",__func__,empty,occupied);

	/* If another thread uses the buffer, wait */
	pthread_mutex_lock(&bcvUVCBuffer.mutex);
	at = bcvUVCBuffer.tail%bcvUVCBuffer.max_frames;
	printf("%s:at=%d\n",__func__, at);
	bcvUVCBuffer.tail = (bcvUVCBuffer.tail+1)%bcvUVCBuffer.max_frames;
	pthread_mutex_unlock(&bcvUVCBuffer.mutex);
	sem_post(&bcvUVCBuffer.occupied);
	//printf("%s:5\n",__func__);
	
	sem_getvalue(&bcvUVCBuffer.empty, &empty);
	sem_getvalue(&bcvUVCBuffer.occupied, &occupied);
	printf("<<<%s:empty=%d, occupied=%d\n",__func__,empty,occupied);

	memcpy(bcvUVCBuffer.data + at*bcvUVCBuffer.frame_size , in,  
		   bcvUVCBuffer.frame_size);
	return bcvUVCBuffer.data + at*bcvUVCBuffer.frame_size;
}

/**
 * get a frame bufer address from the uvc frame buffer
 */
uint8_t *uvcBufDelete(int *quit)
{
	if(bcvUVCBuffer.magic != 0x55aa){
		printf("!!!wrong bcvUVCBuffer.magic=0x%x \n", bcvUVCBuffer.magic);
		return NULL;
	}
	int empty,occupied;
	int at;

	printf(">>>%s:frame_size=%d\n",__func__,bcvUVCBuffer.frame_size);
	sem_getvalue(&bcvUVCBuffer.empty, &empty);
	sem_getvalue(&bcvUVCBuffer.occupied, &occupied);
	printf("%s:empty=%d, occupied=%d\n",__func__,empty,occupied);
	
	//sem_wait(&bcvUVCBuffer.occupied);//Semaphore P op, --bcvUVCBuffer->empty
	while( sem_timedwait_secs(&bcvUVCBuffer.occupied, 2) ){
		if(errno == ETIMEDOUT)
			if(quit){
				if(*quit){
					printf(">>%s:sem quit:%d\n",__func__, *quit);
					goto exit;
				}
			}
	}

	//pr_debug(DSP_INFO,">>%s: sem_wait entered and mutex waiting...\n",__func__);
	/* If another thread uses the buffer, wait */
	//pthread_mutex_lock(&bcvUVCBuffer.mutex);
	while( pthread_mutex_timedlock_secs(&bcvUVCBuffer.mutex, 2) ){
		if(errno == ETIMEDOUT)
			if(quit){
				if(*quit){
					printf(">>%s:mutex quit:%d\n",__func__, *quit);
					goto exit;
				}
			}
	}

	sem_getvalue(&bcvUVCBuffer.empty, &empty);
	sem_getvalue(&bcvUVCBuffer.occupied, &occupied);
	printf("<<<%s:empty=%d, occupied=%d\n",__func__,empty,occupied);

	at = bcvUVCBuffer.head%bcvUVCBuffer.max_frames;
	bcvUVCBuffer.head = (bcvUVCBuffer.head+1)%bcvUVCBuffer.max_frames;
	pthread_mutex_unlock(&bcvUVCBuffer.mutex);
	sem_post(&bcvUVCBuffer.empty);

exit:
	printf("%s:at=%d,0x%x \n",__func__, at, at*bcvUVCBuffer.frame_size);
	return bcvUVCBuffer.data + at*bcvUVCBuffer.frame_size;	
}

/**
 * if head == 1, get uvc buffer head address
 * if head == 0, get uvc buffer tail address
 */
uint8_t *uvcBufAt(int head)
{
	int at;
	if(bcvUVCBuffer.magic != 0x55aa){
		printf("!!!wrong bcvUVCBuffer.magic=0x%x \n", bcvUVCBuffer.magic);
		return NULL;
	}
	pthread_mutex_lock(&bcvUVCBuffer.mutex);
	if(head)
		at = bcvUVCBuffer.head%bcvUVCBuffer.max_frames;
	else
		at = bcvUVCBuffer.tail%bcvUVCBuffer.max_frames;
	pthread_mutex_unlock(&bcvUVCBuffer.mutex);

	return bcvUVCBuffer.data + at*bcvUVCBuffer.frame_size;
}

int uvcBufReset(void)
{
	//memset(&bcvUVCBuffer, 0, sizeof(UVC_BUFFER_T) );
	sem_init(&bcvUVCBuffer.occupied, 0, 0);
	sem_init(&bcvUVCBuffer.empty, 0, bcvUVCBuffer.max_frames);//video stream number
	pthread_mutex_init(&bcvUVCBuffer.mutex, NULL);
	bcvUVCBuffer.magic = 0x55aa;
	return 0;	
}

int uvcBufDeInit(void)
{
	if(bcvUVCBuffer.magic != 0x55aa){
		printf("!!!wrong bcvUVCBuffer.magic=0x%x \n", bcvUVCBuffer.magic);
		return -1;
	}

	if(bcvUVCBuffer.data){
		free(bcvUVCBuffer.data) ;
		bcvUVCBuffer.data=NULL;
	}

	pthread_mutex_destroy(&bcvUVCBuffer.mutex);
	sem_destroy(&bcvUVCBuffer.occupied);
	sem_destroy(&bcvUVCBuffer.empty);
	return 0;
}
