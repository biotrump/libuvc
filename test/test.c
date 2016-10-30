/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2010-2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#define _BSD_SOURCE	//implicit declaration of function usleep
#define _GNU_SOURCE             /* See feature_test_macros(7) */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <sys/time.h>

#ifdef USE_OPENCV
#include <opencv/highgui.h>
IplImage* cvImg;
#endif

#include "libuvc/libuvc.h"
#include "uvc_thd.h"

int frame_count;
int fixedInterval=0;
char format[20];
float fps=30.0f;
int InfraRed=0;
int repeat=1;
int pid=0;
int vid=0;
int verbose=0;
char output[256];
int frame_width=640;
int frame_height=480;
int frame_size;
int uvc_pix_fmt=UVC_FRAME_FORMAT_YUYV;//UVC_FRAME_FORMAT_GRAY8, UVC_FRAME_FORMAT_RGB,UVC_FRAME_FORMAT_UYVY
int v4l2_pix_fmt=V4L2_PIX_FMT_YUYV;//V4L2_PIX_FMT_GREY, 
int quit=0;
char sn[255];
UVC_INFO_T myUVC;

static const char short_options[] = "c:D:Ef:F:hIk:o:p:u:v:V";

static const struct option
long_options[] = {
	{ "count",  required_argument, NULL, 'c' },
	{ "windim", required_argument, NULL, 'D' },
	{ "fixedInterval", no_argument, NULL, 'E' },
	{ "format", required_argument,  NULL, 'f' },
	{ "fps",  		required_argument, NULL, 'F' },
	{ "help",   no_argument,       NULL, 'h' },
	{ "InfraRed",no_argument, NULL, 'I' },
	{ "repeat", required_argument, NULL, 'k' },
	{ "output", required_argument, NULL, 'o' },
	{ "pid", required_argument, NULL, 'p' },
	{ "uvcformat", required_argument,  NULL, 'u' },
	{ "vid", required_argument, NULL, 'v' },
	{ "verbose", 	no_argument,       NULL, 'V' },
	{ 0, 0, 0, 0 }
};

void errno_exit(const char *s)
{
	printf("+%s:%s error %d, %s\n", __func__, s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int option(int argc, char **argv)
{
	int r=0;
	printf("+%s:\n",__func__);

	for (;;) {
		int idx;
		int c;

		c = getopt_long(argc, argv,
				short_options, long_options, &idx);

		if (-1 == c)
			break;

		switch (c) {
		case 0: /* getopt_long() flag */
			break;

		case 'F':
			errno = 0;
			fps = strtof(optarg, NULL);
			printf("%s:fps=%.1f\n",__func__, fps);
			if (errno)
				errno_exit(optarg);
			break;
		case 'f':
			if(optarg && strlen(optarg)){
				printf("pixel fmt:%s\n",optarg);
				if(strncmp(optarg,"YUYV", strlen("YUYV"))==0 )
					v4l2_pix_fmt=V4L2_PIX_FMT_YUYV;
				else if(strncmp(optarg,"NV21",strlen("NV21"))==0 )
					v4l2_pix_fmt=V4L2_PIX_FMT_NV21;
				else if(strncmp(optarg,"GREY",strlen("GREY"))==0 )
					v4l2_pix_fmt=V4L2_PIX_FMT_GREY;
				else if(strncmp(optarg,"Y10",strlen("Y10"))==0 )
					v4l2_pix_fmt=V4L2_PIX_FMT_Y10;
				else if(strncmp(optarg,"Y12",strlen("Y12"))==0 )
					v4l2_pix_fmt=V4L2_PIX_FMT_Y12;
				else if(strncmp(optarg,"Y16",strlen("Y16"))==0 )
					v4l2_pix_fmt=V4L2_PIX_FMT_Y16;
			}
			printf("v4l2_pix_fmt=0x%x\n", v4l2_pix_fmt);
			break;

		case 'u':
			if(optarg && strlen(optarg)){
				printf("uvc fmt:%s\n",optarg);
				if(strncmp(optarg,"YUYV", strlen("YUYV"))==0 )
					uvc_pix_fmt=UVC_FRAME_FORMAT_YUYV;
				else if(strncmp(optarg,"UYVY", strlen("UYVY"))==0 )
					uvc_pix_fmt=UVC_FRAME_FORMAT_YUYV;
				else if(strncmp(optarg,"MJPEG",strlen("MJPEG"))==0 )
					uvc_pix_fmt=UVC_FRAME_FORMAT_MJPEG;
				else if(strncmp(optarg,"GREY8",strlen("GREY8"))==0 )
					uvc_pix_fmt=UVC_FRAME_FORMAT_GRAY8;
				else if(strncmp(optarg,"RGB",strlen("RGB"))==0 )
					uvc_pix_fmt=UVC_FRAME_FORMAT_RGB;
				else if(strncmp(optarg,"BGR",strlen("BGR"))==0 )
					uvc_pix_fmt=UVC_FRAME_FORMAT_BGR;
			}
			printf("uvc_pix_fmt=0x%x\n", uvc_pix_fmt);
			break;

		case 'D':
			if(optarg && strlen(optarg)){
				int i=0;
				printf("win dim:%s\n",optarg);
				while(optarg[i] && optarg[i] !='x') i++;
				if(optarg[i] =='x') {
					optarg[i]=' ';//delimeter
					sscanf(optarg,"%d %d", &frame_width, &frame_height);
					printf("frame_width=%d, frame_height=%d\n", frame_width, frame_height);
				}
			}
			break;
		case 'p':
			pid = strtol(optarg,NULL,16);
			printf("%s:pid %s=0x%x\n",__func__, optarg, pid);
			break;

		case 'v':
			vid=strtol(optarg,NULL,16);
			printf("%s:vid %s=0x%x\n",__func__, optarg, vid);
			break;

#if 0
		case 'c':
			errno = 0;
			frame_count = strtol(optarg, NULL, 0);
			if (errno)
				errno_exit(optarg);
			break;
		case 'E':
			fixedInterval=1;
			break;

		case 'k':
			errno = 0;
			repeat_count = strtol(optarg, NULL, 0);
			if (errno)
				errno_exit(optarg);
			break;

		case 'h':
			usage(stdout, argc, argv);
			r=-1;
			break;

		case 'I':
			enableInfraRed=1;
			//UVC, v4l2 default is YUYV422 for 8bit NIR GREY
			//v4l2_pix_fmt=V4L2_PIX_FMT_Y10;//NIR 10bits grey scale
			//v4l2_pix_fmt=V4L2_PIX_FMT_GREY;
			printf("IR mode enabled!\n");
			break;
		case 'o':
			errno = 0;
			strncpy(ofile_bcv, optarg, 1024);
			printf("%s\n", optarg);
			if (errno)
				errno_exit(optarg);
			break;

#endif

		case 'V':
			//setVerbose(1);
			break;

		default:
			//usage(stderr, argc, argv);
			r=-1;
		}
	}
	return r;
}

int uvcClose(PUVC_INFO_T my_uvc)
{
	uvc_context_t *ctx = my_uvc->ctx;
	uvc_device_handle_t *devh = my_uvc->devh;
	uvc_device_t *dev=my_uvc->dev;
	uvc_stream_ctrl_t ctrl=my_uvc->ctrl;

	printf("%s:Device closed\n",__func__);
	uvc_close(devh);

	uvc_unref_device(dev);

	uvc_exit(ctx);
	printf("%s:UVC exited\n",__func__);
}

int uvcStartCapture(PUVC_INFO_T my_uvc, uvc_frame_callback_t *cb, void *usr_ptr)
{
	uvc_device_handle_t *devh = my_uvc->devh;
	uvc_stream_ctrl_t ctrl=my_uvc->ctrl;
	int frame_size = my_uvc->frame_size;

	uvc_error_t res;
	res = uvc_start_streaming(devh, &ctrl, cb, /*12345*/usr_ptr, 0);
	//printf("%s:width=%d, height=%d\n",__func__,
	//	   devh->streams->frame.width, devh->streams->frame.height);
	if (res < 0) {
		uvc_perror(res, "start_streaming");
		return res;
	}
	res = uvcBufInit(my_uvc);//must behind uvc_start_streaming
	return res;
}

void uvcStopCapture(PUVC_INFO_T my_uvc)
{
	uvc_stop_streaming(my_uvc->devh);
	printf("%s:Done streaming.\n",__func__);

}

/**
 * uvcOpen
 * uvcStartCapture
 * 
 * vcStopCapture
 * uvcClose
 */
int uvcOpen(UVC_INFO_T *my_uvc) 
{
	uvc_error_t res;
	int pid = my_uvc-> pid;
	int vid = my_uvc->vid;
	char *sn = NULL;
	if(strlen(my_uvc->sn))
		sn = my_uvc->sn;

	res = uvc_init(&my_uvc->ctx, NULL);
	if (res < 0) {
		uvc_perror(res, "uvc_init");
		return res;
	}

	printf("%s:UVC initialized\n",__func__);

	res = uvc_find_device(
		my_uvc->ctx, &my_uvc->dev,
		my_uvc->vid, my_uvc->pid, /*NULL*/sn);
	if (res < 0) {
		uvc_perror(res, "uvc_find_device");
	} else {
		printf("%s:Device found\n",__func__);
		res = uvc_open(my_uvc->dev, &my_uvc->devh);

		if (res < 0) {
		uvc_perror(res, "uvc_open");
		} else {
		puts("Device opened");

		uvc_print_diag(my_uvc->devh, stderr);

		res = uvc_get_stream_ctrl_format_size(
			my_uvc->devh, &my_uvc->ctrl, my_uvc->uvc_pix_fmt/*UVC_FRAME_FORMAT_YUYV*/, 
			my_uvc->frame_width, my_uvc->frame_height, my_uvc->fps/*30*/
		);

		uvc_print_stream_ctrl(&my_uvc->ctrl, stderr);
		printf("res=%d\n", res);

      if (res < 0) {
        uvc_perror(res, "get_mode");
      } else {

		//clock_gettime(CLOCK_REALTIME, &ts0);
		switch(my_uvc->uvc_pix_fmt){
			case UVC_FRAME_FORMAT_YUYV:
			case UVC_FRAME_FORMAT_UYVY:
				my_uvc->frame_size = frame_width*frame_height*2;
				break;
			case UVC_FRAME_FORMAT_GRAY8:
			default:
				my_uvc->frame_size = frame_width*frame_height;
				break;
		}
	  }
		}
	}
	return res;
}

/**
 * 
 */
char ShowFrame(uint8_t *data, int stride)
{
	char ch=0;
#ifdef USE_OPENCV
  cvSetData(cvImg, data, stride);
  cvShowImage("Test", cvImg);
  ch = cvWaitKey(20);
#endif	//USE_OPENCV
  return ch;
}

/**
 * 
 */
int frameProcess(PUVC_INFO_T my_uvc, UVC_BUFFER_T *uvcBuffer)
{
	int occupied=0, empty=0;
	uint8_t *frame=NULL;

	printf(">>%s thread start...., quit=%d\n",__func__, quit);
	while (!quit) {
        frame=uvcBufDelete(&quit);//get one frame
		printf("%s:frame=%p\n",__func__, frame);
		//show it
		char ch = ShowFrame(frame, uvcBuffer->frame_stride * uvcBuffer->frame_width);
		if(ch == 'q' || ch == 'Q' ||ch ==27){
			quit=1;
			break;
		}
    }
exit:
    printf("<<%s exit:quit=%d\n",__func__, quit);
	quit = 0;
}

int main(int argc, char **argv) 
{
	uvc_error_t res;
	option(argc, argv);
	
	
	///////////////////////////////////////////////////////
	int priority=0;
	int	policy=/*SCHED_OTHER*/ /*SCHED_RR*/ SCHED_FIFO;//real time
	pthread_attr_t tattr;
	struct sched_param param;
	printf("%s:SCHED_FIFO=%d, SCHED_RR=%d, SCHED_OTHER=%d\n",__func__, SCHED_FIFO,
		SCHED_RR, SCHED_OTHER);
#if 0
  	/* setting the new scheduling param */
	ret = pthread_attr_init (&tattr);
	ret = pthread_attr_getschedpolicy(&tattr, &policy);
	priority = sched_get_priority_max(policy);
	printf("%s:current policy=%d, priority=%d\n",__func__, policy, priority);
	//setup the thread attr before create the thread
	policy=SCHED_FIFO;//set realtime priority
	ret= pthread_attr_setschedpolicy(&tattr, policy);
	printf("%s:pthread_attr_setschedpolicy ret=%d, policy=%d\n",__func__, ret, policy);
	/* safe to get existing scheduling param */
	ret = pthread_attr_getschedparam (&tattr, &param);//tattr->param
	/* set the priority; others are unchanged */
	param.sched_priority = 1;	//1-99
	/* setting the new scheduling param */
	ret = pthread_attr_setschedparam (&tattr, &param);//param->tattr
#endif
	//////////////////////////////////////////////////////////////////
  	int ret = pthread_getschedparam(pthread_self(), &policy, &param);
	printf("%s:current ret=%d, policy=%d, param.sched_priority=%d\n",__func__, ret,
		   policy, param.sched_priority);
	policy = SCHED_FIFO;
	param.sched_priority=1;//1-99, 99 is the less 
	ret = pthread_setschedparam(pthread_self(), policy, &param);
	printf(">>%s:ret=%d, policy=%d, param.sched_priority=%d\n",__func__, ret,policy, 
		   param.sched_priority);
	ret = pthread_getschedparam(pthread_self(), &policy, &param);
	printf("<<<%s:current ret=%d, policy=%d, param.sched_priority=%d\n",__func__, ret,
		   policy, param.sched_priority);
	////////////////////////////////////

	memset(&myUVC, 0 , sizeof(myUVC) );
 	myUVC.pid = pid;
	myUVC.vid = vid;

	strncpy(myUVC.sn,sn, 30);
	
	myUVC.fps= fps;
	myUVC.uvc_pix_fmt=uvc_pix_fmt;
	myUVC.v4l2_pix_fmt=v4l2_pix_fmt;
	myUVC.frame_width = frame_width;
	myUVC.frame_height = frame_height;

#ifdef USE_OPENCV
	cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
	if(uvc_pix_fmt == UVC_FRAME_FORMAT_YUYV){
		myUVC.frame_stride=3;	//RGB display
		cvImg = cvCreateImageHeader(
				cvSize(frame_width, frame_height),
				IPL_DEPTH_8U,
				3);
	}else{
		cvImg = cvCreateImageHeader(
				cvSize(frame_width, frame_height),
				IPL_DEPTH_8U,
				1);
		myUVC.frame_stride=1;//8 bit grey
	}
#endif
	/////
	res = uvcOpen(&myUVC);
	if(res){
		printf("%s:uvcOpen fails = %d\n",__func__, res);
		goto exit;
	}
	///start_streaming and callback in cb()
	res=uvcStartCapture(&myUVC, callback, (void *)12345);
	if (res < 0) {
		uvc_perror(res, "start_streaming");
		goto exit;
	} 

	//uvc_error_t resAEMODE = uvc_set_ae_mode(devh, 1);
	//uvc_perror(resAEMODE, "set_ae_mode");

	/* uvc_error_t resPT = uvc_set_pantilt_abs(devh, i * 20 * 3600, 0); */
	/* uvc_perror(resPT, "set_pt_abs"); */
	//uvc_error_t resEXP = uvc_set_exposure_abs(devh, 20 + i * 5);
	//uvc_perror(resEXP, "set_exp_abs");

	frameProcess(&myUVC, &bcvUVCBuffer);//process the uvc frame
	
	uvcStopCapture(&myUVC);//stop

exit:
	uvcClose(&myUVC);

#ifdef USE_OPENCV
    cvReleaseImageHeader(&cvImg);
#endif

	return 0;
}
