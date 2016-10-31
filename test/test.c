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
#include <opencv/cv.h>
IplImage* mainCVImg;
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
int *quit=NULL;
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

/** @brief : a hook to openCV or any display system
 * to show the process data.
 * @param[IN] cvimg: if null, mainCVImg, else convert to (IplImage *)
 * @param[IN] data : the frame data to show, like RGB  24bit pixel
 * @param[IN] stride : bytes per row.RGB is width *3 for 24bits
 */
char hookShowFrame(void *cvimg, uint8_t *data, int stride)
{
	char ch=0;

#ifdef USE_OPENCV
	IplImage* img= cvimg?(IplImage*)cvimg: mainCVImg;

	cvSetData(img, data, stride);
	cvShowImage("Test", img);
	ch = cvWaitKey(10);
#endif	//USE_OPENCV
  return ch;
}

/** @brief A necessary hook to process the uvc frame buffer
 * 
 */
int hookFrameProcess(PUVC_INFO_T my_uvc, UVC_BUFFER_T *uvcBuffer)
{
	int occupied=0, empty=0;
	uint8_t *frame=NULL;
	uint8_t *rgb=malloc(uvcBuffer->frame_width * uvcBuffer->frame_height * 
		  uvcBuffer->frame_stride);
	IplImage *yuyvImg = cvCreateImageHeader(
				cvSize(frame_width, frame_height),
				IPL_DEPTH_8U,2);
	IplImage *rgbImg = cvCreateImageHeader(
				cvSize(frame_width, frame_height),
				IPL_DEPTH_8U,3);
	cvSetData(rgbImg, rgb, uvcBuffer->frame_width*3);
	printf(">>%s thread start...., quit=%d\n",__func__, *quit);
	while (! *quit) {
#if 0
		sleep(1);
#else
		/////////////////////////////////////////////////////////
        // get a frame from the uvc frame buffer
        //frame=uvcBufDelete(quit);//get one frame
		//frame=uvcBufDelete_nonblocking(quit);//get one frame
		frame=UVCBUFRead(quit);
		printf("%s:frame=%p\n",__func__, frame);
		///////////////////////////////////////////////////////////
		
		//show it
		cvSetData(yuyvImg, frame, uvcBuffer->frame_width*2);
		cvCvtColor( yuyvImg, rgbImg, CV_YUV2BGR_YUYV);
		char ch = hookShowFrame(mainCVImg, rgb, uvcBuffer->frame_stride * uvcBuffer->frame_width);
		if(ch == 'q' || ch == 'Q' ||ch ==27){
			*quit=1;
			break;
		}
		//bcvframe engine
#endif
    }
exit:
	if(rgb)
		free(rgb);
	rgb=NULL;

	cvReleaseImageHeader(&yuyvImg);
	cvReleaseImageHeader(&rgbImg);
	printf("<<%s exit:quit=%d\n",__func__, *quit);
	*quit = 0;
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
	quit = &forceQuit;	//!!!MUST
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
		mainCVImg = cvCreateImageHeader(
				cvSize(frame_width, frame_height),
				IPL_DEPTH_8U,
				3);
	}else{
		mainCVImg = cvCreateImageHeader(
				cvSize(frame_width, frame_height),
				IPL_DEPTH_8U,
				1);
		myUVC.frame_stride=1;//8 bit grey
	}
#endif

	/** 
	 * main routine
	 */
	res = uvcProcess(&myUVC);
	if(res){
		printf("%s:uvcProcess fails = %d\n",__func__, res);
	}

#ifdef USE_OPENCV
    cvReleaseImageHeader(&mainCVImg);
#endif

	return 0;
}
