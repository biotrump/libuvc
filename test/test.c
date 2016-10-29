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
#include <sys/time.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>

#ifdef USE_OPENCV
#include <opencv/highgui.h>
#endif

#include "libuvc/libuvc.h"

struct timespec ts0;
IplImage* cvImg;

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

void cb(uvc_frame_t *frame, void *usr_ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;

  	printf("%s:sequence=%d, tv_sec=%d, tv_nsec=%lu\n",__func__, 
		   frame->sequence, frame->capture_time.tv_sec, 
		  frame->capture_time.tv_nsec);

	static struct timespec tv0;
	struct timespec dt;
	float fps;
	if(tv0.tv_sec==0 && tv0.tv_nsec == 0){//init
		tv0 =frame->capture_time;
		memset(&dt,0,sizeof(dt));
		fps=0.0f;
	}else{
		dt = diff(tv0, frame->capture_time);
		fps = 1000000000l /(dt.tv_sec * 1000000000l + dt.tv_nsec);
		tv0=frame->capture_time;
	}
	printf("%s:tv_sec=%d, tv_nsec=%lu, fps=%.1f\n",__func__, 
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

#ifdef USE_OPENCV
  cvSetData(cvImg, bgr->data, bgr->width * 3);
  cvShowImage("Test", cvImg);
  cvWaitKey(5);
#endif	//USE_OPENCV
  
  uvc_free_frame(bgr);
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_error_t res;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;

  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device");
  } else {
    puts("Device found");

    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open");
    } else {
      puts("Device opened");

      uvc_print_diag(devh, stderr);

      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, UVC_FRAME_FORMAT_YUYV, 640, 480, 30/*fps*/
      );

      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode");
      } else {

	#ifdef USE_OPENCV
	    cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
		cvImg = cvCreateImageHeader(
			cvSize(640 /*bgr->width*/, /*bgr->height*/ 480),
			IPL_DEPTH_8U,
			3);
	#endif
		clock_gettime(CLOCK_REALTIME, &ts0);

		  res = uvc_start_streaming(devh, &ctrl, cb, 12345, 0);
		//printf("%s:width=%d, height=%d\n",__func__,
		//	   devh->streams->frame.width, devh->streams->frame.height);
		  

        if (res < 0) {
          uvc_perror(res, "start_streaming");
        } else {
          puts("Streaming for 10 seconds...");
          uvc_error_t resAEMODE = uvc_set_ae_mode(devh, 1);
          uvc_perror(resAEMODE, "set_ae_mode");
          int i;
          for (i = 1; i <= 10; i++) {
            /* uvc_error_t resPT = uvc_set_pantilt_abs(devh, i * 20 * 3600, 0); */
            /* uvc_perror(resPT, "set_pt_abs"); */
            uvc_error_t resEXP = uvc_set_exposure_abs(devh, 20 + i * 5);
            uvc_perror(resEXP, "set_exp_abs");
            
            sleep(1);
          }
          sleep(10);
          uvc_stop_streaming(devh);
	  puts("Done streaming.");
        }
      }

      uvc_close(devh);
      puts("Device closed");
    }

    uvc_unref_device(dev);
  }

  uvc_exit(ctx);
  puts("UVC exited");

#ifdef USE_OPENCV
    cvReleaseImageHeader(&cvImg);
#endif

	return 0;
}

