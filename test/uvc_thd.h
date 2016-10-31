#ifndef _H_UVC_CAPTURE_THREAD_H
#define	_H_UVC_CAPTURE_THREAD_H

#ifdef __cplusplus
extern "C"{
#endif

#define	UVC_CAPTURE_NON_BLOCK	(1)

/** UP to 30 seconds @640*480*30fps == 276,480,000 bytes
 * 
 */

#define	UVC_CAPTURE_BUFFER_FRAMES	(30*30)
#define	UVC_CAPTURE_FRAME_WIDTH		(640)
#define	UVC_CAPTURE_FRAME_HEIGHT	(480)
#define	UVC_CAPTURE_BUFFER_SIZE		(UVC_CAPTURE_FRAME_WIDTH*UVC_CAPTURE_FRAME_HEIGHT*UVC_CAPTURE_BUFFER_FRAMES)

typedef struct _uvc_cq_t
{
	uint16_t magic;		//0x55aa
    uint8_t *data;   /* cqueue var */
    int frame_size;
	int frame_stride;
	int frame_width;
	int frame_height;
    int tail;               /* buf[tail%BUFF_SIZE] is the first empty slot */
    int head;              /* buf[head%BUFF_SIZE] is the first occupied slot */
    int max_frames;
	int cur_frames;
    sem_t occupied;       /* keep track of the number of occupied spots */
    sem_t empty;          /* keep track of the number of empty spots */
    pthread_mutex_t mutex;          /* enforce mutual exclusion to cqueue data */
    pthread_t tid;
} UVC_BUFFER_T, *PUVC_BUFFER;

typedef struct _uvc_info_t{
	uvc_context_t *ctx;
	uvc_device_t *dev;
	uvc_device_handle_t *devh;
	uvc_stream_ctrl_t ctrl;
	int vid;
	int pid;
	char sn[30];
	int uvc_pix_fmt;
	int v4l2_pix_fmt;
	int frame_stride;
	int frame_width;
	int frame_height;
	int frame_size;
	float fps;
} UVC_INFO_T, *PUVC_INFO_T;

/** @brief : a hook to openCV or any display system
 * to show the process data.
 * @param[IN] cvimg: if null, mainCVImg, else convert to (IplImage *)
 * @param[IN] data : the frame data to show, like RGB  24bit pixel
 * @param[IN] stride : bytes per row.RGB is width *3 for 24bits
 */
int hookFrameProcess(PUVC_INFO_T my_uvc, UVC_BUFFER_T *uvcBuffer);
/** @brief  uvc process until terminates 
 * 
 */
int uvcProcess(UVC_INFO_T* myuvc);

char hookShowFrame(void *cvimg, uint8_t *data, int size);//debug to show the frame
void callback(uvc_frame_t *frame, void *usr_ptr);

int uvcBufInit(PUVC_INFO_T my_uvc);
uint8_t *uvcBufInsert(uint8_t *in ,int *quit);
uint8_t *uvcBufInsert_nonblocking(uint8_t *in, int *quit);

uint8_t *uvcBufDelete(int *quit);
uint8_t *uvcBufDelete_nonblocking(int *quit);
#define	UVCBUFRead(q)	uvcBufDelete_nonblocking(q)

/**
 * if head == 1, get uvc buffer head address
 * if head == 0, get uvc buffer tail address
 */
uint8_t *uvcBufAt(int head);
int uvcBufReset(void);
int uvcBufDeInit(void);

extern UVC_BUFFER_T bcvUVCBuffer;
extern int forceQuit;

#ifdef __cplusplus
}
#endif

#endif	//_H_UVC_CAPTURE_THREAD_H