#ifndef VERSION1_0_TRANSDATA_H
#define VERSION1_0_TRANSDATA_H

#include <iostream>
#include <thread>
#include <mutex>
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/pixdesc.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
#include <libavutil/avassert.h>
#include <libavutil/imgutils.h>

};
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
//enum AVPixelFormat get_hw_format(AVCodecContext *ctx,
//                                        const enum AVPixelFormat *pix_fmts);

//typedef enum AVPixelFormat (*get_format_fun)(struct AVCodecContext *, const enum AVPixelFormat *);

class Transdata
{
    AVFormatContext *ifmt_ctx = NULL;
    AVPacket pkt;
    int videoindex=-1;
    AVCodecContext  *pCodecCtx;
    AVCodec         *pCodec;
    AVCodecParameters *codecpar = NULL;
	static	enum AVPixelFormat hw_pix_fmt;
	AVBufferRef *hw_device_ctx = NULL;
    const char *in_filename = "rtsp://192.168.10.92:554/live";
    //const char *in_filename = "rtsp://192.168.1.105:554/1080.264";
    const char *out_filename_v = "test1.h264";				//Output file URL
	mutex   mImage_buf;
	static	enum AVPixelFormat get_hw_format(AVCodecContext *ctx,
                                        const enum AVPixelFormat *pix_fmts);
	int hw_decoder_init(AVCodecContext *ctx, const enum AVHWDeviceType type);
	int decode_write(AVCodecContext *avctx, AVPacket *packet);
public:
	cv::Mat image_test;
    Transdata();
    ~Transdata();
    int Transdata_init();
    int Transdata_Recdata();
    int Transdata_free();
};
#endif //VERSION1_0_TRANSDATA_H
