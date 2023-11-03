#include "transdata.h"
#include "ros/ros.h"
Transdata::Transdata(){}
Transdata::~Transdata(){}

enum AVPixelFormat Transdata::hw_pix_fmt = AV_PIX_FMT_NV12;


int Transdata::Transdata_free()
{
	int	ret=-1;
    avformat_close_input(&ifmt_ctx);
    if (ret < 0 && ret != AVERROR_EOF)
    {
        printf( "Error occurred.\n");
        return -1;
    }
    return 0;
}

int Transdata::decode_write(AVCodecContext *avctx, AVPacket *packet)
{
    AVFrame *frame = NULL, *sw_frame = NULL;
    AVFrame *tmp_frame = NULL;
    uint8_t *buffer = NULL;
    int size;
    int ret = 0;
    //cv::Mat img;
	struct	SwsContext* swsContext;
	int linesize[4];
	int num_bytes;
	uint8_t*	p_global_bgr_buffer;
	uint8_t*  	bgr_buffer[4];
	int	Format=0;
    ret = avcodec_send_packet(avctx, packet);
    if (ret < 0) 
	{
        fprintf(stderr, "Error during decoding\n");
        return ret;
    }

    while (1) 
	{
        if (!(frame = av_frame_alloc()) || !(sw_frame = av_frame_alloc())) 
		{
            fprintf(stderr, "Can not alloc frame\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        ret = avcodec_receive_frame(avctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) 
		{
            av_frame_free(&frame);
            av_frame_free(&sw_frame);
            return 0;
        } 
		else if (ret < 0) 
		{
            fprintf(stderr, "Error while decoding\n");
            goto fail;
        }

        if (frame->format == hw_pix_fmt) 
		{
            /* retrieve data from GPU to CPU */
            if ((ret = av_hwframe_transfer_data(sw_frame, frame, 0)) < 0) 
			{
                fprintf(stderr, "Error transferring the data to system memory\n");
                goto fail;
            }
            tmp_frame = sw_frame;
        } 
		else
            tmp_frame = frame;
#if	0
        size = av_image_get_buffer_size((enum AVPixelFormat)tmp_frame->format, tmp_frame->width,	\
                                        tmp_frame->height, 1);
		//printf("Size is %d. format is %d\n",size,tmp_frame->format);
        buffer = (uint8_t *)av_malloc(size);
        if (!buffer) {
            fprintf(stderr, "Can not alloc buffer\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
		
        ret = av_image_copy_to_buffer(buffer, size,			\
                                      (const uint8_t * const *)tmp_frame->data,			\
                                      (const int *)tmp_frame->linesize, (enum AVPixelFormat)tmp_frame->format,	\
                                      tmp_frame->width, tmp_frame->height, 1);
        if (ret < 0) {
            fprintf(stderr, "Can not copy image to buffer\n");
            goto fail;
        }
#endif
#if	1	
		//img = cv::Mat::zeros(tmp_frame->height, tmp_frame->width, CV_8UC3);
		mImage_buf.lock();
		image_test=cv::Mat::zeros(tmp_frame->height, tmp_frame->width, CV_8UC3);
		swsContext = sws_getContext(tmp_frame->width, tmp_frame->height,(enum AVPixelFormat)tmp_frame->format,	\
			tmp_frame->width, tmp_frame->height, AV_PIX_FMT_BGR24,SWS_FAST_BILINEAR, NULL, NULL, NULL);

		linesize[0] = tmp_frame->linesize[0] * 3;
		linesize[1] = tmp_frame->linesize[1] * 3;
		linesize[2] = tmp_frame->linesize[2] * 3;
		linesize[3] = tmp_frame->linesize[3] * 3;
		num_bytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, tmp_frame->width,	\
		                                   tmp_frame->height, 1);
		p_global_bgr_buffer = (uint8_t*) malloc(num_bytes * sizeof(uint8_t));
		av_image_fill_pointers(bgr_buffer,(enum AVPixelFormat)tmp_frame->format,	\
			tmp_frame->height,p_global_bgr_buffer,linesize);
		
		sws_scale(swsContext,tmp_frame->data,tmp_frame->linesize, 0, tmp_frame->height, bgr_buffer, linesize);
		memcpy(image_test.data,p_global_bgr_buffer,num_bytes);
		mImage_buf.unlock();
		//namedWindow("test", WINDOW_NORMAL);
		//imshow("test",image_test);
 		//waitKey(1);
		//printf("write data is %d.\n",num_bytes);
#endif
		#if	0
        if ((ret = fwrite(buffer, 1, size, output_file)) < 0) {
            fprintf(stderr, "Failed to dump raw data.\n");
            goto fail;
        }
		#endif

    fail:
        av_frame_free(&frame);
        av_frame_free(&sw_frame);
	sws_freeContext(swsContext);
    	free(p_global_bgr_buffer);
        if (ret < 0)
            return ret;
    }
}


int Transdata::Transdata_Recdata()
{
	int	ret;
    if(av_read_frame(ifmt_ctx, &pkt)<0) 
    {
        return -1;
    }
    //printf("pkt.stream_index==%d.\n",pkt.stream_index);
    if (videoindex == pkt.stream_index)
            ret = decode_write(pCodecCtx, &pkt);
    av_packet_unref(&pkt);
    return 0;
}



enum AVPixelFormat Transdata::get_hw_format(AVCodecContext *ctx,
                                        const enum AVPixelFormat *pix_fmts)
{
    const	enum AVPixelFormat *p;
    for (p = pix_fmts; *p != -1; p++) 
	{
        if (*p == hw_pix_fmt)
            return *p;
    }

    fprintf(stderr, "Failed to get HW surface format.\n");
    return AV_PIX_FMT_NONE;
}


int Transdata::hw_decoder_init(AVCodecContext *ctx, const enum AVHWDeviceType type)
{
    int err = 0;
    if ((err = av_hwdevice_ctx_create(&hw_device_ctx, type,
                                      NULL, NULL, 0)) < 0) {
        fprintf(stderr, "Failed to create specified HW device.\n");
        return err;
    }
    ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
    return err;
}


int Transdata::Transdata_init() 
{
	int	ret,i;
	enum AVHWDeviceType type;
    AVDictionary *format_opts = NULL;
	AVStream *video = NULL;
    av_dict_set(&format_opts, "stimeout", std::to_string( 5* 1000000).c_str(), 0); //è®¾ç½®?¾æŽ¥è??¶æ—¶é—´ï?usï¼‰
    av_dict_set(&format_opts, "rtsp_transport",  "tcp", 0); //è®¾ç½®?¨æ??„?¹å?ï¼Œé»˜è®?dp?‚
    av_dict_set(&format_opts, "buffer_size", "10240000", 0);
    //Register
    //av_register_all();
    
    //Network
    avformat_network_init();
	type = av_hwdevice_find_type_by_name("cuda");
    if (type == AV_HWDEVICE_TYPE_NONE) 
	{
        //fprintf(stderr, "Device type %s is not supported.\n", argv[1]);
        fprintf(stderr, "Available device types:");
        while((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE)
            fprintf(stderr, " %s", av_hwdevice_get_type_name(type));
        fprintf(stderr, "\n");
        return -1;
    }
    //Input
    //if ((ret = avformat_open_input(&ifmt_ctx, in_filename, 0, 0)) < 0) {
    if ((ret = avformat_open_input(&ifmt_ctx, in_filename, 0, &format_opts)) < 0) {
        ROS_ERROR("Could not open input file.");
        return -1;
    }
	av_dict_free(&format_opts);
    if ((ret = avformat_find_stream_info(ifmt_ctx, 0)) < 0) {
        printf("Failed to retrieve input stream information");
        return -1;
    }
	/* find the video stream information */
    ret = av_find_best_stream(ifmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &pCodec, 0);
    if (ret < 0) 
	{
        fprintf(stderr, "Cannot find a video stream in the input file\n");
        return -1;
    }
	videoindex = ret;
	
	for (i = 0;; i++) 
	{
        const AVCodecHWConfig *config = avcodec_get_hw_config(pCodec, i);
        if (!config) {
            fprintf(stderr, "Decoder %s does not support device type %s.\n",
                    pCodec->name, av_hwdevice_get_type_name(type));
            return -1;
        }
        if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
            config->device_type == type) {
            hw_pix_fmt = config->pix_fmt;
            break;
        }
    }
	
    pCodecCtx = avcodec_alloc_context3(pCodec);
    if (!pCodecCtx) 
	{
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }
	
	video = ifmt_ctx->streams[videoindex];
    if (avcodec_parameters_to_context(pCodecCtx, video->codecpar) < 0)
        return -1;

	//pCodecCtx->get_format  = get_hw_format(pCodecCtx,(const)hw_pix_fmt);
	pCodecCtx->get_format  = get_hw_format;
    av_opt_set_int(pCodecCtx, "refcounted_frames", 1, 0);

	
	if (hw_decoder_init(pCodecCtx, type) < 0)
			return -1;
	
    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) 
	{
        printf("Couldn't open codec.\n");
        return -1;
    }
}
