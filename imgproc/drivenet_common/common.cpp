/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "common.hpp"

cudaStream_t g_cudaStream  = 0;


//// Driveworks Handles
dwContextHandle_t gSdk                          = DW_NULL_HANDLE;
dwRendererHandle_t gRenderer                    = DW_NULL_HANDLE;
dwRenderBufferHandle_t gLineBuffer              = DW_NULL_HANDLE;
dwSALHandle_t gSal                              = DW_NULL_HANDLE;
dwSensorHandle_t gCameraSensor                  = DW_NULL_HANDLE;
dwRawPipelineHandle_t gRawPipeline              = DW_NULL_HANDLE;

// Sample variables
std::string gInputType;

dwImageCUDA gRCBImage{};

dwImageCUDA gRGBAImage{};
dwImageCUDA gRGGBImage{};

dwImageProperties gRCBProperties{};

dwImageStreamerHandle_t gCuda2gl                = DW_NULL_HANDLE;
dwImageStreamerHandle_t gInput2cuda             = DW_NULL_HANDLE;
dwImageFormatConverterHandle_t gConvert2RGBA    = DW_NULL_HANDLE;
dwImageFormatConverterHandle_t gConvertRGB2RCB    = DW_NULL_HANDLE;
dwImageFormatConverterHandle_t gConvertRGB2RGBA    = DW_NULL_HANDLE;
dwImageFormatConverterHandle_t gConvertRGBA2RCB    = DW_NULL_HANDLE;
dwImageFormatConverterHandle_t gConvertRGB2RGGB    = DW_NULL_HANDLE;
dwImageFormatConverterHandle_t gConvertRGB2RAW    = DW_NULL_HANDLE;

//// frame processing

//// Sample variables
dwRect gScreenRectangle{};

//// Colors for rendering bounding boxes
const uint32_t gMaxBoxColors = 5;
float32_t gBoxColors[5][4] = {{1.0f, 0.0f, 0.0f, 1.0f},
                              {0.0f, 1.0f, 0.0f, 1.0f},
                              {0.0f, 0.0f, 1.0f, 1.0f},
                              {0.0f, 1.0f, 1.0f, 1.0f},
                              {1.0f, 1.0f, 0.0f, 1.0f}};
//------------------------------------------------------------------------------

void drawROI(dwRect roi, const float32_t color[4], dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer)
{
    float32_t x_start = static_cast<float32_t>(roi.x) ;
    float32_t x_end   = static_cast<float32_t>(roi.x + roi.width);
    float32_t y_start = static_cast<float32_t>(roi.y);
    float32_t y_end   = static_cast<float32_t>(roi.y + roi.height);

    float32_t *coords     = nullptr;
    uint32_t maxVertices  = 0;
    uint32_t vertexStride = 0;
    dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, renderBuffer);
    coords[0]  = x_start;
    coords[1]  = y_start;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_start;
    coords[1] = y_start;
    dwRenderBuffer_unmap(8, renderBuffer);
    dwRenderer_setColor(color, renderer);
    dwRenderer_setLineWidth(2, renderer);
    dwRenderer_renderBuffer(renderBuffer, renderer);
}

//------------------------------------------------------------------------------
bool initPipeline(const dwImageProperties &rawImageProps, const dwCameraProperties &cameraProps, dwContextHandle_t ctx)
{
    dwStatus result = dwRawPipeline_initialize(&gRawPipeline, rawImageProps, cameraProps, ctx);
    if (result != DW_SUCCESS) {
        std::cout << "Image streamer initialization failed: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    // Initialize Raw pipeline
    dwRawPipeline_setCUDAStream(g_cudaStream, gRawPipeline);
    dwRawPipeline_getDemosaicImageProperties(&gRCBProperties, gRawPipeline);

    // RCB image to get output from the RawPipeline


//    // RGBA image to display over GL
//    dwImageProperties rgbaImageProperties = gRCBProperties;
//    rgbaImageProperties.pxlFormat         = DW_IMAGE_RGBA;
//    rgbaImageProperties.pxlType           = DW_TYPE_UINT8;
//    rgbaImageProperties.planeCount        = 1;
//    dwImageCUDA_create(&gRGBAImage, &rgbaImageProperties, DW_IMAGE_CUDA_PITCH);

//    // Setup streamer to pass input to CUDA and from CUDA to GL
//    result = result != DW_SUCCESS ? result : dwImageStreamer_initialize(&gInput2cuda, &rawImageProps, DW_IMAGE_CUDA, ctx);
//    result = result != DW_SUCCESS ? result : dwImageStreamer_initialize(&gCuda2gl, &rgbaImageProperties, DW_IMAGE_GL, ctx);
//    if (result != DW_SUCCESS) {
//        std::cout << "Image streamer initialization failed: " << dwGetStatusName(result) << std::endl;
//        return false;
//    }

    /*
     *
     *  Jung addhoc
     *
     * */

    gRCBProperties.type = DW_IMAGE_CUDA;
    gRCBProperties.pxlFormat = DW_IMAGE_RCB;
    gRCBProperties.planeCount = 3;
    gRCBProperties.width = (uint32_t)960*2;
    gRCBProperties.height = (uint32_t)604*2;
    gRCBProperties.pxlType = DW_TYPE_FLOAT16;


    dwImageProperties rawImageCPU_prop{};

    rawImageCPU_prop.type = DW_IMAGE_CPU;
    rawImageCPU_prop.pxlFormat = DW_IMAGE_RGB;
    rawImageCPU_prop.planeCount = 1;
    rawImageCPU_prop.width = (uint32_t)960*2;
    rawImageCPU_prop.height = (uint32_t)604*2;
    rawImageCPU_prop.pxlType = DW_TYPE_UINT8;

    dwImageProperties rawImageCPU_RAW_prop{};

    rawImageCPU_RAW_prop.type = DW_IMAGE_CPU;
    rawImageCPU_RAW_prop.pxlFormat = DW_IMAGE_RAW;
    rawImageCPU_RAW_prop.planeCount = 1;
    rawImageCPU_RAW_prop.width = (uint32_t)960*2;
    rawImageCPU_RAW_prop.height = (uint32_t)604*2;
    rawImageCPU_RAW_prop.pxlType = DW_TYPE_UINT8;


    dwImageProperties cudaImage_prop_RGB{};

    cudaImage_prop_RGB.type = DW_IMAGE_CUDA;
    cudaImage_prop_RGB.pxlFormat = DW_IMAGE_RGB;
    cudaImage_prop_RGB.planeCount = 1;
    cudaImage_prop_RGB.width = (uint32_t)960*2;
    cudaImage_prop_RGB.height = (uint32_t)604*2;
    cudaImage_prop_RGB.pxlType = DW_TYPE_UINT8;

    dwImageProperties cudaImage_prop_RGGB{};

    cudaImage_prop_RGGB.type = DW_IMAGE_CUDA;
    cudaImage_prop_RGGB.pxlFormat = DW_IMAGE_RGGB;
    cudaImage_prop_RGGB.planeCount = 1;
    cudaImage_prop_RGGB.width = (uint32_t)960*2;
    cudaImage_prop_RGGB.height = (uint32_t)604*2;
    cudaImage_prop_RGGB.pxlType = DW_TYPE_UINT8;

    dwImageCUDA_create(&gRGGBImage, &cudaImage_prop_RGGB, DW_IMAGE_CUDA_PITCH);

    dwImageProperties cudaImage_prop{};

    cudaImage_prop.type = DW_IMAGE_CUDA;
    cudaImage_prop.pxlFormat = DW_IMAGE_RCB;
    cudaImage_prop.planeCount = 3;
    cudaImage_prop.width = (uint32_t)960*2;
    cudaImage_prop.height = (uint32_t)604*2;
    cudaImage_prop.pxlType = DW_TYPE_UINT8;

    dwImageCUDA_create(&gRCBImage, &cudaImage_prop, DW_IMAGE_CUDA_PITCH);

    dwImageProperties rgbaImage_prop{};

    rgbaImage_prop.type = DW_IMAGE_CUDA;
    rgbaImage_prop.pxlFormat = DW_IMAGE_RGBA;
    rgbaImage_prop.planeCount = 1;
    rgbaImage_prop.width = (uint32_t)960*2;
    rgbaImage_prop.height = (uint32_t)604*2;
    rgbaImage_prop.pxlType = DW_TYPE_UINT8;

    dwImageCUDA_create(&gRGBAImage, &rgbaImage_prop, DW_IMAGE_CUDA_PITCH);

    result = result != DW_SUCCESS ? result : dwImageStreamer_initialize(&gInput2cuda, &rawImageCPU_prop, DW_IMAGE_CUDA, ctx);
    result = result != DW_SUCCESS ? result : dwImageStreamer_initialize(&gCuda2gl, &rgbaImage_prop, DW_IMAGE_GL, ctx);
    if (result != DW_SUCCESS) {
        std::cerr << "Image streamer initialization failed: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    // init format converter to convert from RCB->RGBA
    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvert2RGBA, &cudaImage_prop, &rgbaImage_prop, ctx);
    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvertRGB2RCB, &cudaImage_prop_RGB, &cudaImage_prop, ctx);
    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvertRGB2RGBA, &cudaImage_prop_RGB, &rgbaImage_prop, ctx);
//    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvertRGBA2RCB, &rgbaImage_prop, &cudaImage_prop, ctx);
    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvertRGB2RGGB, &cudaImage_prop_RGB, &cudaImage_prop_RGGB, ctx);
//    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvertRGB2RAW, &rawImageCPU_prop, &rawImageCPU_RAW_prop, ctx);
    if (result != DW_SUCCESS) {
        std::cout << "Image format converter initialization failed: " << dwGetStatusName(result) << std::endl;
        return false;
    }
    else
    {
        std::cout << "Image format converter initialization success " << std::endl;
    }

    return true;
}

//------------------------------------------------------------------------------
void initSdk(dwContextHandle_t *context, WindowBase *window)
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams{};

    std::string path = DataPath::get();
    sdkParams.dataPath = path.c_str();

#ifdef VIBRANTE
    sdkParams.eglDisplay = window->getEGLDisplay();
#else
    (void)window;
#endif

    dwInitialize(context, DW_VERSION, &sdkParams);
}


//------------------------------------------------------------------------------
void initRenderer(const dwImageProperties& rcbProperties, dwRendererHandle_t *renderer, dwContextHandle_t context, WindowBase *window)
{
    dwStatus result = dwRenderer_initialize(renderer, context);
    if (result != DW_SUCCESS)
        throw std::runtime_error(std::string("Cannot init renderer: ") + dwGetStatusName(result));

    // Set some renderer defaults
    gScreenRectangle.width  = window->width();
    gScreenRectangle.height = window->height();
    gScreenRectangle.x      = 0;
    gScreenRectangle.y      = 0;

    float32_t rasterTransform[9];
    rasterTransform[0] = 1.0f;
    rasterTransform[3] = 0.0f;
    rasterTransform[6] = 0.0f;
    rasterTransform[1] = 0.0f;
    rasterTransform[4] = 1.0f;
    rasterTransform[7] = 0.0f;
    rasterTransform[2] = 0.0f;
    rasterTransform[5] = 0.0f;
    rasterTransform[8] = 1.0f;

    dwRenderer_set2DTransform(rasterTransform, *renderer);
    float32_t boxColor[4] = {0.0f,1.0f,0.0f,1.0f};
    dwRenderer_setColor(boxColor, *renderer);
    dwRenderer_setLineWidth(2.0f, *renderer);

    dwRenderer_setRect(gScreenRectangle, *renderer);

    uint32_t maxLines = 20000;
    {
        dwRenderBufferVertexLayout layout;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        dwRenderBuffer_initialize(&gLineBuffer, layout, DW_RENDER_PRIM_LINELIST, maxLines, context);
        dwRenderBuffer_set2DCoordNormalizationFactors((float32_t)rcbProperties.width,
                                                      (float32_t)rcbProperties.height, gLineBuffer);
    }
}

//------------------------------------------------------------------------------
bool initSensors(dwSALHandle_t *sal, dwSensorHandle_t *camera, dwImageProperties *cameraImageProperties,
                 dwCameraProperties* cameraProperties, dwContextHandle_t context)
{
    dwStatus result;

    result = dwSAL_initialize(sal, context);
    if (result != DW_SUCCESS) {
        std::cout << "Cannot initialize SAL: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    // create GMSL Camera interface
    dwSensorParams params;
#ifdef VIBRANTE
    if (gInputType.compare("camera") == 0) {
        std::string parameterString = "camera-type=" + gArguments.get("camera-type");
        parameterString += ",csi-port=" + gArguments.get("csi-port");
        parameterString += ",slave=" + gArguments.get("slave");
        parameterString += ",serialize=false,output-format=raw,camera-count=4";
        std::string cameraMask[4] = {"0001", "0010", "0100", "1000"};
        uint32_t cameraIdx = std::stoi(gArguments.get("camera-index"));
        if(cameraIdx < 0 || cameraIdx > 3){
            std::cerr << "Error: camera index must be 0, 1, 2 or 3" << std::endl;
            return false;
        }
        parameterString += ",camera-mask=" + cameraMask[cameraIdx];

        params.parameters           = parameterString.c_str();
        params.protocol             = "camera.gmsl";

        result                      = dwSAL_createSensor(camera, params, *sal);
    }else
#endif
    {
        std::string parameterString = gArguments.parameterString();
        params.parameters           = parameterString.c_str();
        params.protocol             = "camera.virtual";
        result                      = dwSAL_createSensor(camera, params, *sal);
    }
    if (result != DW_SUCCESS) {
        std::cout << "Cannot create driver: camera.virtual with params: " << params.parameters << std::endl
                  << "Error: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    dwSensorCamera_getImageProperties(cameraImageProperties, DW_CAMERA_RAW_IMAGE, *camera);
    dwSensorCamera_getSensorProperties(cameraProperties, *camera);

#ifdef VIBRANTE
    if(gInputType.compare("camera") == 0){
        if(cameraImageProperties->pxlFormat == DW_IMAGE_RCCB ||
           cameraImageProperties->pxlFormat == DW_IMAGE_BCCR ||
           cameraImageProperties->pxlFormat == DW_IMAGE_CRBC ||
           cameraImageProperties->pxlFormat == DW_IMAGE_CBRC){

           std::cout << "Camera image with " << cameraProperties->resolution.x << "x"
                << cameraProperties->resolution.y << " at " << cameraProperties->framerate << " FPS" << std::endl;

           return true;
        }
        else{
            std::cerr << "Camera is not supported" << std::endl;

            return false;
        }
    }
#endif

    return true;
}

//------------------------------------------------------------------------------
void resizeWindowCallback(int width, int height) {
   gScreenRectangle.width = width;
   gScreenRectangle.height = height;
   gScreenRectangle.x = 0;
   gScreenRectangle.y = 0;
   dwRenderer_setRect(gScreenRectangle, gRenderer);
}


//------------------------------------------------------------------------------
bool getNextFrameImages(dwImageCUDA** rcbCudaImageOut, dwImageGL** rgbaGLImageOut, dwCameraFrameHandle_t frameHandle)
{
    dwStatus result = DW_SUCCESS;

    const dwCameraDataLines* dataLines;
    dwImageCPU  rawImageCPU;
    dwImageCUDA *rawImageCUDA;

    /*
     *      Jung add hoc part
     * */
    // Load sample image
    cv::Mat ori_img = cv::imread("/home/usrg_eurecar_stu/Documents/1507702514030235.jpg");
    cv::resize(ori_img,ori_img,cv::Size(),2,2);
//    cv::cvtColor(ori_img,ori_img,CV_BGR2RGB);


    dwImageProperties rawImageCPU_prop;

    rawImageCPU_prop.type = DW_IMAGE_CPU;
    rawImageCPU_prop.pxlFormat = DW_IMAGE_RGB;
    rawImageCPU_prop.planeCount = 1;
    rawImageCPU_prop.width = (uint32_t)ori_img.cols;
    rawImageCPU_prop.height = (uint32_t)ori_img.rows;
    rawImageCPU_prop.pxlType = DW_TYPE_UINT8;

    result = dwImageCPU_create(&rawImageCPU,(const dwImageProperties*)&rawImageCPU_prop);

    if(result != DW_SUCCESS)
    {
        std::cout << "dw ImageCPU create error" << std::endl;
    }
    else{
//        std::cout << "dw ImageCPU create success" << std::endl;
    }

    memcpy(rawImageCPU.data[0],ori_img.data, ori_img.total()*ori_img.channels());



#ifdef VIBRANTE
    dwImageNvMedia *rawImageNvMedia = nullptr;

    if (gInputType.compare("camera") == 0) {
        result = dwSensorCamera_getImageNvMedia(&rawImageNvMedia, DW_CAMERA_RAW_IMAGE, frameHandle);
        rawImageNvMedia->prop.pxlFormat = DW_IMAGE_RAW;
    }else
#endif
    {
//        result = dwSensorCamera_getImageCPU(&rawImageCPU, DW_CAMERA_RAW_IMAGE, frameHandle);
    }

//    if (result != DW_SUCCESS) {
//        std::cout << "Cannot get raw image: " << dwGetStatusName(result) << std::endl;
//        return false;
//    }
    dwSensorCamera_getDataLines(&dataLines, frameHandle);

    // process
#ifdef VIBRANTE
    if (gInputType.compare("camera") == 0) {
        result = dwImageStreamer_postNvMedia(rawImageNvMedia, gInput2cuda);
    }else
#endif
    {
        result = dwImageStreamer_postCPU(&rawImageCPU, gInput2cuda);
    }
    if (result != DW_SUCCESS) {
        std::cout << "Cannot post raw image: " << dwGetStatusName(result) << std::endl;
        return false;
    }

    // input image was posted, get now CUDA image out of the streamer
    dwImageStreamer_receiveCUDA(&rawImageCUDA, 10000, gInput2cuda);
    {
        // remap CUDA images to the actual data representing the frame
        // the reason is that the image streamer will unfold the embedded lines, if such exists in source image
        dwImageCUDA cudaFrame{};
        {
            dwRect roi;
            roi.x = 0;
            roi.y = 0;
            roi.width = 960*2;
            roi.height = 604*2;
//            dwSensorCamera_getImageROI(&roi, gCameraSensor);
            dwImageCUDA_mapToROI(&cudaFrame, rawImageCUDA, roi);
        }

//        // RAW -> RCB
//        {
//            dwRawPipeline_convertRawToDemosaic(&gRCBImage, &cudaFrame, dataLines, gRawPipeline);
//        }

//        // RGB -> RCB
//        {
//            dwImageFormatConverter_copyConvertCUDA(&gRCBImage, &cudaFrame, gConvertRGB2RCB, g_cudaStream);
//        }




        // return used RAW image, we do not need it anymore, as we now have a copy through the RawPipeline
        dwImageStreamer_returnReceivedCUDA(rawImageCUDA, gInput2cuda);
    }


    // wait
#ifdef VIBRANTE
    if (gInputType.compare("camera") == 0) {
        dwImageStreamer_waitPostedNvMedia(&rawImageNvMedia, 10000, gInput2cuda);
    }else
#endif
    {
        dwImageCPU  *rawImageCPU_pt = &rawImageCPU;
        dwImageStreamer_waitPostedCPU(&rawImageCPU_pt, 10000, gInput2cuda);
    }

    // RGB -> RGBA
    {
        dwImageFormatConverter_copyConvertCUDA(&gRGBAImage, rawImageCUDA, gConvertRGB2RGBA, g_cudaStream);
    }

//    dwImageCUDA cuda_RGGB_img_tmp{};
    // RGB -> RGGB
    {
        dwImageFormatConverter_copyConvertCUDA(&gRGGBImage, rawImageCUDA, gConvertRGB2RGGB, g_cudaStream);
    }

    // get GL image
    {
        dwImageStreamer_postCUDA(&gRGBAImage, gCuda2gl);
        dwImageStreamer_receiveGL(rgbaGLImageOut, 10000, gCuda2gl);
    }

    // RCB result
    *rcbCudaImageOut = &gRCBImage;

//    *rcbCudaImageOut = rawImageCUDA;
    return true;
}

//------------------------------------------------------------------------------
void returnNextFrameImages(dwImageCUDA* rcbCudaImageOut, dwImageGL* rgbaGLImage)
{
    (void)rcbCudaImageOut;

    // return GL image
    {
        dwImageStreamer_returnReceivedGL(rgbaGLImage, gCuda2gl);
        dwImageCUDA *returnedFrame;
        dwImageStreamer_waitPostedCUDA(&returnedFrame, 10000, gCuda2gl);
    }
}

//------------------------------------------------------------------------------
void release()
{
    if (gRGBAImage.dptr[0]) dwImageCUDA_destroy(&gRGBAImage);
    if (gRCBImage.dptr[0]) dwImageCUDA_destroy(&gRCBImage);

    if (gConvert2RGBA) {
        dwImageFormatConverter_release(&gConvert2RGBA);
    }
    if (gCuda2gl) {
        dwImageStreamer_release(&gCuda2gl);
    }
    if (gInput2cuda) {
        dwImageStreamer_release(&gInput2cuda);
    }
    if (g_cudaStream) {
        cudaStreamDestroy(g_cudaStream);
    }
    if (gLineBuffer) {
        dwRenderBuffer_release(&gLineBuffer);
    }
    if (gRenderer) {
        dwRenderer_release(&gRenderer);
    }
    if (gCameraSensor) {
        dwSAL_releaseSensor(&gCameraSensor);
    }
    if (gSal) {
        dwSAL_release(&gSal);
    }
    if (gRawPipeline) {
        dwRawPipeline_release(&gRawPipeline);
    }
    dwRelease(&gSdk);
    dwLogger_release();
}
