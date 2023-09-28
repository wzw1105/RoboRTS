#include "layer.h"
#include "net.h"

#if defined(USE_NCNN_SIMPLEOCV)
#include "simpleocv.h"
#else
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif
#include <float.h>
#include <cstdio>
#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>

#define YOLOX_NMS_THRESH  0.6 // nms threshold
#define YOLOX_CONF_THRESH 0.7 // threshold of bounding box prob
#define YOLOX_TARGET_SIZE 416  // target image size after resize, might use 416 for small model

// YOLOX use the same focus in yolov5
class YoloV5Focus : public ncnn::Layer
{
public:
    YoloV5Focus() { one_blob_only = true; }
    virtual int forward(const ncnn::Mat& bottom_blob, ncnn::Mat& top_blob, const ncnn::Option& opt) const;
};

DEFINE_LAYER_CREATOR(YoloV5Focus)

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};


int init_model(const std::string param_path, const std::string model_path);
int detect_yolox(const cv::Mat& bgr, std::vector<Object>& objects);
cv::Mat draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects);
