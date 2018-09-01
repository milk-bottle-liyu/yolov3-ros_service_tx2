#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "detect/obj_detect.h"

#include "darknet.h"
#include "ros_params.hpp"

list * g_options;
network * g_net;
float g_thresh,g_hier_thresh,g_nms;

inline int limit_range(float x,int a,int b)
{
    if (x<=a) return a;
    if (x>=b) return b;
    return x;
}

void ms_sleep(unsigned msecs )
{
    int rc;
    struct timeval tv;

    do
    {
        tv.tv_sec = msecs / 1000;
        tv.tv_usec = ( msecs % 1000 ) * 1000;
        rc = select( 0, NULL, NULL, NULL, &tv );
    }
    while ( rc == -1 && errno == EINTR );
}

image color_mat_to_image(const cv::Mat &src)
{
    int h = src.rows;
    int w = src.cols;
    int c = src.channels();

    image out = make_image(w, h, c);
    IplImage src_tmp(src);

    unsigned char *data = (unsigned char *)src_tmp.imageData;
    h = src_tmp.height;
    w = src_tmp.width;
    c = src_tmp.nChannels;
    int step = src_tmp.widthStep;
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                out.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return out;
}

bool detect_func(detect::obj_detect::Request &req,
                detect::obj_detect::Response &res)
{
    cv_bridge::CvImagePtr src= cv_bridge::toCvCopy(req.img,sensor_msgs::image_encodings::BGR8);

    image im = color_mat_to_image(src->image);
    image sized = letterbox_image(im, g_net->w, g_net->h);
    layer l = g_net->layers[g_net->n-1];

    float *X = sized.data;
    network_predict(g_net, X);
    int nboxes = 0;
    detection *dets = get_network_boxes(g_net, im.w, im.h, g_thresh, g_hier_thresh, 0, 1, &nboxes);
    if (g_nms) do_nms_sort(dets, nboxes, l.classes, g_nms);

    int max_idx=-1;
    float max_prob=-1.0;
    for (int i=0;i<nboxes;i++)
    {
        if (max_prob<dets[i].prob[dets[i].sort_class])
        {
            max_prob=dets[i].prob[dets[i].sort_class];
            max_idx=i;
        }
    }

    if (max_idx==-1) {
        res.lablel = -1;
        res.bottom_left_x=0;
        res.bottom_left_y=0;
        res.top_left_y=0;
        res.top_left_x=0;
    }
    else {
        res.lablel = dets[max_idx].sort_class;
        res.bottom_left_x=limit_range(im.w*(dets[max_idx].bbox.x+dets[max_idx].bbox.w/2),0,im.w);
        res.bottom_left_y=limit_range(im.h*(dets[max_idx].bbox.y+dets[max_idx].bbox.h/2),0,im.h);
        res.top_left_y=limit_range(im.w*(dets[max_idx].bbox.x-dets[max_idx].bbox.w/2),0,im.w);
        res.top_left_x=limit_range(im.h*(dets[max_idx].bbox.x-dets[max_idx].bbox.h/2),0,im.h);
    }

    ROS_INFO("label %d",res.lablel);
    free_detections(dets,nboxes);
    free_image(im);
    free_image(sized);
    return true;
}
int main(int argc,char *argv[])
{
    ros::init(argc,argv,"detect_server");
    ros::NodeHandle handle;

    READ_PARAM_BEGIN;
    READ_PARAM_WITH_DEFAULT(std::string, cfg_path , "/home/eli/catkin_ws/src/detector_server/cfg/");
    READ_PARAM_WITH_DEFAULT(std::string, data_file , "voc.data");
    READ_PARAM_WITH_DEFAULT(std::string, cfg_file , "test.cfg");
    READ_PARAM_WITH_DEFAULT(std::string, weight_file, "model.weights");
    READ_PARAM_WITH_DEFAULT(std::string, name_file, "voc.name");
    READ_PARAM_WITH_DEFAULT(float, thresh ,0.99);
    READ_PARAM_WITH_DEFAULT(float, hier_thresh, 0.5);
    READ_PARAM_WITH_DEFAULT(float, nms, 0.45);
    READ_PARAM_END;

    g_thresh=thresh;
    g_hier_thresh=hier_thresh;
    g_nms=nms;

    g_net=load_network((cfg_path+cfg_file).c_str(),(cfg_path+weight_file).c_str(),0);
    set_batch_network(g_net,1);
    srand(2282332);

    ros::ServiceServer server=handle.advertiseService("detect_srv",detect_func);
    ROS_INFO("Ready to detect");
    while (ros::ok())
    {
        ms_sleep(500);
        ros::spinOnce();
    }

    free_network(g_net);
    return 0;
}
