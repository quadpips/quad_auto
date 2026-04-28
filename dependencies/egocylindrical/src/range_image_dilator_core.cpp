#include <egocylindrical/range_image_dilator_core.h>
#include <egocylindrical/range_image_common.h>

#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/msg/image.hpp>  //Redundant

#include <stdexcept>    //To throw runtime error

//   template<typename T> struct RangeVals {};

 
// template<>
// struct RangeVals<uint16_t>
// {
//     static inline uint scale() { return 1000;}
//     static inline bool is_valid(uint16_t v) {return v > 0;}
// };

// template<>
// struct RangeVals<float>
// {
//     static inline uint scale() { return 1;}
//     static inline bool is_valid(float v) {return v == v;}
// };

namespace egocylindrical
{

namespace utils
{


template <typename T>
cv::Mat dilateImage(const cv::Mat image_in)
{
    cv::Mat image_out(image_in.rows, image_in.cols, image_in.type(), cv::Scalar(0));

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::Point anchor(-1,-1);
    int iterations = 1;
    cv::dilate(image_in, image_out, kernel, anchor, iterations);
    cv::erode(image_out, image_out, kernel, anchor, iterations);

    for (int x = 0; x < image_in.cols; x++)
    {
        for (int y = 0; y < image_in.rows; y++)
        {
            T v_in = image_in.at<T>(y,x);
            if(RangeVals<T>::is_valid(v_in))
            {
                image_out.at<T>(y,x) = v_in;
            }
        }
    }
    return image_out;
}


sensor_msgs::msg::Image::SharedPtr dilateImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg_in)
{
//     if(image_msg_in->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
//     {
// //         // // ROS_ERROR(_STREAM("Range Image Dilator only supports encoding '16U'!");
//         throw std::runtime_error("Range Image Dilator only supports encoding '16U'!");
//     }

    cv_bridge::CvImageConstPtr cv_image_in = cv_bridge::toCvShare(image_msg_in);
//     const cv::Mat image_in = cv_bridge::toCvShare(image_msg_in)->image;
    const cv::Mat image_in = cv_image_in->image;

    cv::Mat image_out;
    if(image_msg_in->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        image_out = dilateImage<uint16_t>(image_in);
    }
    else if(image_msg_in->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        image_out = dilateImage<float>(image_in);
    }
    else
    {
        //// // ROS_ERROR(_STREAM("Range Image Dilator encountered unrecognized encoding [" << image_msg_in->encoding << "]");
        throw std::runtime_error("Range Image Dilator encountered unrecognized encoding");
    }

    // cv::Mat image_out(image_in.rows, image_in.cols, image_in.type(), cv::Scalar(0));

    // cv::Mat kernel;
    // cv::Point anchor(-1,-1);
    // int iterations = 1;
    // cv::dilate(image_in, image_out, kernel, anchor, iterations);

    // //Now remove the effect of dilation from pixels that started with non-0 values
    // // image_out.setTo(image_in, image_in!=0);
    // // cv::Mat good_val_mask = image_in > 0;
    // // image_out = (image_in & good_val_mask) + (image_out & ~good_val_mask);

    // for (int x = 0; x < image_in.cols; x++)
    // {
    //     for (int y = 0; y < image_in.rows; y++)
    //     {
    //         if (image_in.at<uint16_t>(y,x) != 0)
    //         {
    //             image_out.at<uint16_t>(y,x) = image_in.at<uint16_t>(y,x);
    //         }
    //     }
    // }



    cv_bridge::CvImage cv_image_out;
    cv_image_out.header = cv_image_in->header;
    cv_image_out.encoding = cv_image_in->encoding;
    cv_image_out.image = image_out;

    sensor_msgs::msg::Image::SharedPtr image_msg_out = cv_image_out.toImageMsg();
    return image_msg_out;
}

}

}