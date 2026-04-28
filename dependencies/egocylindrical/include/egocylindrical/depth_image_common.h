#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_COMMON_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_COMMON_H

#include <image_geometry/pinhole_camera_model.h>




namespace egocylindrical
{
    namespace utils
    {

        template<typename T> struct DepthScale {};
        
        template<>
        struct DepthScale<uint16_t>
        {
          static inline uint scale() { return 1000;}
        };
        
        template<>
        struct DepthScale<float>
        {
          static inline uint scale() { return 1;}
        };
        
        
        class InlineCameraModel : public image_geometry::PinholeCameraModel
        {
        public:
          cv::Point2d project3dToPixel(const cv::Point3d& xyz) const
          {
            assert( initialized() );
            assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane

            // [U V W]^T = P * [X Y Z 1]^T
            // u = U/W
            // v = V/W
            cv::Point2d uv_rect;
            uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
            uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
            return uv_rect;
          }

          cv::Point3d projectPixelTo3dRay(const cv::Point2d& uv_rect) const
          {
            assert( initialized() );

            cv::Point3d ray;
            ray.x = (uv_rect.x - cx() - Tx()) / fx();
            ray.y = (uv_rect.y - cy() - Ty()) / fy();
            ray.z = 1.0;
            return ray;
          }

        };
        
        //Inherits from PinholeCamerModel in order to access protected member function initRectificationMaps
        class CleanCameraModel : public InlineCameraModel
        {
        public:
            void init()
            {
                //Some of the camera model's functions don't work unless this has been called first
                PinholeCameraModel::initRectificationMaps();
            }
        };
        
    } //end namespace utils
} //end namespace egocylindrical

#endif  //EGOCYLINDRICAL_DEPTH_IMAGE_COMMON_H
