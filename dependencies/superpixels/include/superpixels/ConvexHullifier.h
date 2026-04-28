#pragma once
#include <superpixels/utils.h>

#include <cv_bridge/cv_bridge.h>

class ConvexHullifier
{
    public:
        ConvexHullifier(const SuperpixelParams & params);

        void setParams(const SuperpixelParams & params);

        void run(std::vector<std::vector<double>> & centers,
                    std::vector<int> & center_counts,
                    std::vector<std::vector<cv::Point>> & superpixels,
                    std::vector<std::vector<Eigen::Vector2d>> & superpixel_projections,
                    std::vector<std::vector<Eigen::Vector2d>> & superpixel_convex_hulls,
                    std::vector<Eigen::Matrix3d> & superpixel_rotations,
                    const cv::Mat & depth_image);

    private:

        void convexHull(const std::vector<Eigen::Vector2d> & superpixel, std::vector<Eigen::Vector2d> & convex_hull);

        bool polarSort(const Eigen::Vector2d & a, const Eigen::Vector2d & b, const Eigen::Vector2d & lowest);

        bool ccw(const Eigen::Vector2d & a, const Eigen::Vector2d & b, const Eigen::Vector2d & c);

        void grahamScan(const std::vector<Eigen::Vector2d> & superpixel, std::vector<Eigen::Vector2d> & convex_hull);

        Eigen::Vector3d projectPointOntoPlane(const Eigen::Vector3d & regionPt);

        SuperpixelParams params_;    

        rclcpp::Logger logger_ = rclcpp::get_logger("ConvexHullifier");
};
