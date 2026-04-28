#pragma once

// #include <ros/ros.h>
// #include "rclcpp/rclcpp.hpp"

struct PinholeCamera
{
    double fx; // focal length x
    double fy; // focal length y
    double inv_fx; // 1 / focal length x (for efficiency)
    double inv_fy; // 1 / focal length y (for efficiency)
    double u_0; // optical center x
    double v_0; // optical center y
    double k1; // radial distortion coefficient
    double k2; // radial distortion coefficient
    double k3; // radial distortion coefficient
    double p1; // tangential distortion coefficient
    double p2; // tangential distortion coefficient
    int width; // image width (in pixels)
    int height; // image height (in pixels)

    PinholeCamera()
    {
        fx = 347.997;
        fy = 347.997;
        inv_fx = 1.0 / fx;
        inv_fy = 1.0 / fy;
        width = 640;
        height = 480;
        u_0 = width / 2.0;
        v_0 = height / 2.0;
        k1 = 0.0;
        k2 = 0.0;
        k3 = 0.0;
        p1 = 0.0;
        p2 = 0.0;
    }

    PinholeCamera(double fx, double fy, double u_0, double v_0, double k1, double k2, double k3, double p1, double p2,
                    double width, double height)
    {
        this->fx = fx;
        this->fy = fy;
        this->u_0 = u_0;
        this->v_0 = v_0;
        this->k1 = k1;
        this->k2 = k2;
        this->k3 = k3;
        this->p1 = p1;
        this->p2 = p2;
        this->width = width;
        this->height = height;
    }
};