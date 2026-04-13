/**
 * @file detection.h
 * @brief Detection class definitions with the 2D and 3D detections
 */

#ifndef MULTIVIEW_DETECTION_H
#define MULTIVIEW_DETECTION_H

#include "ray.h"
#include <Eigen/Eigen>

namespace MultiView {

class Detection
{
public:
    Detection(/* args */) {};
    ~Detection() {};
    struct Detection2D
    {
        Eigen::Vector2d center;
        double width;
        double height;
        std::string class_id;
        float confidence;
        Ray ray;
    } detection2d;
    struct Detection3D
    {
        Eigen::Vector3d position;
        Eigen::Matrix3d covariance;
        std::vector<std::string> detection_ids;  // ByteTrack IDs from contributing 2D detections
    } detection3d;
};
}  // namespace MultiView
#endif  // MULTIVIEW_DETECTION_H