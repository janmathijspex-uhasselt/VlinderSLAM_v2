#ifndef POSEIO_H
#define POSEIO_H

#pragma once
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <string>

namespace poseio {

/// Save a 4x4 pose matrix to a txt file
static inline bool savePose(const std::string& filename,
                            const cv::Mat& pose)
{
    if (pose.rows != 4 || pose.cols != 4 || pose.type() != CV_32F) {
        std::cerr << "Pose must be a 4x4 CV_32F matrix!" << std::endl;
        return false;
    }

    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Could not open file for writing: " << filename << std::endl;
        return false;
    }

    // Write matrix row by row
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out << pose.at<float>(i, j);
            if (j < 3) out << " ";
        }
        out << "\n";
    }

    return true;
}

/// Load a 4x4 pose matrix from a txt file
/// Returns matrix as row-major cv::Mat (CV_32F, 4x4)
static inline bool loadPose(const std::string& filename, cv::Mat& pose)
{
    std::ifstream in(filename);
    if (!in.is_open()) {
        std::cerr << "Could not open file for reading: " << filename << std::endl;
        return false;
    }

    cv::Mat tmp(4, 4, CV_32F);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!(in >> tmp.at<float>(i, j))) {
                std::cerr << "File format error while reading matrix: " << filename << std::endl;
                return false;
            }
        }
    }

    pose = tmp;
    return true;
}

} // namespace poseio


#endif // POSEIO_H
