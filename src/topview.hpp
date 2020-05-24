//
// Created by qinhan on 2020/5/14.
//

#ifndef CALIBRATION_TOPVIEW_HPP
#define CALIBRATION_TOPVIEW_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <sophus/se3.hpp>


using namespace cv;

class topview {

public:
    topview(std::vector<Mat> &rawimages) {
        images = rawimages;
    }

    void initSize() {

        shiftSize = Size(200, 200);
        vehicleSize = Size(240, 380);
        chessBordSize = Size(60, 60);
        imageSize = Size(2 * shiftSize.width + vehicleSize.width + 2 * chessBordSize.width,
                         2 * shiftSize.height + vehicleSize.height + 2 * chessBordSize.height);
    }

    // init project table
    // 0-front/ 1-right/ 2-back/ 3-left
    void init_project_table(){

        char buf[64];
        for (int i = 0; i < 4; ++i) {
            sprintf(buf, "../data/CameraParam%d.yml", i);
            cv::FileStorage fs(buf,cv::FileStorage::READ);

        }

    }

    void calculte_project_table(){


    }

    void project_to_ground(){

    }




private:
    Mat p_GF_table_32F;
    Mat p_GB_table_32F;
    Mat p_GR_table_32F;
    Mat p_GL_table_32F;
    int cameraType;
    Matx33d intrinsic_matrix;
    Vec4d distortion_coeffs;
    Matx33d rotation_matrix;
    Vec3d transform_vector;
    Eigen::Matrix3d matrix_temp;
    Eigen::Vector3d vector_temp;

};


#endif //CALIBRATION_TOPVIEW_HPP
