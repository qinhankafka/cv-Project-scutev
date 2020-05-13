//
// Created by qinhan on 2020/5/12.
//

#ifndef CALIBRATION_CALIBRATIONTOOLS_HPP
#define CALIBRATION_CALIBRATIONTOOLS_HPP

#include <string>
#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/imgproc.hpp>

using namespace cv;

class CalibrationTool {
public:
    int cameratype;
    int imgNum;
    int succesNum;
    bool patternfound;
    std::vector<Mat> images;
    std::vector<Mat> images_gray;
    std::vector<Mat> success_images;
    Mat imgtemp;

    Size board_size = Size(10, 9);
    Size square_size = Size(100, 100);


    // 
    CalibrationTool(std::vector<Mat>& input_images, int camera_type, int img_number) {
        cameratype = camera_type;
        images = input_images;
        imgNum = img_number;
        images_gray.clear();
        corners_seq.clear();
        success_images.clear();
        object_points.clear();
        patternfound = 0;
        succesNum = 0;
        convertImage();
        findCorners();
        initPointSet();
        calibrate();
    }

    void convertImage() {
        for (Mat& img : images) {
            cvtColor(img, imgtemp, COLOR_RGB2GRAY);
            images_gray.push_back(imgtemp);
        }
    }

    // Extract the checkerboard corners in the image
    void findCorners() {

        for (int i = 0; i < imgNum; ++i) {
            patternfound = findChessboardCorners(images[i], board_size, corners,
                                                 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
                                                 CALIB_CB_FAST_CHECK);
            if (!patternfound) {
                continue;
            } else {
                cornerSubPix(images_gray[i], corners, Size(11, 11), Size(-1, 1),
                             TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.1));
                success_images.push_back(images[i]);
                succesNum++;
                corners_seq.push_back(corners);
            }

        }
    }

    void initPointSet() {
        for (int i = 0; i < board_size.height; ++i) {
            for (int j = 0; j < board_size.width; ++j) {
                Point3f tempPoint;
                tempPoint.x = i * square_size.width;
                tempPoint.y = j * square_size.height;
                tempPoint.z = 0.0;
                pointset.push_back(tempPoint);
            }
        }
        for (int k = 0; k < succesNum; ++k) {
            object_points.push_back(pointset);
        }
    }

    void calibrate() {
        if (cameratype == 0) {
            // fisheye camera calibration
            fisheye::calibrate(object_points, corners_seq, images[0].size(), intrinsic_matrix, distortion_coeffs, rotation_vector,
                               translation_vectors,
                               fisheye::CALIB_RECOMPUTE_EXTRINSIC + fisheye::CALIB_CHECK_COND + fisheye::CALIB_FIX_SKEW,
                               TermCriteria(3, 20, 1e-6));
        } else if (cameratype == 1) {
            //normal camera calibration
            calibrateCamera(pointset, corners, images[0].size(), intrinsic_matrix, distortion_coeffs, rotation_vector,
                            translation_vectors, CALIB_USE_INTRINSIC_GUESS, TermCriteria(3, 20, 1e-6));
        }
    }

    Matx33d getIntrinsicMatrix(){
        return intrinsic_matrix;
    }


private:
    Matx33d intrinsic_matrix;
    Vec4d distortion_coeffs;
    std::vector<Vec3d> rotation_vector;
    std::vector<Vec3d> translation_vectors;
    std::vector<Point2f> corners;
    std::vector<std::vector<Point2f>> corners_seq;
    std::vector<Point3f> pointset;
    std::vector<std::vector<Point3f>> object_points;

};


#endif //CALIBRATION_CALIBRATIONTOOLS_HPP
