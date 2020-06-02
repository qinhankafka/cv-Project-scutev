//
// Created by qinhan on 2020/5/14.
//

#ifndef CALIBRATION_TOPVIEW_HPP
#define CALIBRATION_TOPVIEW_HPP

#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>


using namespace cv;

class Topview {

public:

    Topview() {
        std::cout<<"Initialization start"<<std::endl;
        initSize();
        init_K_G();
        init_project_table();
        generate_masks();
        std::cout<<"Initialization finished"<<std::endl;

    };

    explicit Topview(std::vector<Mat> &raw_images) {
        rawimages.clear();
        rawimages = raw_images;
        initSize();
        images_GC = std::vector<Mat>(4);
        read_masks();
        project_to_ground();
        generate_topview_img();
    }

    //Initialize the  size of the image, the size of the vehicle in image, etc.
    void initSize() {

        shiftSize = Size(145, 80);
        vehicleSize = Size(250, 540);
        imageSize = Size(2 * shiftSize.width + vehicleSize.width,
                         2 * shiftSize.height + vehicleSize.height);
    }

    // Init project table, and save the projection table to the data folder.
    // 0-front/ 1-right/ 2-back/ 3-left
    //rows = height = 700
    //cols = width = 540
    void init_project_table() {

        char buf[64];
        for (int i = 0; i < 4; ++i) {
            sprintf(buf, "../data/CameraParam%d.yml", i);
            cv::FileStorage fs(buf, cv::FileStorage::READ);
            fs["intrinsic_matrix"] >> intrinsic_matrix;
            fs["distortion_coeffs"] >> distortion_coeffs;
            fs["rotation_matrix"] >> rotation_matrix;
            fs["translation_vector"] >> translation_vector;
            fs.release();

            Mat p_G = Mat::ones(3, imageSize.width * imageSize.height, CV_64FC1);

            for (int j = 0; j < imageSize.height; j++) {
                for (int k = 0; k < imageSize.width; k++) {
                    p_G.at<double>(0, imageSize.width * j + k) = k;
                    p_G.at<double>(1, imageSize.width * j + k) = j;
                }
            }

            cv::Mat P_G = cv::Mat::ones(4, imageSize.width * imageSize.height, CV_64FC1);
            P_G(cv::Rect(0, 0, imageSize.width * imageSize.height, 3)) = K_G.inv() * p_G;
            P_G(cv::Rect(0, 2, imageSize.width * imageSize.height, 1)) = 0;


            Mat P_GC = Mat::zeros(4, imageSize.width * imageSize.height, CV_64FC1);
            Mat T_GC = Mat::zeros(4, 4, CV_64FC1);

            rotation_matrix.convertTo(rotation_matrix, CV_64FC1);

            for (int l = 0; l < 3; ++l) {
                for (int j = 0; j < 3; ++j) {
                    T_GC.at<double>(l, j) = rotation_matrix.at<double>(l, j);
                }
            }

            for (int m = 0; m < 3; ++m) {
                T_GC.at<double>(m, 3) = translation_vector[m];
            }
            T_GC.at<double>(3, 3) = 1.0;

            T_GC.convertTo(T_GC, CV_64FC1);
            P_G.convertTo(P_G, CV_64FC1);
            P_GC = T_GC * P_G;


            Mat P_GC1 = Mat::zeros(1, imageSize.width * imageSize.height, CV_64FC2);
            std::vector<Mat> channels(2);
            split(P_GC1, channels);
            channels[0] = P_GC(cv::Rect(0, 0, imageSize.width * imageSize.height, 1)) /
                          P_GC(cv::Rect(0, 2, imageSize.width * imageSize.height, 1));
            channels[1] = P_GC(cv::Rect(0, 1, imageSize.width * imageSize.height, 1)) /
                          P_GC(cv::Rect(0, 2, imageSize.width * imageSize.height, 1));


            merge(channels, P_GC1);


            Mat p_GC = cv::Mat::zeros(1, imageSize.width * imageSize.height, CV_64FC2);
            fisheye::distortPoints(P_GC1, p_GC, intrinsic_matrix, distortion_coeffs);

            p_GC.reshape(imageSize.height, imageSize.width);

            Mat p_GC_table = p_GC.reshape(0, imageSize.height);
            std::vector<Mat> p_GC_table_channels(2);
            cv::split(p_GC_table, p_GC_table_channels);

            p_GC_table.convertTo(p_GC_table_32F, CV_32FC2);

            char buf2[64];
            sprintf(buf2, "../data/projectTable%d.yml", i);
            FileStorage fs2(buf2, FileStorage::WRITE);
            if (fs2.isOpened()) {
                fs2 << "projectTable" << p_GC_table_32F;
                fs2.release();
            }
        }
    }

    //Initialize the internal parameter matrix of the virtual camera.
    void init_K_G() {
        int rows = imageSize.height;
        int cols = imageSize.width;
        double dX = 0.1;
        double dY = 0.1;
        double fx = 1 / dX;
        double fy = -1 / dY;
        K_G = Mat::zeros(3, 3, CV_64FC1);
        K_G.at<double>(0, 0) = fx;
        K_G.at<double>(1, 1) = fy;
        K_G.at<double>(0, 2) = cols / 2;
        K_G.at<double>(1, 2) = rows / 2;
        K_G.at<double>(2, 2) = 1.0;
    }

    //Project pictures from four cameras onto the ground.
    void project_to_ground() {
        char buf[64];
        for (int i = 0; i < 4; ++i) {
            sprintf(buf, "../data/projectTable%d.yml", i);
            FileStorage fs(buf, FileStorage::READ);
            fs["projectTable"] >> p_GC_table_32F;
            p_GC_table_32F.reshape(0, imageSize.height);
            p_GC_table_32F.convertTo(p_GC_table_32F, CV_32FC2);
            remap(rawimages[i], imgtemp, p_GC_table_32F, Mat(), INTER_LINEAR);
//            imshow("imgtemp",imgtemp);
//            char filename[64];
//            sprintf(filename, "../genera/generated%d.png", i);
//            imwrite(filename,imgtemp);
            images_GC[i] = imgtemp.clone();
        }
    }

    //Generate top-view image
    void generate_topview_img() {
        img_G = Mat::zeros(imageSize, CV_8UC3);

        images_GC[0].copyTo(img_G, mask_F);

        images_GC[1].copyTo(img_G, mask_R);

        images_GC[2].copyTo(img_G, mask_B);

        images_GC[3].copyTo(img_G, mask_L);


    }

    //Generate picture masks for quick extraction of projected pictures,
    // and store the masks in the data folder in the form of pictures.
    void generate_masks() {
        mask_F = Mat::zeros(imageSize, CV_8UC1);
        mask_R = Mat::zeros(imageSize, CV_8UC1);
        mask_B = Mat::zeros(imageSize, CV_8UC1);
        mask_L = Mat::zeros(imageSize, CV_8UC1);


        std::vector<std::vector<Point>> contour;
        std::vector<Point> pts;
        pts.emplace_back(0, 0);
        pts.emplace_back(145, 80);
        pts.emplace_back(395, 80);
        pts.emplace_back(540, 0);
        contour.push_back(pts);

        drawContours(mask_F, contour, 0, Scalar::all(255), -1);

        pts.clear();
        contour.clear();
        pts.emplace_back(0, 0);
        pts.emplace_back(145, 80);
        pts.emplace_back(145, 620);
        pts.emplace_back(0, 700);
        contour.push_back(pts);
        drawContours(mask_L, contour, 0, Scalar::all(255), -1);


        pts.clear();
        contour.clear();
        pts.emplace_back(0, 700);
        pts.emplace_back(145, 620);
        pts.emplace_back(395, 620);
        pts.emplace_back(540, 700);
        contour.push_back(pts);
        drawContours(mask_B, contour, 0, Scalar::all(255), -1);


        pts.clear();
        contour.clear();
        pts.emplace_back(540, 700);
        pts.emplace_back(395, 620);
        pts.emplace_back(395, 80);
        pts.emplace_back(540, 0);
        contour.push_back(pts);
        drawContours(mask_R, contour, 0, Scalar::all(255), -1);

        char maskName[64];
        sprintf(maskName, "../data/image_mask_F.png");
        imwrite(maskName, mask_F);

        sprintf(maskName, "../data/image_mask_R.png");
        imwrite(maskName, mask_R);

        sprintf(maskName, "../data/image_mask_B.png");
        imwrite(maskName, mask_B);

        sprintf(maskName, "../data/image_mask_L.png");
        imwrite(maskName, mask_L);

    }

    //Read masks from the data folder
    void read_masks() {
        char maskName[64];
        sprintf(maskName, "../data/image_mask_F.png");
        mask_F = imread(maskName, CV_8UC1);

        sprintf(maskName, "../data/image_mask_R.png");
        mask_R = imread(maskName, CV_8UC1);

        sprintf(maskName, "../data/image_mask_B.png");
        mask_B = imread(maskName, CV_8UC1);

        sprintf(maskName, "../data/image_mask_L.png");
        mask_L = imread(maskName, CV_8UC1);
    }


    Mat return_topview_image() {
        return img_G;
    }

    std::vector<Mat> return_images() {
        std::cout << images_GC.size() << std::endl;

        return images_GC;
    }

private:

    Mat K_G;

    Mat p_GC_table_32F;
    Mat img_G;
    std::vector<Mat> rawimages;
    std::vector<Mat> images_GC;
    Mat imgtemp;

    Mat mask_F;
    Mat mask_B;
    Mat mask_R;
    Mat mask_L;

    Matx33d intrinsic_matrix;
    Vec4d distortion_coeffs;
    Mat rotation_matrix;
    Vec3d translation_vector;

    // image size
    Size2i imageSize;
    Size2i shiftSize;
    Size2i vehicleSize;

};


#endif //CALIBRATION_TOPVIEW_HPP
