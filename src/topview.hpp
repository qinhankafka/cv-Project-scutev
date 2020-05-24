//
// Created by qinhan on 2020/5/14.
//

#ifndef CALIBRATION_TOPVIEW_HPP
#define CALIBRATION_TOPVIEW_HPP

#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

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

    void initRect() {
        imageRect[0] = Rect(0, 0, shiftSize.width + chessBordSize.width + shiftAdjustSize.width, imageSize.height);
        imageRect[1] = Rect(0, 0, imageSize.width, shiftSize.height + chessBordSize.height + shiftAdjustSize.height);
        imageRect[2] = Rect(imageSize.width - shiftSize.width - chessBordSize.width - shiftAdjustSize.width, 0,
                            shiftSize.width + chessBordSize.width + shiftAdjustSize.width, imageSize.height);
        imageRect[3] = Rect(0, imageSize.height - shiftSize.height - chessBordSize.height - shiftAdjustSize.height,
                            imageSize.width, shiftSize.height + chessBordSize.height + shiftAdjustSize.height);
    }

    void initMask(){
        maskF = Mat(imageRect[1].size(),CV_8UC1,Scalar(1));
        maskB = Mat(imageRect[1].size(),CV_8UC1,Scalar(1));
        std::vector<std::vector<Point>> maskVec;
        maskVec[0].push_back(Point(0,imageRect[1].height));
    }


    Mat transformView() {
        int seq[4] = {0, 2, 1, 3};
        for (int i : seq) {
            warpPerspective(images[i], tempImage, birdTransform[i], mSize);
            switch (i) {
                case 1:
                    tempImage(imageRect[1]).copyTo(topviewImg(imageRect[1]), maskF);
                    break;
                case 3:
                    tempImage(imageRect[3]).copyTo(topviewImg(imageRect[3]), maskB);
                    break;
                default:
                    tempImage(imageRect[i]).copyTo(topviewImg(imageRect[i]));
                    break;
            }
        }
    }

private:
    Rect imageRect[4];
    Rect carPicPos;
    int clickCount;
    int camID;
    int maskHeight;
    Mat topviewImg;
    Mat birdTransform[4];
    Mat maskF;
    Mat maskB;
    Mat vehicleImg;
    Mat vehicleImgtemp;
    Mat tempImage;
    Size imageSize;//output image size (pixes)
    std::vector<Mat> images;
    std::vector<Mat> birdviewImages;
    std::vector<std::vector<Point2f>> targetPoint;
    std::vector<std::vector<Point2f>> sourecePoint;
    Size shiftAdjustSize, shiftSize, chessBordSize, vehicleSize;
    bool SourcePoint_OK, ParamSet_OK;
};


#endif //CALIBRATION_TOPVIEW_HPP
