#include <iostream>
#include <opencv4/opencv2/imgcodecs.hpp>
#include "calibrationtools.hpp"

using namespace cv;

int main() {
    std::vector<Mat> images;
    Mat image;
    int imgNumber;
    imgNumber = 9;
    std::string img_path = "../pict/";
    for (int i = 0; i < imgNumber; ++i) {
        std::string img_name = img_path + "left-00" + std::to_string(i) + ".png";
        image = imread(img_name, IMREAD_COLOR);
        images.push_back(image);
    }

    CalibrationTool tools = CalibrationTool(images, 0, imgNumber, 0);

    imshow("test01", images[0]);

    Matx33d result = tools.getIntrinsicMatrix();
    std::cout << result << std::endl;

    return 0;
}
