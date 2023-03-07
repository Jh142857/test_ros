//
// Created by cjh on 23-3-2.
//
#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

void cvTest(){
    // TODO: ros里面怎么使用cv::findContours
    cv::Mat image = cv::imread("/home/cjh/Learn/cppTest/image.jpeg", CV_8UC1);
    std::cout << image.size << std::endl;
    cv::Mat valid_image = image < 101;
    // std::cout << valid_image << std::endl;
    std::vector<std::vector<cv::Point> > test_contour;
    cv::findContours(valid_image, test_contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // std::cout << test_contour[0] << test_contour[1] << std::endl;
    cv::Mat contours = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    std::cout << contours.size << std::endl;
    cv::imshow("hello0", contours);
    for(auto& contour: test_contour){
        for(auto&point: contour) {
            contours.at<char>(point.y, point.x) = -1;
            std::cout << point << std::endl;
        }
    }

    cv::imshow("hello1", contours);
    cv::imshow("hello2", valid_image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    std::cout << "111" << std::endl;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "test_opencv");
    ros::NodeHandle n;
    cvTest();
    ros::spin();
    return 0;
}