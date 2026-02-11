#include "./orb/orb_debug.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test4_node");
    cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");

    Ten::ORB::orb_filter of;
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
    of.camerainfo_.set_RT(rvec, tvec);
    std::cout << "R: " << std::endl;
    std::cout << of.camerainfo_.R() << std::endl;
    std::cout << "T: " << std::endl;
    std::cout << of.camerainfo_.T() << std::endl;

    int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
    of.set_exist_boxes(exist_box);

    of.filterValidCorners();

    Ten::ORB::orb_transform ot;

    ot.projectValidCorners(of.getCubeList(), of.camerainfo_.K(), of.camerainfo_.distCoeffs(), of.camerainfo_.rvec(), of.camerainfo_.tvec());

    cv::Mat debug = ot.drawValidPoints(img);
    cv::imshow("debug", debug);
    cv::waitKey(0);


    return 0;
}

