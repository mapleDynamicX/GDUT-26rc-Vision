#include "./orb/orb_debug.h"
#include "./orb/orb_getpoint.h"

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "test4_node");
//     cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");

//     Ten::ORB::orb_filter of;
//     cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
//     cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
//     of.camerainfo_.set_RT(rvec, tvec);
//     std::cout << "R: " << std::endl;
//     std::cout << of.camerainfo_.R() << std::endl;
//     std::cout << "T: " << std::endl;
//     std::cout << of.camerainfo_.T() << std::endl;

//     // int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
//     // of.set_exist_boxes(exist_box);

//     of.filterValidCorners();

//     Ten::ORB::orb_transform ot;

//     ot.projectValidCorners(of.getCubeList(), of.camerainfo_.K(), of.camerainfo_.distCoeffs(), of.camerainfo_.rvec(), of.camerainfo_.tvec());

//     cv::Mat debug = ot.drawValidPoints(img);
//     cv::imshow("debug", debug);
//     cv::imwrite("/home/maple/study2/ORB_ws3/src/yolo/src/images/image1_a.png", debug);
//     cv::waitKey(0);


//     return 0;
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test4_node");
    cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");

    Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.2, 0.2, 0);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);

    int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
    og.set_exist_boxes(exist_box);
    og.generate_points(rvec, tvec, img);


    cv::Mat debug = og.getdebugimage();
    cv::imshow("debug", debug);
    //cv::imwrite("/home/maple/study2/ORB_ws3/src/yolo/src/images/image1_a.png", debug);
    cv::waitKey(0);


    return 0;
}