#include "./orb/orb_debug.h"
#include "./orb/orb_getpoint.h"
#include "./orb/orb_exhaust .h"

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



// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "test4_node");
//     cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");

//     Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.2, 0.2, 0);
//     cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
//     cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);

//     int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
//     og.set_exist_boxes(exist_box);
//     og.generate_points(rvec, tvec, img);


//     cv::Mat debug = og.getdebugimage();
//     cv::imshow("debug", debug);
//     //cv::imwrite("/home/maple/study2/ORB_ws3/src/yolo/src/images/image1_a.png", debug);
//     cv::waitKey(0);


//     return 0;
// }



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test4_node");
    cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
    cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
    cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);

    // cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.354002, 1.118546, 1.558849);
    // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
    // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.253942, -1.337019, 1.158317);
    // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.048514, 1.452847, -1.638665);

    Ten::ORB::orb_exhaust oe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



    std::vector<Ten::ORB::orb_exhaust_element> oees;
    Ten::ORB::orb_exhaust_element oee;
    oee.image_ = img;
    oee.rvec_ = rvec;
    oee.tvec_ = tvec;
    oees.push_back(oee);
    oee.image_ = img2;
    oee.rvec_ = rvec2;
    oee.tvec_ = tvec2;
    oees.push_back(oee);
    std::vector<int> place = oe.getplace(oees);



    std::cout << "place: ";
    for(auto it : place)
    {
        std::cout << it << ",";
    }
    std::cout << std::endl;


    //检验
    int exist_box[12] = {0};
    for(int k = 0; k < 12 && k < place.size(); k++)
    {
        exist_box[k] = place[k];
    }



    // int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
    // cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);

    Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);
    og.set_exist_boxes(exist_box);
    og.generate_points(rvec, tvec, img);

    cv::Mat debug = og.getdebugimage();
    Ten::ORB::orb_match om;
    om.set_debug_image(debug);
    double loss = om.getloss(og.getPixelPoints(), og.getyoloPoints());
    std::cout << "expect_loss: " << loss << std::endl;

    

    cv::Mat debug2 = om.get_debug_img();

    cv::imshow("debug", debug2);
    //cv::imwrite("/home/maple/study2/ORB_ws3/src/yolo/src/images/image1_a.png", debug);
    cv::waitKey(0);

    return 0;
}