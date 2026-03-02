#include "./orb/orb_debug.h"
#include "./orb/orb_getpoint.h"
#include "./orb/orb_exhaust.h"
#include "./orb/orb_overall_match.h"
#include "./orb/orb_overall_exhaust.h"

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



// int main(int argc, char** argv)
// {
//     // ros::init(argc, argv, "test4_node");
//     // cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
//     // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
//     // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
//     // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
//     // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
//     // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);

//     // Ten::ORB::orb_exhaust oe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



//     // std::vector<Ten::ORB::orb_exhaust_element> oees;
//     // Ten::ORB::orb_exhaust_element oee;
//     // oee.image_ = img;
//     // oee.rvec_ = rvec;
//     // oee.tvec_ = tvec;
//     // oees.push_back(oee);
//     // oee.image_ = img2;
//     // oee.rvec_ = rvec2;
//     // oee.tvec_ = tvec2;
//     // oees.push_back(oee);
//     // std::vector<int> place = oe.getplace(oees);



//     // std::cout << "place: ";
//     // for(auto it : place)
//     // {
//     //     std::cout << it << ",";
//     // }
//     // std::cout << std::endl;


//     // //检验
//     // int exist_box[12] = {0};
//     // for(int k = 0; k < 12 && k < place.size(); k++)
//     // {
//     //     exist_box[k] = place[k];
//     // }



//     int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
//     cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
//     cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
//     cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
//     Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);
//     og.set_exist_boxes(exist_box);
//     og.generate_points(rvec, tvec, img);

//     // int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
//     // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
//     // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
//     // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);
//     // Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);
//     // og.set_exist_boxes(exist_box);
//     // og.generate_points(rvec2, tvec2, img2);

//     cv::Mat debug = og.getdebugimage();
//     Ten::ORB::orb_match om;
//     om.set_debug_image(debug);
//     double loss = om.getloss(og.getPixelPoints(), og.getyoloPoints());
//     std::cout << "expect_loss: " << loss << std::endl;

    

//     cv::Mat debug2 = om.get_debug_img();

//     cv::imshow("debug", debug2);
//     cv::imwrite("/home/maple/study2/ORB_ws6/src/yolo/src/images/image4.png", debug2);
//     cv::waitKey(0);

//     return 0;
// }




// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "test4_node");
//     cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
//     cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
//     cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
//     cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
//     cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
//     cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);

//     Ten::ORB::orb_optimize_exhaust ooe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



//     std::vector<Ten::ORB::orb_exhaust_element> oees;
//     Ten::ORB::orb_exhaust_element oee;
//     oee.image_ = img;
//     oee.rvec_ = rvec;
//     oee.tvec_ = tvec;
//     oees.push_back(oee);
//     oee.image_ = img2;
//     oee.rvec_ = rvec2;
//     oee.tvec_ = tvec2;
//     oees.push_back(oee);
//     std::vector<int> place = ooe.getplace(oees);



//     std::cout << "place: ";
//     for(auto it : place)
//     {
//         std::cout << it << ",";
//     }
//     std::cout << std::endl;


//     //检验
//     int exist_box[12] = {0};
//     for(int k = 0; k < 12 && k < place.size(); k++)
//     {
//         exist_box[k] = place[k];
//     }



//     // int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
//     // cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
//     // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
//     // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
//     Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);
//     og.set_exist_boxes(exist_box);
//     og.generate_points(rvec, tvec, img);

//     // int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
//     // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
//     // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
//     // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);
//     // Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);
//     // og.set_exist_boxes(exist_box);
//     // og.generate_points(rvec2, tvec2, img2);

//     cv::Mat debug = og.getdebugimage();
//     Ten::ORB::orb_optimize_match oom;
//     Ten::ORB::orb_optimize_rt oort;
//     oom.set_debug_image(debug);
//     oort.optimize_rt(oom.get_optimize_match(og.getPixelPoints(), og.getyoloPoints()));
//     cv::Mat debug2 = oom.get_debug_img();
//     cv::imshow("debug_before", debug2);

//     og.generate_points(oort.camera_info_.rvec(), oort.camera_info_.tvec(), img);
//     cv::Mat debug3 = og.getdebugimage();
//     Ten::ORB::orb_match om;
//     om.set_debug_image(debug3);
//     double loss = om.getloss(og.getPixelPoints(), og.getyoloPoints());
//     std::cout << "expect_loss: " << loss << std::endl;
//     cv::Mat debug4 = om.get_debug_img();
//     cv::imshow("debug_last", debug4);
    
//     // cv::imwrite("/home/maple/study2/ORB_ws7/src/yolo/src/images/image_before.png", debug2);
//     // cv::imwrite("/home/maple/study2/ORB_ws7/src/yolo/src/images/image_last.png", debug4);
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

    Ten::ORB::orb_optimize_exhaust ooe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



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
    std::vector<int> place = ooe.getplace(oees);



    std::cout << "place: ";
    for(auto it : place)
    {
        std::cout << it << ",";
    }
    std::cout << std::endl;


    std::vector<double> losses = ooe.get_loss();
    for(size_t i = 0; i < losses.size(); i++)
    {
        std::cout << "loss[" << i << "]: " << losses[i] << std::endl;
    }

    std::vector<Ten::ORB::orb_exhaust_element> oprt = ooe.get_RT(oees);

    //检验
    int exist_box[12] = {0};
    for(size_t k = 0; k < 12 && k < place.size(); k++)
    {
        exist_box[k] = place[k];
    }


    Ten::ORB::orb_getpoint og("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);

    for(size_t i = 0; i < oprt.size(); i++)
    {
        og.set_exist_boxes(exist_box);
        // std::cout<< "r: " << oprt[i].rvec_ << std::endl;
        // std::cout<< "t: " << oprt[i].tvec_ << std::endl;
        og.generate_points(oprt[i].rvec_, oprt[i].tvec_, oprt[i].image_, 0);
        cv::Mat debug = og.getdebugimage();
        Ten::ORB::orb_match om;
        om.set_debug_image(debug);
        double loss = om.getloss(og.getPixelPoints(), og.getyoloPoints());
        std::cout << "expect_loss: " << loss << std::endl;
        cv::Mat debug2 = om.get_debug_img();
        cv::imshow("debug", debug2);
        cv::waitKey(0);
    }

    return 0;
}