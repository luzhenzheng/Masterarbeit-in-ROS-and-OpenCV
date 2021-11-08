// test19.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//写代码要一行一行写 写一行运行一下 看看对不对 而不是写了一大段再运行
//all variables and function names are same as in python version
//all functions should return void and use reference as much as possible for effectivity

//mind the path here!!! different as in Windows
#include "fisheye_stitching/Fisheye.h"


int main(int argc, char** argv)
{

	ros::init(argc,argv,"frame_rate_publisher_node");
	Fisheye f;
	// ros::init(argc, argv, "subscriber");
    // ros::NodeHandle n;
 
	// // ros::param::get("")
    // ros::Subscriber sub = n.subscribe("/sensors/camera/ueye_cam_nodelet_fisheye_left/parameter_updates", 100, chatterCallback);
 
    // ros::spin();
    
	return 0;
}


// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
