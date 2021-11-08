#include"fisheye_stitching/Fisheye.h"

int extract_index_vector(const vector<cv::Point2f>& vec, const cv::Point2f& pt)//д��const��Ϊ�˷�ֹ�ı�
{

	auto it = find(vec.begin(), vec.end(), pt);
	auto is = vec.begin();
	int index = int(it - is);
	return index;
}

void get_landmarks_after_warping(const TYPE T, const vector<cv::Mat>& warpingTemplates, vector<cv::Point>& landmark_points)
{
	vector<cv::Scalar>colorList;
	get_colorList(colorList);
	cv::Mat img;
	if (T == TYPE::L)
		img = warpingTemplates[0];
	else if (T == TYPE::M)
		img = warpingTemplates[1];
	else
	{
		img = warpingTemplates[2];
	}
	vector<cv::Point> landmark_points_row;
	for (int i = 0; i < 15; ++i)
	{
		//���ﲻ����pushback ����vector���
		landmark_points_row.clear();
		get_each_row_coordinates(img, colorList[i], landmark_points_row);
		landmark_points.insert(landmark_points.end(), landmark_points_row.begin(), landmark_points_row.end());
	}
	// if(step==200)
	// 	assert(landmark_points.size() == 72);
	// else if(step==100)
	//assert(landmark_points.size()==255);
	return;
}

bool sort_coordinates(const cv::Point& a, const cv::Point& b)
{
	return a.x < b.x;
}

void get_each_row_coordinates(const cv::Mat& img, const cv::Scalar& color, vector<cv::Point>& landmark_points)
{
	cv::Mat mask_color;
	cv::inRange(img, color, color, mask_color);
	/*cv::imshow("mask_color", mask_color);
	cv::waitKey(0);*/
	vector < vector<cv::Point>>contours;//����findContours����ʱ ������cv::Point ������cv::Point2f
	cv::findContours(mask_color, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	//https://blog.csdn.net/zhuoyueljl/article/details/53588271
	vector<cv::Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	vector<cv::Point2f> mc(contours.size());
	int cx, cy;
	for (int i = 0; i < contours.size(); i++)
	{
		cx = int(mu[i].m10 / mu[i].m00);
		if (cx < 1)//filter the bad result which caused by tiny points
			continue;
		cy = int(mu[i].m01 / mu[i].m00);

		landmark_points.push_back(cv::Point(cx, cy));
	}
	//sort them from left to right
	//https://blog.csdn.net/zhouxun623/article/details/49887555

	sort(landmark_points.begin(), landmark_points.end(), sort_coordinates);
	/*cv::namedWindow("img", cv::WINDOW_NORMAL);
	for (auto& c : landmark_points)
	{
		cv::circle(img, c, 5, cv::Scalar(255, 255, 255), -1);
		cv::imshow("img", img);
		cv::waitKey(0);
	}*/
	return;
}

void create_corresponding_landmarks(const int height, const int width, const int step,const TYPE T,
	const vector<cv::Mat>& warpingTemplates, vector<cv::Point>& landmarks_points,
	vector<cv::Point>& landmarks_points2, vector<vector<int>>& indexes_triangles)
{
	//CV_8UC1 , as mask has only one channel
	//CV_Assert(warpingTemplates.size() == 3);
	cv::Mat mask = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);

	//每行有17个点 每列15点 所以列方向上间距是(height-1)/14 行方向上的间距是(width-1)/16
	//each row has 17 points, each col has 15 points, so the step on col direction is (height-1)/14 ans step on row direction is (width-1)/16
	int intervalNumVertical=0;
	int intervalNumHorizontal=0;
	
	if(step ==100)
	{
		intervalNumVertical=14;
		intervalNumHorizontal=16;
	}
	else{

		intervalNumVertical=7;
		intervalNumHorizontal=8;
	}


	for (int j = 0; j < height; j += (height-1)/intervalNumVertical)
		for (int i = 0; i < width; i += (width-1)/intervalNumHorizontal)
		{
			landmarks_points.push_back(cv::Point(i, j));
		}
		
	cout << "landmarksPoints.size() == " << landmarks_points.size() << endl;


	vector<cv::Point> hull;//https://www.cnblogs.com/mikewolf2002/p/3422197.html
	cv::convexHull(landmarks_points, hull);
	//cv::polylines(img1, hull, true, cv::Scalar(255, 0, 0), 3);
	cv::Rect rect = cv::boundingRect(hull);
	cv::Subdiv2D subdiv2D;
	subdiv2D.initDelaunay(rect);
	//��vector<Point>����vector<Point2f> �൱��ǿ������ת��
	const vector<cv::Point2f> landmarks_points_f(landmarks_points.cbegin(), landmarks_points.cend());
	subdiv2D.insert(landmarks_points_f);//����opencv�ĵ� ������Point2f���� https://www.ccoderun.ca/programming/doxygen/opencv/classcv_1_1Subdiv2D.html#a37223a499032ef57364f1372ad0c9c2e

	//https://cloud.tencent.com/developer/article/1165773

	vector<cv::Vec6f>triangles;
	subdiv2D.getTriangleList(triangles);

	cv::Point2f pt1;
	cv::Point2f pt2;
	cv::Point2f pt3;
	int index_pt1, index_pt2, index_pt3;
	vector<int> triangle;

	for (auto t : triangles)
	{
		pt1 = cv::Point2f(t[0], t[1]);
		pt2 = cv::Point2f(t[2], t[3]);
		pt3 = cv::Point2f(t[4], t[5]);
		/*cout << pt1 << endl;
		cout << pt2 << endl;
		cout << pt3 << endl;*/
		index_pt1 = extract_index_vector(landmarks_points_f, pt1);
		index_pt2 = extract_index_vector(landmarks_points_f, pt2);
		index_pt3 = extract_index_vector(landmarks_points_f, pt3);
		/*cout << "index_pt1 = " << index_pt1 << endl;
		cout << "index_pt2 = " << index_pt2 << endl;
		cout << "index_pt3 = " << index_pt3 << endl;*/
		triangle = { index_pt1,index_pt2,index_pt3 };
		indexes_triangles.push_back(triangle);
	}
	cout << "len(indexes_triangles) = " << indexes_triangles.size() << endl;

	get_landmarks_after_warping(T, warpingTemplates, landmarks_points2);
	cout << "len(landmarks_points) = " << landmarks_points.size() << endl;
	cout << "len(landmarks_points2) = " << landmarks_points2.size() << endl;

	assert(landmarks_points.size() == landmarks_points2.size());
	/*cv::imshow("img1", img1);
	cv::waitKey(0);*/
	return;
}

void get_colorList(vector<cv::Scalar>& colorList)
{
	const cv::Scalar color0(0, 0, 255);
	const cv::Scalar color1(0, 97, 255);
	const cv::Scalar color2(0, 255, 255);
	const cv::Scalar color3(0, 255, 0);
	const cv::Scalar color4(255, 255, 0);
	const cv::Scalar color5(255, 0, 0);
	const cv::Scalar color6(240, 32, 160);
	const cv::Scalar color7(203, 192, 255);
	const cv::Scalar color8(87, 38, 135);
	const cv::Scalar color9(179, 222, 245);
	const cv::Scalar color10(30, 105, 210);
	const cv::Scalar color11(143, 143, 188);
	const cv::Scalar color12(140, 199, 0);
	const cv::Scalar color13(201, 252, 189);
	const cv::Scalar color14(221, 160, 221);

	colorList = { color0,color1,color2,color3,color4,color5,
		color6,color7,color8,color9,color10,color11,color12,color13,color14 };
	return;
}

void warp_image(const cv::Mat& src, cv::Mat& dst, const vector<cv::Point>& landmarks_points,
	const vector<cv::Point>& landmarks_points2, const vector<vector<int>>& indexes_triangles)
{

	cv::Point tr1_pt1, tr1_pt2, tr1_pt3, tr2_pt1, tr2_pt2, tr2_pt3;
	vector<cv::Point2f> triangle1, triangle2;
	cv::Rect rect1, rect2;
	int x, y, w, h;
	cv::Mat cropped_triangle1, cropped_triangle2;
	cv::Mat cropped_triangle1_mask, cropped_triangle2_mask;
	vector<cv::Point> points1, points2;
	cv::Mat M;//���ﲻ���ȶ���óߴ������ ����getAffineTransform���Զ�����ߴ������

	cv::Mat warped_triangle, dst_new_rect_area, dst_new_rect_area_gray, mask_triangles_designed;

	cv::Mat cropped_triangle1_masked, warped_triangle_masked, warped_triangle_masked2;//���ڴ��mask�������ͼƬ
	for (const auto& c : indexes_triangles)
	{
		//���ڴ��mask�������ͼƬ��ÿ�α�����գ������Ӱ����һ��ѭ��
		cropped_triangle1_masked.release();
		warped_triangle_masked.release();
		warped_triangle_masked2.release();

		tr1_pt1 = landmarks_points[c[0]];
		tr1_pt2 = landmarks_points[c[1]];
		tr1_pt3 = landmarks_points[c[2]];
		triangle1 = { tr1_pt1,tr1_pt2,tr1_pt3 };
		rect1 = cv::boundingRect(triangle1);
		x = rect1.x;
		y = rect1.y;
		w = rect1.width;
		h = rect1.height;

		cropped_triangle1 = src(rect1);//ROI

		/*cv::imshow("src", src);
		cv::imshow("cropped_triangle1", cropped_triangle1);
		cv::waitKey(0);*/
		cropped_triangle1_mask = cv::Mat::zeros(cropped_triangle1.size(), CV_8UC1);//mask has to be one channel CV_8UC1 not CV_8UC3
		points1 = {
			cv::Point(tr1_pt1.x - x,tr1_pt1.y - y),
			cv::Point(tr1_pt2.x - x,tr1_pt2.y - y),
			cv::Point(tr1_pt3.x - x,tr1_pt3.y - y)
		};

		//fillConvexPoly��points1ֻ�ܽ���cv::Point����
		cv::fillConvexPoly(cropped_triangle1_mask, points1, 255);//cropped_triangle1_mask has only 1 channel so the color applied on it should also contain only 1 channel

		//You are wrong if u write as follows:
		//cv::bitwise_and(cropped_triangle1, cropped_triangle1,cropped_triangle1, cropped_triangle1_mask);��ôд��� һ��Ҫ�ȿ��ٳ����������ͼ��

		//cv::imshow("cropped_triangle1_after_masking", cropped_triangle1_masked);
		//https://stackoverflow.com/questions/11532924/opencv-bitwise-and-mask
		//dont need to use cv::bitwise_and as in python video and use copyTo instead

		cropped_triangle1.copyTo(cropped_triangle1_masked, cropped_triangle1_mask);

		tr2_pt1 = landmarks_points2[c[0]];
		tr2_pt2 = landmarks_points2[c[1]];
		tr2_pt3 = landmarks_points2[c[2]];

		triangle2 = { tr2_pt1,tr2_pt2,tr2_pt3 };
		rect2 = cv::boundingRect(triangle2);
		x = rect2.x;
		y = rect2.y;
		w = rect2.width;
		h = rect2.height;

		cropped_triangle2 = dst(rect2);
		cropped_triangle2_mask = cv::Mat::zeros(cropped_triangle2.size(), CV_8UC1);

		points2 = {
			cv::Point(tr2_pt1.x - x,tr2_pt1.y - y),
			cv::Point(tr2_pt2.x - x,tr2_pt2.y - y),
			cv::Point(tr2_pt3.x - x,tr2_pt3.y - y)
		};
		cv::fillConvexPoly(cropped_triangle2_mask, points2, 255);
		//ǿ������ת��
		vector<cv::Point2f> points1f(points1.begin(), points1.end());
		vector<cv::Point2f>points2f(points2.begin(), points2.end());
		//getAffineTransformֻ����cv::Point2f���� ������cv::Point
		M = cv::getAffineTransform(points1f, points2f);
		//cout << "after M.size() = " << M.size() << endl;
		warped_triangle_masked.release();
		cv::warpAffine(cropped_triangle1, warped_triangle, M, cv::Size(w, h), cv::INTER_NEAREST);
		warped_triangle.copyTo(warped_triangle_masked, cropped_triangle2_mask);//do not copy to itself! need a new Mat to storage dstMat

		dst_new_rect_area = dst(rect2);
		cv::cvtColor(dst_new_rect_area, dst_new_rect_area_gray, cv::COLOR_BGR2GRAY);
		cv::threshold(dst_new_rect_area_gray, mask_triangles_designed, 1, 255, cv::THRESH_BINARY_INV);

		//U r wrong if u write it as following: src and dst in cv::bitwise_and must not be same!
		//cv::bitwise_and(warped_triangle_masked, warped_triangle_masked, warped_triangle_masked, mask_triangles_designed);
		warped_triangle_masked.copyTo(warped_triangle_masked2, mask_triangles_designed);
		cv::add(dst_new_rect_area, warped_triangle_masked2, dst_new_rect_area);
		dst(rect2) = dst_new_rect_area;
	}
	return;
}

vector<cv::Mat> get_mask_templates(const string& path1, const string& path2, const string& path3)
{
	cv::Mat temp;
	vector<cv::Mat> maskTemplates;
	//mind the flag! have to read as grayscale image!
	temp = generate_mask(path1);
	maskTemplates.push_back(temp);
	temp = generate_mask(path2);
	maskTemplates.push_back(temp);
	temp = generate_mask(path3);
	maskTemplates.push_back(temp);
	
	return maskTemplates;
}

void panorama(const vector<cv::Mat>& maskTemplates,
	const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& img3, cv::Mat& output)
{
	
	cv::Mat mask_L_and_R = ~maskTemplates[1];
	/*cv::imshow("mask1", maskTemplates[0]);
	cv::imshow("mask2", maskTemplates[1]);
	cv::imshow("mask3", maskTemplates[2]);*/
	cv::Mat img1_temp, img2_temp, img3_temp;
	img1.copyTo(img1_temp, maskTemplates[0]);
	/*cv::imshow("img1", img1);
	cv::imshow("img1_temp", img1_temp);
	cv::waitKey(0);*/
	img2.copyTo(img2_temp, maskTemplates[1]);
	img3.copyTo(img3_temp, maskTemplates[2]);
	cv::Mat img1_3 = img1_temp + img3_temp;
	cv::Mat img1_3_temp;
	img1_3.copyTo(img1_3_temp, mask_L_and_R);
	output = img1_3_temp + img2_temp;
	//    img = img[200:1080,150:1870] for project 11(72 landmarks)
	//     img = img[120:1120,50:1950] for project 10(255 landmarks)
	output = output(cv::Range(120,1120), cv::Range(50,1950));

}

cv::Mat generate_mask(const string& path)
{
	const cv::Mat img = cv::imread(path);
	if (img.empty())
	{
		cerr << "can not load mask templates!";
		throw;
	}
	cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);

	const cv::Scalar seamColor(81, 121, 151);
	cv::Mat maskOfSeam;
	cv::inRange(img, seamColor, seamColor, maskOfSeam);

	vector<vector<cv::Point>>contours;
	cv::findContours(maskOfSeam, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
	cv::drawContours(mask, contours, 1, cv::Scalar(255, 255, 255), -1);

	return mask;
}

vector<cv::Mat> get_warping_templates(const string& path1, const string& path2, const string& path3)
{
	//common error for vector if writing like this:
	/*warpingTemplates[0] = cv::imread(path1);
	warpingTemplates[1] = cv::imread(path2);
	warpingTemplates[2] = cv::imread(path3);*/
	cv::Mat temp;
	vector<cv::Mat> warpingTemplates;
	temp = cv::imread(path1);

	warpingTemplates.push_back(temp);
	temp = cv::imread(path2);
	warpingTemplates.push_back(temp);
	temp = cv::imread(path3);
	warpingTemplates.push_back(temp);
	if (warpingTemplates[0].empty() || warpingTemplates[1].empty() || warpingTemplates[2].empty())
	{
		cerr << "can not load warping templates!" << endl;
		throw;
	}
	return warpingTemplates;
}

void get_destination_shape(const string& path, int& h, int& w)
{
	cv::Mat img = cv::imread(path);
	h = img.rows;
	w = img.cols;
}

void warp_video(const vector<cv::Mat>& warpingTemplates, 
const vector<cv::Mat>& maskTemplates,const int step)
{
	ros::NodeHandle nh;
	ros::Publisher frame_rate_publisher = nh.advertise<std_msgs::String>("/fisheye_pano/frame_rate",10);
	ros::Publisher mega_byte_publisher=nh.advertise<std_msgs::String>("/fisheye_pano/mega_byte_per_sec",10);
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pano_publisher =it.advertise("/fisheye_pano",10);
	// ros::Rate loop_rate(1000);
	ros::Subscriber fisheye_L_sub,fisheye_M_sub,fisheye_R_sub;
	std::string fisheye_L_topic = "/sensors/camera/fisheye_left/image_raw/compressed";
	std::string fisheye_M_topic = "/sensors/camera/fisheye_middle/image_raw/compressed";
	std::string fisheye_R_topic = "/sensors/camera/fisheye_right/image_raw/compressed";

	// fisheye_L_sub =nh.subscribe(fisheye_L_topic,10,);

	//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
	
	std_msgs::String frame_rate_msg;
	std_msgs::String mega_byte_per_sec_msg;

	sensor_msgs::ImageConstPtr image_msg;

	Listener listener;
	fisheye_L_sub=nh.subscribe(fisheye_L_topic,10,&Listener::callback_L,&listener);
	fisheye_M_sub=nh.subscribe(fisheye_M_topic,10,&Listener::callback_M,&listener);
	fisheye_R_sub=nh.subscribe(fisheye_R_topic,10,&Listener::callback_R,&listener);

	string path0=ros::package::getPath("fisheye_stitching")+"/src";

	//for video recording only
	//--------------------------------------------
	// int myFourCC = cv::VideoWriter::fourcc('m', 'p', '4', 'v');//mp4
	// //note that the output video size must be same as trimmed frame_pano size ,otherwise video fails to be written!
	// //frame rate should set to same as input rate
	// cv::VideoWriter output(ros::package::getPath("fisheye_stitching")+"/output.mp4", myFourCC, 10, cv::Size(1900, 1000));
	// //------------------------------------------------

	//buffer area for loading 3 images
	while (ros::ok())
	{
		int initialFrameHeight1=0;
		int initialFrameWidth1=0;
		int initialFrameHeight2=0;
		int initialFrameWidth2=0;
		int initialFrameHeight3=0;
		int initialFrameWidth3=0;
		//buffer area for loading 3 fisheye images
		while (ros::ok())
		{
			if(listener.m_frame1.empty()||listener.m_frame2.empty()||listener.m_frame3.empty())
				{
					ROS_INFO("loading images...");
				}
			else
				{	
					initialFrameHeight1=listener.m_frame1.rows;
					initialFrameWidth1=listener.m_frame1.cols;

					initialFrameHeight2=listener.m_frame2.rows;
					initialFrameWidth2=listener.m_frame2.cols;

					initialFrameHeight3=listener.m_frame3.rows;
					initialFrameWidth3=listener.m_frame3.cols;

					ROS_INFO("images received! program starts!");
					
					break;
				}
			ros::spinOnce();
		}

		vector<cv::Point> landmarks_points_L, landmarks_points_M, landmarks_points_R;
		vector<cv::Point>landmarks_points2_L, landmarks_points2_M, landmarks_points2_R;
		vector<vector<int>>indexes_triangles_L, indexes_triangles_M, indexes_triangles_R;
		create_corresponding_landmarks(initialFrameHeight1, initialFrameWidth1, step,TYPE::L, warpingTemplates,
			landmarks_points_L, landmarks_points2_L, indexes_triangles_L);
		create_corresponding_landmarks(initialFrameHeight2, initialFrameWidth2, step,TYPE::M, warpingTemplates,
			landmarks_points_M, landmarks_points2_M, indexes_triangles_M);
		create_corresponding_landmarks(initialFrameHeight3, initialFrameWidth3, step,TYPE::R, warpingTemplates,
			landmarks_points_R, landmarks_points2_R, indexes_triangles_R);
		int h2, w2;
		h2 = warpingTemplates[0].rows;
		w2 = warpingTemplates[0].cols;
		//frame1_warped,frame2_warped,frame3_warped,frame_pano have to be specified with cv::Size() when created! 
		cv::Mat	frame1_warped = cv::Mat::zeros(cv::Size(w2, h2), CV_8UC3);
		cv::Mat	frame2_warped = cv::Mat::zeros(cv::Size(w2, h2), CV_8UC3);
		cv::Mat	frame3_warped = cv::Mat::zeros(cv::Size(w2, h2), CV_8UC3);

		cv::Mat frame_pano(h2, w2, CV_8UC3);

#if YOLO
		string yoloClassesFile = path0+"/coco.names";
		string yoloModelConfiguration = path0+"/yolov3.cfg";
		string yoloModelWeights = path0+"/yolov3.weights";

		vector<string> classes1,classes2,classes3;
		ifstream ifs(yoloClassesFile.c_str());
		string line;
		while (getline(ifs, line))
		{
			classes1.push_back(line);
		}
		classes2 = classes1;
		classes3 = classes1;

		//must define 3 nets for 3 images
		cv::dnn::Net net1 = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
		net1.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		net1.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		cv::dnn::Net net2 = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
		net2.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		net2.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		cv::dnn::Net net3 = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
		net3.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		net3.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		
		double scalefactor = 1 / 255.0;
		cv::Size size = cv::Size(416, 416);
		cv::Scalar mean = cv::Scalar(0, 0, 0);
		bool swapRB = false;
		bool crop = false;
		cv::Mat blob1,blob2,blob3;

		vector<cv::String>names1;
		vector<int>outLayers = net1.getUnconnectedOutLayers();
		vector<cv::String>layerNames = net1.getLayerNames();
		names1.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
		{
			names1[i] = layerNames[outLayers[i] - 1];
		}
		vector<cv::String>names2 = names1,names3=names1;
		vector<cv::Mat>netOutput1,netOutput2,netOutput3;
		
		vector<int> classIds1,classIds2,classIds3;
		vector<float> confidences1,confidences2,confidences3;
		const float nmsThreshold = 0.2f;
		vector<int>indices1,indices2,indices3;
		vector<BoundingBox>bBoxes1,bBoxes2,bBoxes3;
		float confThreshold = 0.40f;
		vector<cv::Rect> boxes1,boxes2,boxes3;
#endif
				
		//now the panorama program starts
		int frame_cnt=0;
		auto start = cv::getTickCount();

		//-----------------------------------
		//for exposure compensation
		cv::Point point1(0, 0);
		cv::Point point2(0, 0);
		cv::Point point3(0, 0);
		vector<cv::Point> corners{ point1,point2,point3};
		cv::Ptr<cv::detail::ExposureCompensator> compensator
			= cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN);
		cv::UMat mask1, mask2, mask3;
		vector<cv::UMat>masks{ mask1,mask2,mask3};

		for (int i = 0; i < masks.size(); ++i)
		{
			masks[i].create(warpingTemplates[0].size(), CV_8U);
			masks[i].setTo(cv::Scalar::all(255));
		}
		//------------------------------------


		while (ros::ok())
		{	
			//get currentFrameHeight, if not equal to initialFrameHeight then break the loop and start over from create_corresponding_landmarks() again
			int currentFrameHeight1=listener.m_frame1.rows;
			int currentFrameWidth1=listener.m_frame1.cols; 

			int currentFrameHeight2=listener.m_frame2.rows;
			int currentFrameWidth2=listener.m_frame2.cols; 
			
			int currentFrameHeight3=listener.m_frame3.rows;
			int currentFrameWidth3=listener.m_frame3.cols; 

			// int subSampleRate;
			// nh.getParam("",subSampleRate);
			// cout<<"subSampleRate = "<<subSampleRate<<endl;

			if (currentFrameWidth1!=initialFrameWidth1||currentFrameWidth2!=initialFrameWidth2||currentFrameWidth3!=initialFrameWidth3)
			{
				ROS_INFO("Frame size changed! Program restarting!");
				
				break;
			}

			frame1_warped.setTo(cv::Scalar::all(0));
			frame2_warped.setTo(cv::Scalar::all(0));
			frame3_warped.setTo(cv::Scalar::all(0));

			
			std::thread t1(warp_image, ref(listener.m_frame1), ref(frame1_warped),
				ref(landmarks_points_L), ref(landmarks_points2_L), ref(indexes_triangles_L));
			std::thread t2(warp_image, ref(listener.m_frame2), ref(frame2_warped),
				ref(landmarks_points_M), ref(landmarks_points2_M), ref(indexes_triangles_M));
			std::thread t3(warp_image, ref(listener.m_frame3), ref(frame3_warped),
				ref(landmarks_points_R), ref(landmarks_points2_R), ref(indexes_triangles_R));

			t1.join();
			t2.join();
			t3.join();
		
			panorama(maskTemplates,
				frame1_warped, frame2_warped, frame3_warped, frame_pano);

		

			//exposure compensation!
			//exposure_compensate(frame1_warped, frame2_warped, frame3_warped,corners,masks,compensator);
			
			
			auto end = cv::getTickCount();
			auto totalTime = ( end - start ) / cv::getTickFrequency();
			auto fps = frame_cnt/totalTime;
			//histogram equalizier
			//equalizing_hist(frame_pano);



			size_t megaBytes_per_sec = (frame_pano.total() * frame_pano.elemSize())*fps/1000000.0;
			cv::putText(frame_pano, "MB/s: " + to_string(megaBytes_per_sec), cv::Point(100, 200), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 0), 3);

			//cv::putText(frame_pano, to_string(landmarks_amount) + " landmarks", cv::Point(100, 100), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 0), 3);
			cv::putText(frame_pano, "FPS: " + to_string(int(fps)), cv::Point(100, 100), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 0), 3);
			//cv::imshow("frame_pano", frame_pano);
			//output.write(frame_pano);
			//publish frame rate
			frame_rate_msg.data=to_string(int(fps));
			ROS_INFO("FPS: %s",frame_rate_msg.data.c_str());
			frame_rate_publisher.publish(frame_rate_msg);
			//publish mega byte per sec
			mega_byte_per_sec_msg.data=to_string(megaBytes_per_sec);
			ROS_INFO("MB/s: %s",mega_byte_per_sec_msg.data.c_str());
			mega_byte_publisher.publish(mega_byte_per_sec_msg);

			//publish videostream
			image_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame_pano).toImageMsg();
			pano_publisher.publish(image_msg);
			ROS_INFO("frame_pano is being published");
	#if YOLO
			std::thread y1(yolo_detection, ref(frame1_warped), ref(frame_pano), ref(netOutput1), ref(classIds1), ref(confidences1),
				ref(indices1), ref(bBoxes1), ref(scalefactor), ref(size), ref(boxes1), ref(blob1), ref(mean), ref(swapRB), ref(crop), ref(net1), ref(names1), ref(classes1), ref(confThreshold), ref(nmsThreshold));
			
			std::thread y2(yolo_detection, ref(frame2_warped), ref(frame_pano), ref(netOutput2), ref(classIds2), ref(confidences2),
				ref(indices2), ref(bBoxes2), ref(scalefactor), ref(size), ref(boxes2), ref(blob2), ref(mean), ref(swapRB), ref(crop), ref(net2), ref(names2), ref(classes2), ref(confThreshold), ref(nmsThreshold));

			std::thread y3(yolo_detection, ref(frame3_warped), ref(frame_pano), ref(netOutput3), ref(classIds3), ref(confidences3),
				ref(indices3), ref(bBoxes3), ref(scalefactor), ref(size), ref(boxes3), ref(blob3), ref(mean), ref(swapRB), ref(crop), ref(net3), ref(names3), ref(classes3), ref(confThreshold), ref(nmsThreshold));

			
			y1.join();
			y2.join();
			y3.join();
			
	#endif
			++frame_cnt;
			ros::spinOnce();
			// if(cv::waitKey(1)=='s')
			// {
			// 	cv::imwrite("L_warped.png",frame1_warped);
			// 	cv::imwrite("M_warped.png",frame2_warped);
			// 	cv::imwrite("R_warped.png",frame3_warped);
			// }			
		}
	}
	
	

}

	
void yolo_detection(const cv::Mat& img,const cv::Mat&frame_pano, vector<cv::Mat>&netOutput, vector<int> &classIds, 
	vector<float>& confidences, vector<int>&indices, vector<BoundingBox>&bBoxes, 
	const double scalefactor,const cv::Size& size, vector<cv::Rect> &boxes, cv::Mat& blob,
	const cv::Scalar& mean,const bool swapRB,const bool crop, cv::dnn::Net& net, const vector<cv::String>&names,
	const vector<string>& classes,const float confThreshold, const float nmsThreshold)
{
		netOutput.clear();
		classIds.clear();
		confidences.clear();
		indices.clear();
		bBoxes.clear();
		boxes.clear();
		cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);
		net.setInput(blob);
		net.forward(netOutput, names);

		
		for (size_t i = 0; i < netOutput.size(); ++i)
		{
			float* data = (float*)netOutput[i].data;
			for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols)
			{
				cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
				cv::Point classId;
				double confidence;

				cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
				if (confidence > confThreshold)
				{
					cv::Rect box; int cx, cy;
					cx = (int)( data[0] * img.cols );
					cy = (int)( data[1] * img.rows );
					box.width = (int)( data[2] * img.cols );
					box.height = (int)( data[3] * img.rows );
					box.x = cx - box.width / 2;
					box.y = cy - box.height / 2;

					boxes.push_back(box);
					classIds.push_back(classId.x);
					confidences.push_back((float)confidence);
				}
			}
		}
		//https://blog.csdn.net/cgcoder/article/details/7367965
		
		cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
		
		for (auto it = indices.begin(); it != indices.end(); ++it)
		{
			BoundingBox bBox;
			bBox.roi = boxes[*it];
			bBox.classID = classIds[*it];
			bBox.confidence = confidences[*it];
			bBox.boxID = (int)bBoxes.size();
			bBoxes.push_back(bBox);
		}

		for (auto it = bBoxes.begin(); it != bBoxes.end(); ++it)
		{
			if (it->classID == 0)
			{
				int top, left, width, height;
				top = ( *it ).roi.y;
				left = ( *it ).roi.x;
				width = ( *it ).roi.width;
				height = ( *it ).roi.height;

				cv::rectangle(frame_pano, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);

				string label = cv::format("%.2f", ( *it ).confidence);
				label = classes[( ( *it ).classID )] + ":" + label;

				int baseLine;
				cv::Size labelSize = cv::getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
				top = max(top, labelSize.height);
				//cv::putText(img, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
			}
		}

}

Fisheye::Fisheye()
{
	int step = 100;
	string path1, path2, path3;

	string path0=ros::package::getPath("fisheye_stitching");

	if (step == 100)
	{
		path1 = path0+"/images/warped/255/10/L_warped_10.png";
		path2 = path0+"/images/warped/255/10/M_warped_10.png";
		path3 = path0+"/images/warped/255/10/R_warped_10.png";
	}
	else if (step == 200)
	{
		path1 = path0+"/images/warped/72/11/L_warped_11.png";
		path2 = path0+"/images/warped/72/11/M_warped_11.png";
		path3 = path0+"/images/warped/72/11/R_warped_11.png";
	}
	//���ܴ��յ�Mat��ȥ �ᱨ��
	vector<cv::Mat> warpingTemplates;
	warpingTemplates = get_warping_templates(path1, path2, path3);

	string pathOfLeftMask, pathOfMidMask, pathOfRightMask;
	if (step == 100)
	{
		pathOfLeftMask = path0+"/images/warped/255/10/L_masked_10.png";
		pathOfMidMask = path0+"/images/warped/255/10/M_masked_10.png";
		pathOfRightMask = path0+"/images/warped/255/10/R_masked_10.png";
	}
	else if (step == 200)
	{
		pathOfLeftMask =path0+ "/images/warped/72/11/L_masked_11.png";
		pathOfMidMask = path0+"/images/warped/72/11/M_masked_11.png";
		pathOfRightMask = path0+"/images/warped/72/11/R_masked_11.png";
	}
	vector<cv::Mat> maskTemplates = get_mask_templates(pathOfLeftMask, pathOfMidMask, pathOfRightMask);


	//https://blog.csdn.net/lxlong89940101/article/details/95474529
	
	warp_video(warpingTemplates, maskTemplates,step);
}



#define test 0

void Listener::callback_L(const sensor_msgs::CompressedImageConstPtr&msg)
{
	

	 try
		{
			//Note:do not use clone() here becuase this won't return the image to listener.img_L
			#if test
				cv::imdecode(cv::Mat(msg->data),1).copyTo(temp);
				cv::resize(temp,m_frame1,cv::Size(),resize_factor,resize_factor);
			#else	
				cv::imdecode(cv::Mat(msg->data),1).copyTo(m_frame1);
			#endif
				// ROS_INFO("receving fisheye_left_image");
		}
		catch(cv_bridge::Exception&e)
		{
				cout<<"No image_L!"<<endl;
				ROS_ERROR("cv_bridge exception: %s",e.what());
		}	
}

void Listener::callback_M(const sensor_msgs::CompressedImageConstPtr&msg)
{
	 try
		{
			#if test
				cv::imdecode(cv::Mat(msg->data),1).copyTo(temp);
				cv::resize(temp,m_frame2,cv::Size(),resize_factor,resize_factor);
			#else	
				cv::imdecode(cv::Mat(msg->data),1).copyTo(m_frame2);
			#endif
				// ROS_INFO("receving fisheye_middle_image");
		}
		catch(cv_bridge::Exception&e)
		{
				cout<<"No image_M!"<<endl;
				ROS_ERROR("cv_bridge exception: %s",e.what());
		}	
}

void Listener::callback_R(const sensor_msgs::CompressedImageConstPtr&msg)
{
	 try
		{
			#if test
				cv::imdecode(cv::Mat(msg->data),1).copyTo(temp);
				cv::resize(temp,m_frame3,cv::Size(),resize_factor,resize_factor);
			#else	
				cv::imdecode(cv::Mat(msg->data),1).copyTo(m_frame3);
			#endif
				// ROS_INFO("receving fisheye_right_image");
		}
		catch(cv_bridge::Exception&e)
		{
				cout<<"No image_R!"<<endl;

				ROS_ERROR("cv_bridge exception: %s",e.what());
		}	
}

// void chatterCallback(dynamic_reconfigure/config &config)
// {
//     ROS_INFO("I heard: [%s]", config.c_str());
// }


void exposure_compensate(cv::Mat &src1,cv::Mat&src2,cv::Mat&src3, vector<cv::Point>&corners,
	vector<cv::UMat>& masks
	,cv::Ptr<cv::detail::ExposureCompensator>&compensator)
{/*
	cv::Mat src1 = cv::imread("2/L_warped_bright.png");
	cv::Mat src2=cv::imread("2/M_warped.png");*/
	/*if (src1.empty() || src2.empty())
	{
		cerr << "No Images" << endl;
	}*/
	cv::UMat src1U;
	cv::UMat src2U;
	cv::UMat src3U;
	src1.copyTo(src1U);
	src2.copyTo(src2U);
	src3.copyTo(src3U);

	//int expos_comp_type = cv::detail::ExposureCompensator::GAIN_BLOCKS;
	/*cv::Ptr<cv::detail::ExposureCompensator> compensator
		= cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN);
	*/
	vector<cv::UMat> images{src1U,src2U,src3U};
	
	compensator->feed(corners,images,masks);
	for (int i = 0; i < masks.size(); ++i)
	{
		compensator->apply(i, corners[i], images[i], masks[i]);
	}
	
}

void equalizing_hist(cv::Mat& frame_pano)
{
	vector<cv::Mat> splitBGR(frame_pano.channels());
	cv::split(frame_pano, splitBGR);
	for (int i = 0; i < frame_pano.channels(); ++i)
	{
		cv::equalizeHist(splitBGR[i], splitBGR[i]);
	}
	merge(splitBGR, frame_pano);
}