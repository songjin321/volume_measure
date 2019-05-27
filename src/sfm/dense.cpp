
#include <opencv2/opencv.hpp>
#include "elas.h"
#include <chrono>
using namespace cv;
using namespace std;
// 输入:左右眼的原始图像
// 输出：左右眼校正后的图像
void rectify()
{

}

// 输入:左眼图像，视差图和Q矩阵
// 输出：左眼坐标系下的点云
void generatePointCloud(Mat& img_left, Mat& dmap)
{
	Mat V = Mat(4, 1, CV_64FC1);
	Mat pos = Mat(4, 1, CV_64FC1);

	sensor_msgs::PointCloud2Ptr pc = boost::make_shared<sensor_msgs::PointCloud2>();

	pc->header.frame_id = "camera";
	pc->header.stamp = captured_time;
	pc->width = calib_img_size.height;
	pc->height = calib_img_size.width;
	pc->is_bigendian = false;
	pc->is_dense = false;
	sensor_msgs::PointCloud2Modifier pc_modifier(*pc);
	pc_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*pc, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*pc, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*pc, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc, "b");

//  int valid =0;
	for (int i = 0; i < img_left.cols; i++) {
		for (int j = 0; j < img_left.rows; j++) {
			float d = dmap.at<float>(j, i);
			// if low disparity, then ignore
			// if (d < 3.) {
			// 	continue;
			// }
			// V is the vector to be multiplied to Q to get
			// the 3D homogenous coordinates of the image point
			V.at<double>(0, 0) = (double) (i);
			V.at<double>(1, 0) = (double) (j);
			V.at<double>(2, 0) = (double) d;
			V.at<double>(3, 0) = 1.;
			pos = Q * V; // 3D homogeneous coordinate
			double X = pos.at<double>(0, 0) / pos.at<double>(3, 0);
			double Y = pos.at<double>(1, 0) / pos.at<double>(3, 0);
			double Z = pos.at<double>(2, 0) / pos.at<double>(3, 0);

			int32_t red, blue, green;
			red = img_left.at < Vec3b > (j, i)[2];
			green = img_left.at < Vec3b > (j, i)[1];
			blue = img_left.at < Vec3b > (j, i)[0];
			if ( Z < 0.5 || d < 3.)
			{
				Z = 100;
			}
			// fabs(X) < 0.5 || fabs(Y) < 0.5 || fabs(Z) < 0.5 ||
			*iter_x = X;
			*iter_y = Y;
			*iter_z = Z;
			*iter_r = red;
			*iter_g = green;
			*iter_b = blue;

			++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b;
		}
	}
	point_cloud_pub.publish(*pc);
	bag.write("/point_cloud", captured_time, *pc);	
}


/*
* input:两张矫正后的灰度图像
* output:生成成功返回视差图
*/
bool generateDisparityMap(Mat& left, Mat& right, Mat &disparity_map) {
	if (left.empty() || right.empty())
		return false;
	const Size imsize = left.size();
	const int32_t dims[3] = { imsize.width, imsize.height, imsize.width };
	Mat leftdpf = Mat::zeros(imsize, CV_32F);
	Mat rightdpf = Mat::zeros(imsize, CV_32F);

	Elas::parameters param;
	param.postprocess_only_left = true;
	param.disp_max = 80;
	//param.support_threshold = 0.7;
	//param.match_texture = 32;
	Elas elas(param);
  
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	vector<Eigen::Vector3d> tri = elas.process(left.data, right.data, leftdpf.ptr<float>(0), rightdpf.ptr<float>(0), dims);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double> >(t2 - t1);
    cout <<  "genereate disparity image take time : " << time_span.count() << " seconds" << endl;

    disparity_map = leftdpf;

	// imshow left, right, and disparity map
    // find maximum disparity for scaling output disparity images to [0..255]
    double disp_min, disp_max;
    cv::minMaxLoc(leftdpf, &disp_min, &disp_max);
    std::cout << "col = " << leftdpf.cols << " disp_max = " << disp_max << std::endl;

    Mat disparity_show;
    leftdpf.convertTo(disparity_show, CV_32FC1, 255.f/disp_max);
	imshow("disparity_show", disparity_show);
    imwrite("disparity_show.jpg", disparity_show);
	waitKey(200);
	// generator rectified image
    Mat rectified_image = cv::Mat(left.rows, 2*left.cols, left.type());
    left.copyTo(rectified_image(cv::Rect(0, 0, left.cols, left.rows)));
    right.copyTo(rectified_image(cv::Rect(left.cols, 0, left.cols, left.rows)));
	// std::cout << "left size = " << left.size() << std::endl;
	imwrite("rectified_image.jpg", rectified_image);
	// waitKey(-1);
	return true;
}

// read sfm json file 
// for each image generate depth map 
// fuse all depth map to a point cloud
int main(int argc, char **argv)
{   
    // for rectify
    Mat Q, P1, P2;
    Mat lmapx, lmapy, rmapx, rmapy;
    Rect validRoi[2];
    // camera intrinsic
    Mat K1, K2, D1, D2, R1, R2;
    // camera extrinsic
    Mat R_w_1, R_w_2;
    Mat t_w_1, t_w_2;
    // image size
    Size calib_img_size(4032,3016);
    Size out_img_size(4032,3016);
    // image
    Mat img_left, img_right, P1, P2, R1, R2;
    // from json read parameters

    K1 = (Mat1d(3, 3) << 3100.97303930001, 0, 1981.2491798477354, 0, 3100.97303930001, 1555.1063480419512, 0, 0, 1);
    K2 = K1;
    D1 = (Mat1d(1, 5) << 0.3045357354739094, -1.429408528670386, 0, 0, 1.9829614242559038);   
    D2 = D1;
    R_w_1 = (Mat1d(3, 3) <<                 
                    
                        0.9975499411260206,
                        0.06031512879455703,
                        0.03544291463703804
                    ,
                    
                        0.06697120243262017,
                        -0.969786153694405,
                        -0.23458404495476857
                    ,
                    
                        0.020223080976977223,
                        0.23638295484469236,
                        -0.9714494972229337
                                        
                    );
    R_w_2 = (Mat1d(3, 3) <<                 
                
                    
                        0.9985397058870259,
                        -0.03468421914315981,
                        0.04141812054502026
                    ,
                    
                        -0.022929953476304198,
                        -0.9663027133181588,
                        -0.2563850297258771
                    ,
                    
                        0.04891495681922952,
                        0.2550609165991404,
                        -0.9656869346858636
                        
                    );
    t_w_1 = (Mat1d(3, 1) << -103.96032545035273,
                    -15.54509673293458,
                    366.5595563504047);

    t_w_2 = (Mat1d(3, 1) << -45.961419242838129,
                    -24.315234334313215,
                    363.09550038603148);   

    Mat ones = (Mat1d(1, 4) << 0.0,
                    0.0,
                    0.0,
                    1.0);  
    // 
    Mat T_w1, T_w2; 
    hconcat(R_w_1, t_w_1, T_w1);
    vconcat(T_w1, ones, T_w1);
    hconcat(R_w_2, t_w_2, T_w2);
    vconcat(T_w2, ones, T_w2);
    cout << "T_w1 = " <<  T_w1  << endl;
    cout << "T_w2 = " <<  T_w2  << endl;

    Mat T_21 = T_w1.inv() * T_w2;
    cout << "T_21 = " <<  T_21  << endl;
    Mat R_21 = T_21.colRange(0,3).rowRange(0,3); 
    Mat t_21 = T_21.colRange(3,4).rowRange(0,3) * 0.001;
    cout << "R_21 = " <<  R_21 << " \n " << "t_21 = " << t_21 << endl;


    std::cout << " try to rectify" << std::endl;
	stereoRectify(K1, D1, K2, D2, calib_img_size, R_21, t_21, R1, R2, P1, P2, Q,
			CV_CALIB_ZERO_DISPARITY, 0, calib_img_size, &validRoi[0], &validRoi[1]);
    cv::initUndistortRectifyMap(K1, D1, R1, P1, out_img_size, CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, out_img_size, CV_32F, rmapx, rmapy);
    // recitify two image
    Mat l_img_rectified(out_img_size, img_left.type());
    Mat r_img_rectified(out_img_size, img_left.type());
    remap(img_left, l_img_rectified, lmapx, lmapy, cv::INTER_LINEAR);
    remap(img_right, r_img_rectified, rmapx, rmapy, cv::INTER_LINEAR);
  
    Mat disparity_map;

    Mat l_img_rectified = imread("/home/neousys/Project/volume_measurement/data/competition_test/colmap/dense/1.jpg-2.jpg/1.jpg");
    Mat r_img_rectified = imread("/home/neousys/Project/volume_measurement/data/competition_test/colmap/dense/1.jpg-2.jpg/2.jpg");
    Mat l_img_rectified_grey = imread("/home/neousys/Downloads/ELAS-master/img/aloe_left.pgm");
    Mat r_img_rectified_grey = imread("/home/neousys/Downloads/ELAS-master/img/aloe_right.pgm");
    //cv::cvtColor(l_img_rectified, l_img_rectified_grey, cv::COLOR_BGR2GRAY);
    //cv::cvtColor(r_img_rectified, r_img_rectified_grey, cv::COLOR_BGR2GRAY);
    rectify()
    generateDisparityMap(l_img_rectified_grey, r_img_rectified_grey, disparity_map);
    generatePointCloud(img_left_color, disparity_map);
    std::cout << "Hello World" << std::endl;
}