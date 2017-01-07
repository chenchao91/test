#include<iostream>
#include<opencv2/opencv.hpp>
#include <Cmath>
#include<thread>
#include<time.h>
#include <mutex> 
using namespace cv;
using namespace std;
const int SCALE = 1;
const float Pi = 3.14;
struct PID {
	float p;
	float i;
	float d;
	float last_error;
};
struct carparam {
	float x;
	float y;
	float angle;//小车的起始位置和角度
	float height_b;
	float width_b;//底盘摄像头所采集图片的高度和宽度
	float high_f;
	float angle_f;
	float height_f;
	float width_f;//前置摄像头所采集图片的高度和宽度
	float frame;//帧率
};
struct carstate {
	float x;
	float y;
	float angle;
	float v;
	float w;

};
//=======================================
carparam param;
carstate car;
Mat front_img;
Mat button_img;
std::mutex mtx_front;
std::mutex mtx_button;
std::mutex mtx_car;
PID pid;
Mat show;
//=======================================
float myPID(float error1,float error2)
{
	float derror = error1 - pid.last_error;
	pid.last_error = error1;
	float result = pid.p*error1 +
		pid.d*derror;
	return result;
}
void carstate_upstate(Mat &map,Mat &show)
{
	double dt;
	clock_t start=0, finish=0;
	while(1)
	{

		dt=(finish-start)/(float)CLOCKS_PER_SEC;
		//cout <<"dt  time:"<< dt << "    "<<CLOCKS_PER_SEC << endl;
		start = finish;
		//cout << "=======================dt start" << start << endl;
		{
			//std::unique_lock<std::mutex> lck(mtx_car);
			car.x += car.v*sin(car.angle)*dt;
			car.y -= car.v*cos(car.angle)*dt;
			car.angle += car.w*dt;
			waitKey(300);
			//cout <<"catstate"<< car.x << " " << car.y <<" "<<car.angle<<"  "<<car.v<< "  "<<car.w<<endl;
			show.at<Vec3b>(car.y, car.x)[0] = 0;
			show.at<Vec3b>(car.y, car.x)[1] = 0;
			show.at<Vec3b>(car.y, car.x)[2] = 255;
		}
		finish = clock();
		//cout << "dt finish" << finish << endl;
	}
}
float cal_error(Mat src)
{
	for (int j = 0; j<src.cols; j++)
		if (src.at<uchar>(src.rows / 2, j) == 0)
		{
			return j - src.cols / 2;
		}
	return 0;
}
void camera_front(Mat &map,Mat &show,carparam &param)
{
	float car_x, car_y, car_angle;
	Mat tmp(param.height_f, param.width_f, CV_8UC1);
	while (1)
	{  
		{
			//std::unique_lock<std::mutex> lck(mtx_car);
			car_x = car.x;
			car_y = car.y;
			car_angle = car.angle;
		}
		//cout << "front::"<<clock()<<"   "<< car.x << "            " << car.y << "    " << car.angle << "  " << car.v << endl;
		show.at<Vec3b>(car.y+4, car.x)[0] = 0;
		show.at<Vec3b>(car.y+4, car.x)[1] = 255;
		show.at<Vec3b>(car.y+4, car.x)[2] = 0;
		for (int i = 0; i < tmp.rows; i++)
			for (int j = 0; j < tmp.cols; j++)
			{
				int im_y = car_y - tmp.rows / 2 - param.high_f *tan(param.angle_f) + i;
				int im_x = car_x - tmp.cols / 2 + j;
				int x = (im_x - car_x)*cos(car_angle) - (im_y - car_y)*sin(car_angle) + car_x;
				int y = (im_x - car_x)*sin(car_angle) + (im_y - car_y)*cos(car_angle) + car_y;
				if (y >= 0 && x >= 0 && y < map.rows&&x < map.cols)
					tmp.at<uchar>(i, j) = map.at<uchar>(y, x);
			}
		{
			//std::unique_lock<std::mutex> lck(mtx_front);
			//imshow("tmp", tmp);
			front_img = tmp.clone();
		}
		waitKey(30);
	}
}
void camera_button(Mat &map,Mat &show, carparam &param)
{
	float car_x, car_y, car_angle;
	Mat tmp(param.height_b, param.width_b, CV_8UC1);
	while (1)
	{
		{
			//std::unique_lock<std::mutex> lck(mtx_car);
			car_x = car.x;
			car_y = car.y;
			car_angle = car.angle;

		}
		show.at<Vec3b>(car.y+2, car.x)[0] = 255;
		show.at<Vec3b>(car.y+2, car.x)[1] = 0;
		show.at<Vec3b>(car.y+2, car.x)[2] = 0;
		cout <<"button   " << clock() << " " << car.x << " " << car.y << " " << car.angle << "  " << car.v <<"  "<<car.w<< endl;
		clock_t start, finish;
		//start = clock();
		for (int i = 0; i < tmp.rows; i++)
			for (int j = 0; j < tmp.cols; j++)
			{
				int im_y = car_y - tmp.rows / 2 + i;
				int im_x = car_x - tmp.cols / 2 + j;
				int x = (im_x - car_x)*cos(car_angle) - (im_y - car_y)*sin(car_angle) + car_x;
				int y = (im_x - car_x)*sin(car_angle) + (im_y - car_y)*cos(car_angle) + car_y;
				if (y >= 0 && x >= 0 && y < map.rows&&x < map.cols)
					tmp.at<uchar>(i, j) = map.at<uchar>(y, x);
			}
		{
			//std::unique_lock<std::mutex> lck(mtx_front);
			button_img = tmp.clone();
		}
		//finish = clock();
		//cout << start << "          " << finish << endl;
		waitKey(30);
	}

}

void simulation(Mat &map,Mat &show ,carparam &param, float t, PID pid)//t表示时延
{
	//carstate car;
	car.x = param.x;
	car.y = param.y;
	car.v = 2;
	car.w = 0;
	car.angle = param.angle;	
	thread front(camera_front,map,show,param);
	front.detach();
	thread button(camera_button, map, show,param);
	button.detach();
	thread (carstate_upstate, map, show).detach();
	while (1)
	{
		imshow("front", front_img);
		imshow("button", button_img);
		imwrite("button.png", button_img);
		float error1 = cal_error(button_img);
		float error2 = cal_error(front_img);
		float w_calculate = myPID(error1, error2);
		cout << "error1:" << error1 << endl;
		cout <<"w_calculate: "<< w_calculate << endl;
		car.w = w_calculate;
		imshow("show", show);
		waitKey(30);
	}

}

int main()
{
	Mat map;
	Mat show = imread("map.png");
	cvtColor(show, map, CV_BGR2GRAY);
	//carparam param;
	{
		param.x = 76;//像素为单位
		param.y = 60;
		param.angle = Pi / 2;//车头朝左为正90度，朝上为0度。
		param.height_b = 60;
		param.width_b = 80; 
		param.angle_f = Pi / 4;
		param.high_f = 90;//前置摄像头的角度和高度，这里仿真时以像素为单位
		param.height_f = 60;
		param.width_f = 80;
		param.frame = 30;
	}
	{
		front_img.create(param.height_f, param.width_f, CV_8UC1);
		button_img.create(param.height_b, param.width_b, CV_8UC1);
	}
	{
		pid.p = 0.1;
		pid.i = 0.3;
		pid.d = 0; 
		pid.last_error = 0;
	}
	imshow("src", map); 
	imshow("show", show);
	simulation(map, show,param, 0, pid);
	waitKey();
}