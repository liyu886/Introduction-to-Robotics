
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <iostream> 
#include <fstream>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <time.h>
#include <math.h>

using namespace std;
using namespace webots;



int main() {

	//ofstream fp;

	Robot *robot = new Robot();//使用webots的机器人主体
	
	int timeStep = (int)robot->getBasicTimeStep();//获取你在webots设置一帧的时间
	cout << timeStep << endl;

	Motor *motors[4];//电机和键盘都要用webots给的类型
	char wheels_names[4][8] = { "motor1","motor2","motor3","motor4" };//你在仿真器里面设置的名字

	Camera *camera; //声明照相机变量
	camera = robot->getCamera("mycamera");
	camera->enable(1);//开启相机功能
	
	int image_height = camera->getHeight();
	int image_width = camera->getWidth();


	GPS *gps = robot->getGPS("robot_gps");//声明GPS
	gps->enable(1);//开启GPS

	double speed1[4];

	//初始化所有状态
	for (int i = 0; i < 4; i++)
	{
		motors[i] = robot->getMotor(wheels_names[i]);//按照你在仿真器里面设置的名字获取句柄
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);

		speed1[i] = 0;
	}
	
	//设置速度
	double velocity = 10;
	
	//本次实验中只用到了直走、左旋、右旋三种action
	double speed_forward[4] = { velocity ,velocity ,velocity ,velocity };
	double speed_leftCircle[4] = { velocity ,-velocity ,-velocity ,velocity };
	double speed_rightCircle[4] = { -velocity ,velocity ,velocity ,-velocity };

	//其他变量
	double lf_wheel = 0, rf_wheel = 0, lb_wheel = 0, rb_wheel = 0;//计算四个轮子的平均速度
	double mistake = 0; //计算误差
	int left_x = 0, right_x = 0, middle_x = 0;  //image中首先出现黑色的左点和右点
	double cur_time = 0, init_time = 0; //记录当前时间和在起点处的时间
	int count = 0; //循环次数，为求平均值而设
	double init_pos[3] = {0.0};	//在起点处的位置
	double radius = 1.785;	//圆形轨道的半径
	

	while (robot->step(timeStep) != -1) //仿真运行一帧
	{
		
		
		
		
		
		cur_time = time(NULL);//获得当前时间
		const double* cur_pos = gps->getValues();//获取当前位置
		//在第一次循环中记录初始位置以及初始时间
		if(count == 0){
			for(int i = 0; i < 3; i++){
				init_pos[i] = cur_pos[i];
			}
			init_time = cur_time;
		}else if(fabs(cur_time - init_time) > 2 && fabs(init_pos[0] - cur_pos[0]) < 0.1 && fabs(init_pos[1] - cur_pos[1]) < 0.1 && fabs(init_pos[2] - cur_pos[2]) < 0.1){
			cout << "Run one cycle cost " << cur_time - init_time << "s" << endl;
			init_time = cur_time;

			//计算平均速度
			

			cout << "Average velocity is:" << endl;
			cout << "left-front wheel:" << lf_wheel / count << endl;
			cout << "right-front wheel:" << rf_wheel / count << endl;
			cout << "left-back wheel:" << lb_wheel / count << endl;
			cout << "right-back wheel:" << rb_wheel /count << endl;

			lf_wheel = 0;
			rf_wheel = 0;
			lb_wheel = 0;
			rb_wheel = 0;

			//求误差
			cout << "The mistake is " << mistake / count << endl;

			mistake = 0;

			count = 0;
		}
		
		//获得图像
		const unsigned char *image = camera->getImage();
		
		// fp.open("result.txt");
		// double gray = 0;

		// for (int x = 0; x < image_width; x++){
                     
			// for (int y = 0; y < image_height; y++) {
				// gray = camera->imageGetGray(image, image_width, x, y);
				// fp << "at" << x << "," << y << " gray=" << gray << endl;
				
        	// }
            // fp << endl;
    	// }
    	
		//从左向右扫描image_height/5行。
		for (int x = 0; x < image_width; x++){
			//若其返回的gray值<100则判定它为黑色
			if(camera->imageGetGray(image, image_width, x, 1* image_height/2) < 50){
				//记录当前点的横坐标值并终止循环
				left_x = x;
				break;
			}
		}
		//从右向左扫描image_height/4行
		for(int x = image_width-1; x >= 0; x--){
			//遇到第一个gray值<100的，记录并终止循环
			if(camera->imageGetGray(image, image_width, x, 2 * image_height/3) < 50){
				right_x = x;
				break;
			}
		}
		
		//计算刚刚左右两点的中点
		middle_x = (left_x + right_x) / 2;
		
		//如果黑色区域中点小于整个图像中点，则进行左旋
		//if(middle_x < image_width/2){
		if(middle_x < 30){
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_leftCircle[i];
				
			}
		//}else if(middle_x > image_width/2){//若大于则右旋
		}else if(middle_x > 34){//若大于则右旋
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_rightCircle[i];
			}
		}else{//否则直行
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_forward[i];
			}
		}
		
		//让电机执行
		for (int i = 0; i < 4; i++)
		{
			motors[i]->setVelocity(speed1[i]);
			if(i == 0){
				lf_wheel += speed1[i];
			}else if(i == 1){
				rf_wheel += speed1[i];
			}else if(i == 2){
				lb_wheel += speed1[i];
			}else{
				rb_wheel += speed1[i];
			}
		}

		//fp.close();
		mistake += fabs(sqrt(cur_pos[0] * cur_pos[0] + cur_pos[1] * cur_pos[1]) - radius);
		count++;
	}
	

	return 0;
}
