

#include "com.h"
#include <conio.h>
#include <opencv/cv.h> 
#include <math.h>
#include <opencv/highgui.h> 
#include "GraphUtils.h"
#include<time.h>


#define ELEMENTNUM 1000 
#define WIDTH 1000 
#define HEIGHT 400 
#define GYRO_TYP 65.5 //gyroscope 精度
#define ACCE_RANGE 2
#define GYRO_RANGE 180


float floatVec[7][ELEMENTNUM];
float floatVec1[ELEMENTNUM];
float floatVec2[ELEMENTNUM];
float f[100000];
double allData[7];

float Gyro_y;        //Y轴陀螺仪数据暂存 
float Angle_gy;      //由角速度计算的倾斜角度 
float Accel_x;      //X轴加速度值暂存 
float Angle_az;      //由加速度计算的倾斜角度 
float Angle;         //小车最终倾斜角度 
char value;

float  Q_angle = 0.001;
float  Q_gyro = 0.003;
float  R_angle = 0.5;
float  dt = 0.014;                   //dt为kalman滤波器采样时间; 
char   C_0 = 1;
float  Q_bias=0, Angle_err;
float  PCt_0, PCt_1, E;
float  K_0, K_1, t_0, t_1;
float  Pdot[4] = { 0, 0, 0, 0 };
float  PP[2][2] = { { 1, 0 }, { 0, 1 } };

void Kalman_Filter(float Accel, float Gyro);
void myKalman(float ax, float ay, float az, float gy);



int main(){
	f[0] = 0;
	PortOperate com;
	char *infor[6];
	infor[0] = "ax:";
	infor[1] = "ay:";
	infor[2] = "az:";
	infor[3] = "t:";
	infor[4] = "cx:";
	infor[5] = "cy:";
	infor[6] = "cz:";
	//printf("short:%d  int:%d", sizeof(short), sizeof(int));
	//std::cin >> start;
	com.OpenComm("com5");
	double init[7] = { 0.040374, 0.005359, 0.156069, 33.178735, -186.309, 3964.928000,15.0 };
	/*gyroOffset[0]:-2.733115 gyroOffset[1] : 60.763847 gyroOffset[2] : 0.229008   gyroOffset[0]:65350.222000      gyroOffset[1]:3964.928000       gyroOffset[2]:1
.000000*/

	clock_t start,end,end1,end2,start1;
	int element = 0;
	IplImage *graphImage = NULL;
	cvNamedWindow("CGraphy", CV_WINDOW_AUTOSIZE);
	while(1){
		start = clock();
		unsigned char aBuffer[3],bBuffer[15],check;
		double max = -999, min = 999;
		int temp[7] = { 0, 0, 0, 0, 0, 0, 0 };
		com.ReadData(&aBuffer[0], 2);
		do{
			com.ReadData(&aBuffer[2], 1);
			if (aBuffer[0] == 0xAA && aBuffer[1] == 0xAB && aBuffer[2] == 0xAC){
				com.ReadData(bBuffer, 15);
				check = 0;
				for (int i = 0; i < 14; i++){
					check |= bBuffer[i];
				}
				if (check==bBuffer[14])
					break;
			}
			aBuffer[0] = aBuffer[1];
			aBuffer[1] = aBuffer[2];
		} while (1);

		//std::cout << *temp << std::endl;

		for (int j = 0; j < 7; j++){
			temp[j] = ((int)bBuffer[j * 2] << 8) | (int)(bBuffer[j * 2 + 1]);
			if (temp[j]>=32764&&j<3)
			{
				temp[j] = (16384 - (temp[j] - 49152));
			}
			if (temp[j]>=32768&&j>3)
			{
				temp[j] = temp[j] - 65535;
			}
			
		}
		//printf("ax:%f	ay:%f	az:%f\nt:%f\ncx:%f	cy:%f	cz:%f\nmax:%f	min:%f\n", temp[0] / 16384.0 - init[0], temp[1] / 16384.0 - init[1], temp[2] / 16384.0 - init[2], temp[3] / 340.0 + 36.53, temp[4] / GYRO_TYP - init[4], temp[5] / GYRO_TYP - init[5], temp[6] / GYRO_TYP - init[6], max, min);
		//printf("gx:%d	gy:%d	gz:%d\n", temp[4], (temp[5]), (temp[6]));
	    
		allData[0] = temp[0] / 16384.0;
		allData[1] = temp[1] / 16384.0;
		allData[2] = temp[2] / 16384.0;
		allData[3] = temp[3] / 340.0 + 36.53;
		allData[4] = (temp[4] - init[4] )/ GYRO_TYP;
		allData[5] = (temp[5] - init[5]) / GYRO_TYP;
		allData[6] = (temp[6] - init[6]) / GYRO_TYP;
		//printf("gx:%f	gy:%f	gz:%f\n", allData[4], (allData[5]), (allData[6]));
		if (element <= 999){
			floatVec[0][element] = allData[0];
			floatVec[1][element] = allData[1];
			floatVec[2][element] = allData[2];
			floatVec[3][element] = allData[4];
			floatVec[4][element] = allData[5];
			floatVec[5][element] = allData[6];
			/*floatVec[3][element] = temp[4];// allData[4];
			floatVec[4][element] = temp[5];//allData[5];
			floatVec[5][element] = temp[6];// allData[6];*/
			
			
			myKalman(allData[0], allData[1], allData[2], allData[4]);
			floatVec[6][element] = Gyro_y;
			floatVec[5][element] = Angle_az;
			graphImage = drawFloatGraph(CV_RGB(0, 0, 0), floatVec[6], element, graphImage, -180, 180, WIDTH, HEIGHT);		// dark

			graphImage = drawFloatGraph(CV_RGB(60, 60, 255), floatVec[3], element, graphImage, -GYRO_RANGE, GYRO_RANGE, WIDTH, HEIGHT);     //light blue
			//graphImage = drawFloatGraph(CV_RGB(60, 255, 60), floatVec[4], element, graphImage, -GYRO_RANGE, GYRO_RANGE, WIDTH, HEIGHT);   //light green
			graphImage = drawFloatGraph(CV_RGB(255, 60, 40), floatVec[5], element, graphImage, -GYRO_RANGE, GYRO_RANGE, WIDTH, HEIGHT);    //light red
			//graphImage = drawFloatGraph(CV_RGB(60, 60, 255), floatVec[0], element, graphImage, -ACCE_RANGE, ACCE_RANGE, WIDTH, HEIGHT);     //light blue
			//graphImage = drawFloatGraph(CV_RGB(60, 255, 60), floatVec[1], element, graphImage, -ACCE_RANGE, ACCE_RANGE, WIDTH, HEIGHT);   //light green
			//graphImage = drawFloatGraph(CV_RGB(255, 60, 40), floatVec[2], element, graphImage, -ACCE_RANGE, ACCE_RANGE, WIDTH, HEIGHT);    //light red
			element++;
			end1 = clock();
		}else{
			for (int k = 0; k < 7; k++){
				for (int kk = 0; kk <999; kk++){
					floatVec[k][kk] = floatVec[k][kk+1];
				}
			}
			floatVec[0][999] = allData[0];
			floatVec[1][999] = allData[1];
			floatVec[2][999] = allData[2];
			floatVec[3][999] = allData[4];
			floatVec[4][999] = allData[5];
			floatVec[5][999] = allData[6];
			/*floatVec[3][999] = temp[4];// allData[4];
			floatVec[4][999] = temp[5];//allData[5];
			floatVec[5][999] = temp[6];// allData[6];*/

			
			myKalman(allData[0], allData[1], allData[2], allData[4]);
			floatVec[6][999] = Gyro_y;
			floatVec[5][999] = Angle_az;
			graphImage = drawFloatGraph(CV_RGB(0, 0, 0), floatVec[6], element, graphImage, -180, 180, WIDTH, HEIGHT);		// dark

			graphImage = drawFloatGraph(CV_RGB(60, 60, 255), floatVec[3], 1000, graphImage, -GYRO_RANGE, GYRO_RANGE, WIDTH, HEIGHT);     //light blue
			//graphImage = drawFloatGraph(CV_RGB(60, 255, 60), floatVec[4], 1000, graphImage, -GYRO_RANGE, GYRO_RANGE, WIDTH, HEIGHT);   //light green
			graphImage = drawFloatGraph(CV_RGB(255, 60, 40), floatVec[5], 1000, graphImage, -GYRO_RANGE, GYRO_RANGE, WIDTH, HEIGHT);    //light red
			end2 = clock();
		}
		// 绘曲线图 graphImage = drawFloatGraph(floatVec1, ELEMENTNUM, NULL, -20, 20, WIDTH, HEIGHT, "NormRandomData"); 

		
		
		//if (element < 100000){ f[element + 1] = f[element] + 0.1; }
		//graphImage = drawFloatGraph(CV_RGB(255, 60, 40), f, element, graphImage, -20, 200, WIDTH, HEIGHT);    //light red
		start1 = clock();
		cvShowImage("CGraphy",graphImage);
		cvWaitKey(2);
		//std::cin >> start;
		cvReleaseImage(&graphImage);
		end = clock();
		//printf("time1:%f time2:%f\n", (double)(end1 - start + end - start1) / CLOCKS_PER_SEC, (double)(end2 - start + end - start1) / CLOCKS_PER_SEC);
		/*for (int i = 0; i < element; i++){
			if (floatVec[4][i]>max)max = floatVec[4][i];
			if (floatVec[4][i] < min)min = floatVec[4][i];
		}
		if ((max-min)<30&&element>=1000){
			double gyroOffset[3] = {0,0,0};
			for (int i = 0; i < 3; i++){
				for (int ii = 0; ii < 1000; ii++){
					gyroOffset[i]+=floatVec[i+3][ii];
				}
				gyroOffset[i] /= 1000.0;
				printf("gyroOffset[%d]:%f	", i, gyroOffset[i]);
			}
			break;
		}*/  //find the gyroscope offset. 
		
	}
	return 0;
}

void Kalman_Filter(float Accel, float Gyro){
	Angle += (Gyro - Q_bias) * dt; //先验估计 
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分 
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;

	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分 
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差 
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - Angle; //zk-先验估计 
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;  
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;   //后验估计误差协方差 
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	Angle += K_0 * Angle_err;  //后验估计 
	Q_bias += K_1 * Angle_err;  //后验估计 
	Gyro_y = Gyro - Q_bias;  //输出值(后验估计)的微分=角速度 
}

void myKalman(float ax, float ay, float az,float gx){
	Gyro_y = gx;
	Angle_az = atan(sqrt(ay*ay + ax*ax) / az) * 180 / 3.14159;
	Kalman_Filter(Angle_az, Gyro_y);
}
