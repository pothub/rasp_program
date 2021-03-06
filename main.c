#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <stdio.h>
#include <dirent.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

CvCapture *capture;
char *wname = "camera";
IplImage *img_origin,*imgResult;
IplImage *img[3],*imgThreshold[3],*imgTmp;
int fd;	//mpu6050
int yaw=0,pitch=0;	//mpu6050
#define wi 320
#define hi 240
#define RANGE 100
#define CS 24
#define max_case 5
#define sensor 3800

void set_motor(){
	softPwmCreate(4,0,RANGE);
	softPwmCreate(17,0,RANGE);
	softPwmCreate(27,0,RANGE);
	softPwmCreate(22,0,RANGE);
}
void motor(int num, int pow){
	if(pow > 70) pow = 70;
	if(pow < -70) pow = -70;
	if(num == 0){
		if(pow>0){
			softPwmWrite(4,pow);
			softPwmWrite(17,0);
		}
		else{
			softPwmWrite(4,0);
			softPwmWrite(17,-pow);
		}
	}else{
		if(pow>0){
			softPwmWrite(27,pow);
			softPwmWrite(22,0);
		}
		else{
			softPwmWrite(27,0);
			softPwmWrite(22,-pow);
		}
	}
}
void led(int num, int onoff){
	if(num == 0){
		if(onoff == 0) digitalWrite(25,0);
		else digitalWrite(25,1);
	}
	else if(num == 1){
		if(onoff == 0) digitalWrite(8,0);
		else digitalWrite(8,1);
	}
	else{
		if(onoff == 0) digitalWrite(7,0);
		else digitalWrite(7,1);
	}
}
void set_gpio(){
	pinMode(23,OUTPUT);	//bz
	pinMode(15,OUTPUT);	//air
	pinMode(25,OUTPUT);
	pinMode(8,OUTPUT);
	pinMode(7,OUTPUT);
	digitalWrite(25,0);
	digitalWrite(8,0);
	digitalWrite(7,0);
}
void bz(int onoff){
	if(onoff) digitalWrite(23,1);
	else digitalWrite(23,0);
}
void air(int onoff){
	if(onoff) digitalWrite(15,1);
	else digitalWrite(15,0);
}
void set_spi(){
	wiringPiSPISetup(0,100000);
	pinMode(CS,OUTPUT);
}
int read_adc(unsigned char adcChannel) {
	unsigned char buff[3];
	int adcValue = 0;
	buff[0] = 0x06 + (adcChannel >> 2);
	buff[1] = adcChannel << 6;
	buff[2] = 0x00;
	digitalWrite(CS, 0); // Low : CS Active
	wiringPiSPIDataRW(0, buff, 3);
	buff[1] = 0x0F & buff[1];
	adcValue = ( buff[1] << 8) | buff[2];
	digitalWrite(CS, 1); // High : CS Inactive
	return adcValue;
}
void set_mpu(){
	if((fd=wiringPiI2CSetup(0x68))<0){
		printf("can't find i2c\n");
		return;
	}
	wiringPiI2CWriteReg8(fd,0x6B,00);
}
void update_mpu(){
	short int ax,ay,az,gx,gy,gz;
	static int n=0,p=0;
	ax = wiringPiI2CReadReg8(fd,0x3B)<<8|wiringPiI2CReadReg8(fd,0x3C);
	delay(1);
	ay = wiringPiI2CReadReg8(fd,0x3D)<<8|wiringPiI2CReadReg8(fd,0x3E);
	delay(1);
	az = wiringPiI2CReadReg8(fd,0x3F)<<8|wiringPiI2CReadReg8(fd,0x40);
	delay(1);
	gx = wiringPiI2CReadReg8(fd,0x43)<<8|wiringPiI2CReadReg8(fd,0x44);
	delay(1);
	gy = wiringPiI2CReadReg8(fd,0x45)<<8|wiringPiI2CReadReg8(fd,0x46);
	delay(1);
	gz = wiringPiI2CReadReg8(fd,0x47)<<8|wiringPiI2CReadReg8(fd,0x48);
	delay(1);
	n=0.9*(p+gy/100)+0.1*ay;
	pitch=(0.9*n+0.1*p)/10;
	p=n;
	yaw+=gz/1000;
}
void set_servo(){
	pinMode(18,PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(400);
	pwmSetRange(1024);
	pwmWrite(18,80);
}
void setcam(){
	capture = cvCreateCameraCapture(-1);
	if (capture==NULL) {
		puts("*ERR* cvCreateCameraCapture");
		return;
	}
	cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_WIDTH, wi);
	cvSetCaptureProperty (capture, CV_CAP_PROP_FRAME_HEIGHT, hi);

	int i;
	for(i=0;i<3;i++){
		img[i] 			= cvCreateImage(cvSize(wi,hi),IPL_DEPTH_8U,1);
		imgThreshold[i] 	= cvCreateImage(cvSize(wi,hi),IPL_DEPTH_8U,1);
	}
	imgTmp = cvCreateImage(cvSize(wi,hi),IPL_DEPTH_8U,1);
	imgResult = cvCreateImage(cvSize(wi,hi),IPL_DEPTH_8U,1);
}
int takePicture(const char *path){
	// img_origin = cvQueryFrame(capture);
	if( !img_origin ){
		puts("*ERR* cvQueryFrame");
		return -2;
	}
	if( !cvSaveImage(path, imgResult, NULL) ){
		puts("*ERR* cvSaveImage");
		return -3;
	}
	return 0;
}
int gx=0;
CvMoments moments;
void excolor(){
	if((img_origin = cvQueryFrame(capture)) == NULL) return;
	cvSplit(img_origin,img[2],img[1],img[0],NULL);//BGR

	cvThreshold(img[2],imgThreshold[2],100,255,CV_THRESH_BINARY);
	cvDiv(img[2],img[1],imgTmp,6);
	cvThreshold(imgTmp,imgThreshold[1],6,255,CV_THRESH_BINARY);
	cvDiv(img[2],img[0],imgTmp,6);
	cvThreshold(imgTmp,imgThreshold[0],6,255,CV_THRESH_BINARY);
	cvAnd(imgThreshold[1],imgThreshold[0],imgTmp,NULL);
	cvAnd(imgTmp,imgThreshold[2],imgResult,NULL);

	cvMoments(imgResult,&moments,0);
	double m00 = cvGetSpatialMoment(&moments,0,0);
	double m10 = cvGetSpatialMoment(&moments,1,0);
	gx = m10/m00;
	printf("%f\t%f\t%d\n",m00,m10,gx);
}
int black_x=0;
void exblack(){
	IplImage* gray;
	if((img_origin = cvQueryFrame(capture)) == NULL) return;
	gray = cvCreateImage(cvGetSize(img_origin),IPL_DEPTH_8U,1);
	cvCvtColor(img_origin,gray,CV_BGR2GRAY);
	cvThreshold(gray,imgResult,45,255,CV_THRESH_BINARY);

	int x=160,y,tmp[wi]={0},maxx=10000000;
	for(x=0;x<imgResult->width;x++){
		for(y=0;y<imgResult->height;y++){
			tmp[x] += imgResult->imageData[y * imgResult->widthStep + x * imgResult->nChannels];
		}
		if(tmp[x]<maxx) {
			maxx = tmp[x];
			black_x = x;
		}
	}
	// printf("%d\t%d\n",maxx,black_x);
}
void bz_num(int num){
	int i=0;
	motor(0,0);
	motor(1,0);
	for(;i<num;i++){
		bz(1);
		delay(100);
		bz(0);
		delay(100);
	}
	delay(100);
}
void step1(){
	while(read_adc(3)<sensor){
		if(read_adc(4)<sensor){
			printf("4\n");
			motor(0,30);
			motor(1,20);
		}
		else if(read_adc(7)<sensor){
			printf("7\n");
			motor(0,20);
			motor(1,30);
		}
		else{
			printf("else\n");
			motor(0,20);
			motor(1,20);
		}
	}
}
void step2(){
	yaw=0;
	int turn=900;
	motor(0,-20);
	motor(1,-20);
	delay(500);
	motor(0,0);
	motor(1,0);
	delay(500);
	while(yaw<turn){
		motor(0,-20);
		motor(1,20);
		update_mpu();
	}
	motor(0,0);
	motor(1,0);
	delay(500);
	int i=0;
	while(i<200){
		int m=(yaw-turn)/5;
		update_mpu();
		i++;
		printf("i\t%d\n",i);
		if(m>20) m=20;
		if(m<-20) m=-20;
		motor(0,20+m);
		motor(1,20-m);
	}
	// motor(0,23);
	// motor(1,20);
	// delay(2200);
	motor(0,0);
	motor(1,0);
	delay(500);
	while(yaw>0){
		motor(0,20);
		motor(1,-20);
		update_mpu();
	}
	motor(0,0);
	motor(1,0);
	delay(500);
	i=0;
	while(i<400){
		int m=yaw/5;
		update_mpu();
		i++;
		printf("i\t%d\n",i);
		if(m>20) m=20;
		if(m<-20) m=-20;
		motor(0,20+m);
		motor(1,20-m);
	}
	// motor(0,23);
	// motor(1,20);
	// delay(3200);
	motor(0,0);
	motor(1,0);
	delay(500);
	while(yaw>-turn+200){
		motor(0,20);
		motor(1,-20);
		update_mpu();
	}
	motor(0,0);
	motor(1,0);
	delay(500);
	motor(0,23);
	motor(1,20);
	while((read_adc(4)>sensor)&&(read_adc(5)>sensor)&&(read_adc(6)>sensor)&&(read_adc(7)>sensor));
	motor(0,0);
	motor(1,0);
}
void step3(int pow){
	int turn=0;
	while(turn != 1){
		if(read_adc(4)<sensor){
			motor(0,pow+10);
			motor(1,pow);
			turn=4;
		}
		else if(read_adc(7)<sensor){
			motor(0,pow);
			motor(1,pow+10);
			turn=7;
		}
		else{
			if((read_adc(5)<sensor)||read_adc(6)<sensor){
				motor(0,pow);
				motor(1,pow);
				turn = 56;
			}
			else if(turn == 4){
				motor(0,pow+20);
				motor(1,pow-10);
			}
			else if(turn == 7){
				motor(0,pow-10);
				motor(1,pow+20);
			}
			else if(turn == 56){
				turn =1;
			}
		}
	}
}
void step4(){
	motor(0,20);
	motor(1,20);
	delay(700);
	motor(0,0);
	motor(1,0);

	if((read_adc(4)<sensor||read_adc(5)<sensor||read_adc(6)<sensor||read_adc(7)<sensor)){
		motor(0,20);
		motor(1,-20);
		delay(800);
		step3(20);
	}
}
void step5(int pow){
	yaw = 0;
	while(read_adc(3)<sensor){
		int m=yaw/5;
		update_mpu();
		printf("%d\t%d\t%d\n",yaw,20-m,20+m);
		if(m>20) m=20;
		if(m<-20) m=-20;
		motor(0,20+m);
		motor(1,20-m);
	}
	motor(0,-20);
	motor(1,-20);
	delay(500);
	motor(0,0);
	motor(1,0);
	delay(500);
	yaw = 0;
	while(yaw>-400){
		motor(0,20);
		motor(1,-20);
		update_mpu();
		delay(10);
	}
	// motor(0,20);
	// motor(1,-20);
	// delay(1000);
	while((read_adc(4)>sensor)&&(read_adc(5)>sensor)&&(read_adc(6)>sensor)&&(read_adc(7)>sensor));
	int turn=0;
	int cnt=0;
	while(cnt<300 || pitch > 100){
		update_mpu();
		if(read_adc(4)<sensor){
			motor(0,pow+10);
			motor(1,pow);
			turn=4;
		}
		else if(read_adc(7)<sensor){
			motor(0,pow);
			motor(1,pow+10);
			turn=7;
		}
		else{
			if((read_adc(5)<sensor)||read_adc(6)<sensor){
				motor(0,pow);
				motor(1,pow);
				turn = 56;
			}
			else if(turn == 4){
				motor(0,pow+20);
				motor(1,pow-10);
			}
			else if(turn == 7){
				motor(0,pow-10);
				motor(1,pow+20);
			}
			else if(turn == 56){
				turn =1;
			}
		}
		printf("%d\n",cnt++);
		if(cnt > 250 && cnt < 300) yaw=0;
	}
	motor(0,20);
	motor(1,20);
	delay(1300);
	while(yaw<400){
		motor(0,-20);
		motor(1,20);
		update_mpu();
		printf("step2\t1\t%d\n",yaw);
		delay(10);
	}
	motor(0,0);
	motor(1,0);
	delay(500);
	motor(0,-20);
	motor(1,-20);
	delay(500);
}
void step6(){
	yaw=0;
	motor(0,20);
	motor(1,20);
	delay(500);
	char path[256];
	while(read_adc(3)<sensor){
		excolor();
		update_mpu();
		int m=(170-gx)/10;
		if(m>20) m=20;
		if(m<-20) m=-20;
		motor(0,10-m);
		motor(1,10+m);
		printf("m\t%d\n",m);
		// int i=0;
		// for(;i<10;i++){
		// 	sprintf(path, "photo/cam%05d.jpg", i);
		// 	excolor();
		// 	takePicture(path);
		// 	printf("gx\t%d\n",gx);
		// }
		// delay(2000);
	}
	motor(0,0);
	motor(1,0);
	pwmWrite(18,55);
	delay(1000);
	air(1);
	delay(1000);
	pwmWrite(18,80);

	int i=0;
	bz_num(1);
	while(i<100){
		i++;
		int m=yaw/2;
		m+=yaw/5;
		update_mpu();
		printf("%d\n",m);
		if(m>20) m=20;
		if(m<-20) m=-20;
		motor(0,m);
		motor(1,-m);
		delay(10);
	}
	motor(0,20);
	motor(1,-20);
	delay(200);
	bz_num(1);
	i=0;
	while(i<100){
		i++;
		exblack();
		int black_motor=(150-black_x)/8;
		printf("%d\t%d\n",black_motor,i);
		if(black_motor>20) black_motor=20;
		if(black_motor<-20) black_motor=-20;
		motor(0,-black_motor);
		motor(1,black_motor);
	}
	motor(0,0);
	motor(1,0);
	delay(100);
	yaw = 0;
	i=0;
	while(i<200){
		i++;
		int m=yaw/5;
		update_mpu();
		if(m>20) m=20;
		if(m<-20) m=-20;
		motor(0,20+m);
		motor(1,20-m);
	}
	bz_num(1);
	i=0;
	while(i<100){
		i++;
		exblack();
		int black_motor=(150-black_x)/8;
		printf("%d\t%d\n",black_motor,i);
		if(black_motor>20) black_motor=20;
		if(black_motor<-20) black_motor=-20;
		motor(0,-black_motor);
		motor(1,black_motor);
	}
	motor(0,0);
	motor(1,0);
	delay(100);
	yaw = 0;
	// i=0;
	// while(i<200){
	// 	i++;
	// 	int m=yaw/5;
	// 	update_mpu();
	// 	if(m>20) m=20;
	// 	if(m<-20) m=-20;
	// 	motor(0,20+m);
	// 	motor(1,20-m);
	// }
	// bz_num(1);
	// i=0;
	// while(i<100){
	// 	i++;
	// 	exblack();
	// 	int black_motor=(150-black_x)/8;
	// 	printf("%d\t%d\n",black_motor,i);
	// 	if(black_motor>20) black_motor=20;
	// 	if(black_motor<-20) black_motor=-20;
	// 	motor(0,-black_motor);
	// 	motor(1,black_motor);
	// }
	// motor(0,0);
	// motor(1,0);
	// delay(100);
	while((read_adc(4)<sensor)&&(read_adc(5)<sensor)&&(read_adc(6)<sensor)&&(read_adc(7)<sensor)){
		int m=yaw/5;
		update_mpu();
		if(m>20) m=20;
		if(m<-20) m=-20;
		motor(0,20+m);
		motor(1,20-m);
	}
	motor(0,0);
	motor(1,0);
	pwmWrite(18,55);
	delay(1000);
	air(0);
	delay(1000);
	pwmWrite(18,80);
	// char path[256];
	// while(1){
	// 	int i=0;
	// 	for(;i<10;i++){
	// 		sprintf(path, "photo/cam%05d.jpg", i);
	// 		exblack();
	// 		takePicture(path);
	// 		printf("gx\t%d\n",gx);
	// 	}
	// 	delay(2000);
	// }
}
int main(){
	wiringPiSetupGpio();
	set_motor();
	set_mpu();
	set_spi();
	set_servo();
	set_gpio();
	air(0);
	setcam();
	bz_num(1);

	// step7();
	// bz_num(1);
	int case_num = 1;
	while(read_adc(2)<sensor){
		if(read_adc(0)>sensor){
			case_num++;
			if(case_num>max_case) case_num=1;
			delay(100);
			bz_num(case_num);
		}
		printf("case_num\t%d\n",case_num);
	}

	switch(case_num){
		case 1:
			bz_num(1);
			step1();
		case 2:
			bz_num(2);
			step2();
		case 3:
			bz_num(3);
			step3(20);
			bz_num(1);
			step4();
			bz_num(1);
		case 4:
			// step3(20);
			bz_num(4);
			step5(30);
		case 5:
			step6();
			bz_num(5);
	}
	while(1);
	while(1){
		int i[8]={0},num=0;
		for(num=0;num<8;num++){
			i[num]=read_adc(num);
			printf("%d\t",i[num]);
			delay(1);
		}
		printf("\n");
		delay(100);
	}
	while(1){
		update_mpu();
		printf("%d\t%d\n",yaw,pitch);
		delay(10);
	}

	// char path[256];
	// setcam();
	// for(;i<10;i++){
	// 	sprintf(path, "photo/cam%05d.jpg", i);
	// 	excolor();
	// 	takePicture(path);
	// }
	cvReleaseCapture(&capture);

	return 0;
}
