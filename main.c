#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#define RANGE 100
void set_motor(){
	softPwmCreate(4,0,RANGE);
	softPwmCreate(17,0,RANGE);
	softPwmCreate(27,0,RANGE);
	softPwmCreate(22,0,RANGE);
}
void motor(int num, int pow){
	if(pow > 100) pow = 100;
	if(pow < -100) pow = -100;
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
void set_led(){
	pinMode(23,OUTPUT);	//bz
	pinMode(25,OUTPUT);
	pinMode(8,OUTPUT);
	pinMode(7,OUTPUT);
	led(0,0);
	led(1,0);
	led(2,0);
}
void bz(int onoff){
	if(onoff) digitalWrite(23,1);
	else digitalWrite(23,0);
}
int fd;
short int ax,ay,az,gx,gy,gz;
void set_mpu(){
	pinMode(8,OUTPUT);
	pinMode(9,OUTPUT);
	if((fd=wiringPiI2CSetup(0x68))<0){
		printf("can't find i2c\n");
		return;
	}
	wiringPiI2CWriteReg8(fd,0x6B,00);
}
void update_mpu(){
	ax = wiringPiI2CReadReg8(fd,0x3B)<<8|wiringPiI2CReadReg8(fd,0x3C);
	ay = wiringPiI2CReadReg8(fd,0x3D)<<8|wiringPiI2CReadReg8(fd,0x3E);
	az = wiringPiI2CReadReg8(fd,0x3F)<<8|wiringPiI2CReadReg8(fd,0x40);
	gx = wiringPiI2CReadReg8(fd,0x43)<<8|wiringPiI2CReadReg8(fd,0x44);
	gy = wiringPiI2CReadReg8(fd,0x45)<<8|wiringPiI2CReadReg8(fd,0x46);
	gz = wiringPiI2CReadReg8(fd,0x47)<<8|wiringPiI2CReadReg8(fd,0x48);
}
int main(){
	if(wiringPiSetupGpio() == -1) return -1;
	set_motor();
	set_led();
	set_mpu();
	bz(1);
	delay(100);
	bz(0);

	while(1){
		update_mpu();
		printf("%d\t%d\t%d\t%d\t%d\t%d\n",ax,ay,az,gx,gy,gz);
		delay(100);
	}
	while(1){
		motor(0,-50);
		motor(1,-50);
		delay(2000);
		motor(0,50);
		motor(1,50);
		delay(2000);
		motor(0,0);
		motor(1,0);
		delay(2000);
	}

	return 0;
}
