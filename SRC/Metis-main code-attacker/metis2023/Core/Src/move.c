/*
 * move.c
 *
 *  Created on: Jun 21, 2023
 *      Author: 98912
 */

#include "main.h"
#include "math.h"

void motor(int L1 , int L2 , int R2 , int R1){
	int c;
	if(look_dir == 'F') c = Heading;
	if(look_dir == 'B') c = Heading + 180;
	if(c > 180)  c -= 360;
	if(c < -180) c += 360;

	if(c > 100)  c = 100;
	if(c < -100) c = -100;
 	R1 += c;
 	R2 += c;
 	L1 += c;
 	L2 += c;

 	R1=R1*255;
 	R2=R2*255;
 	L1=L1*255;
 	L2=L2*255;
 	if (R1 > 65535) R1 = 65535;
 	if (R2 > 65535) R2 = 65535;
 	if (L1 > 65535) L1 = 65535;
 	if (L2 > 65535) L2 = 65535;
 	if (R1 < -65535) R1 = -65535;
 	if (R2 < -65535) R2 = -65535;
 	if (L1 < -65535) L1 = -65535;
 	if (L2 < -65535) L2 = -65535;

 	////////////////L1:
 	if(L1 > 0){
 		HAL_GPIO_WritePin(L1A_GPIO_Port, L1A_Pin, 0);
 		HAL_GPIO_WritePin(L1B_GPIO_Port, L1B_Pin, 1);
 		TIM1->CCR1=L1;
 	}
 	else if(L1 < 0){
 		HAL_GPIO_WritePin(L1A_GPIO_Port, L1A_Pin, 1);
 	 	HAL_GPIO_WritePin(L1B_GPIO_Port, L1B_Pin, 0);
 		TIM1->CCR1=-L1;
 	}
 	else{
 		HAL_GPIO_WritePin(L1A_GPIO_Port, L1A_Pin, 1);
		HAL_GPIO_WritePin(L1B_GPIO_Port, L1B_Pin, 1);
 	}
 	////////////////L2:
 	if(L2 > 0){
 		HAL_GPIO_WritePin(L2A_GPIO_Port, L2A_Pin, 0);
 		HAL_GPIO_WritePin(L2B_GPIO_Port, L2B_Pin, 1);
 		TIM1->CCR2=L2;
 	}
 	else if(L2 < 0){
 		HAL_GPIO_WritePin(L2A_GPIO_Port, L2A_Pin, 1);
 		HAL_GPIO_WritePin(L2B_GPIO_Port, L2B_Pin, 0);
 		TIM1->CCR2=-L2;
 	}
 	else{
 		HAL_GPIO_WritePin(L2A_GPIO_Port, L2A_Pin, 1);
		HAL_GPIO_WritePin(L2B_GPIO_Port, L2B_Pin, 1);
 	}
 	////////////////R2:
 	if(R2 > 0){
 		HAL_GPIO_WritePin(R2A_GPIO_Port, R2A_Pin, 1);
		HAL_GPIO_WritePin(R2B_GPIO_Port, R2B_Pin, 0);
 		TIM1->CCR3=R2;
 	}
 	else if(R2 < 0){
 		HAL_GPIO_WritePin(R2A_GPIO_Port, R2A_Pin, 0);
 		HAL_GPIO_WritePin(R2B_GPIO_Port, R2B_Pin, 1);
 		TIM1->CCR3=-R2;
 	}
 	else{
 		HAL_GPIO_WritePin(R2A_GPIO_Port, R2A_Pin, 1);
		HAL_GPIO_WritePin(R2B_GPIO_Port, R2B_Pin, 1);
 	}
 	////////////////R1:
 	if(R1 > 0){
 		HAL_GPIO_WritePin(R1A_GPIO_Port, R1A_Pin, 1);
 		HAL_GPIO_WritePin(R1B_GPIO_Port, R1B_Pin, 0);
 		TIM1->CCR4=R1;
 	}
 	else if(R1 < 0){
 		HAL_GPIO_WritePin(R1A_GPIO_Port, R1A_Pin, 0);
 	 	HAL_GPIO_WritePin(R1B_GPIO_Port, R1B_Pin, 1);
 		TIM1->CCR4=-R1;
 	}
 	else{
 		HAL_GPIO_WritePin(R1A_GPIO_Port, R1A_Pin, 1);
		HAL_GPIO_WritePin(R1B_GPIO_Port, R1B_Pin, 1);
 	}
 }
void motor_without_correction(int L1 , int L2 , int R2 , int R1){
 	R1=R1*255;
 	R2=R2*255;
 	L1=L1*255;
 	L2=L2*255;
 	if (R1 > 65535) R1 = 65535;
 	if (R2 > 65535) R2 = 65535;
 	if (L1 > 65535) L1 = 65535;
 	if (L2 > 65535) L2 = 65535;
 	if (R1 < -65535) R1 = -65535;
 	if (R2 < -65535) R2 = -65535;
 	if (L1 < -65535) L1 = -65535;
 	if (L2 < -65535) L2 = -65535;

 	////////////////L1:
 	if(L1 > 0){
 		HAL_GPIO_WritePin(L1A_GPIO_Port, L1A_Pin, 0);
 		HAL_GPIO_WritePin(L1B_GPIO_Port, L1B_Pin, 1);
 		TIM1->CCR1=L1;
 	}
 	else{
 		HAL_GPIO_WritePin(L1A_GPIO_Port, L1A_Pin, 1);
 	 	HAL_GPIO_WritePin(L1B_GPIO_Port, L1B_Pin, 0);
 		TIM1->CCR1=-L1;
 	}
 	////////////////L2:
 	if(L2 > 0){
 		HAL_GPIO_WritePin(L2A_GPIO_Port, L2A_Pin, 0);
 		HAL_GPIO_WritePin(L2B_GPIO_Port, L2B_Pin, 1);
 		TIM1->CCR2=L2;
 	}
 	else{
 		HAL_GPIO_WritePin(L2A_GPIO_Port, L2A_Pin, 1);
 		HAL_GPIO_WritePin(L2B_GPIO_Port, L2B_Pin, 0);
 		TIM1->CCR2=-L2;
 	}
 	////////////////R2:
 	if(R2 > 0){
 		HAL_GPIO_WritePin(R2A_GPIO_Port, R2A_Pin, 1);
		HAL_GPIO_WritePin(R2B_GPIO_Port, R2B_Pin, 0);
 		TIM1->CCR3=R2;
 	}
 	else{
 		HAL_GPIO_WritePin(R2A_GPIO_Port, R2A_Pin, 0);
 		HAL_GPIO_WritePin(R2B_GPIO_Port, R2B_Pin, 1);
 		TIM1->CCR3=-R2;
 	}
 	////////////////R1:
 	if(R1 > 0){
 		HAL_GPIO_WritePin(R1A_GPIO_Port, R1A_Pin, 1);
 		HAL_GPIO_WritePin(R1B_GPIO_Port, R1B_Pin, 0);
 		TIM1->CCR4=R1;
 	}
 	else{
 		HAL_GPIO_WritePin(R1A_GPIO_Port, R1A_Pin, 0);
 	 	HAL_GPIO_WritePin(R1B_GPIO_Port, R1B_Pin, 1);
 		TIM1->CCR4=-R1;
 	}
 }
void fast_stop(){
	 HAL_GPIO_WritePin(L1A_GPIO_Port, L1A_Pin, 1);
	 HAL_GPIO_WritePin(L1B_GPIO_Port, L1B_Pin, 1);
	 HAL_GPIO_WritePin(L2A_GPIO_Port, L2A_Pin, 1);
	 HAL_GPIO_WritePin(L2B_GPIO_Port, L2B_Pin, 1);
	 HAL_GPIO_WritePin(R1A_GPIO_Port, R1A_Pin, 1);
	 HAL_GPIO_WritePin(R1B_GPIO_Port, R1B_Pin, 1);
	 HAL_GPIO_WritePin(R2A_GPIO_Port, R2A_Pin, 1);
	 HAL_GPIO_WritePin(R2B_GPIO_Port, R2B_Pin, 1);
}
void move(int a){
	if(a>360)     a-=360;
	if(a<0)       a+=360;
	int x = v * cos(a * M_PI / 180);
	int y = v * sin(a * M_PI / 180);
	motor((x + y) , (x - y) , (- x - y), (y - x));

}
void move_without_correction(int a){
	if(a>360)     a-=360;
	if(a<0)       a+=360;
	int x = v * cos(a * M_PI / 180);
	int y = v * sin(a * M_PI / 180);
	motor_without_correction((x + y) , (x - y) , (- x - y), (y - x));
}
void moveForSec(int angle, int sec){
	for(int i=0; i<sec; i++){
		read_sensors();
		move(angle);
	}
}
void spin(int spin){
	if      (spin==1)  		 HAL_GPIO_WritePin(SPIN_GPIO_Port, SPIN_Pin, 1);
	else if (spin==0)		 HAL_GPIO_WritePin(SPIN_GPIO_Port, SPIN_Pin, 0);
}
void shoot(){
	if( already_shooted == 1){
		shoot_cnt++;
		if(shoot_cnt > 50)  already_shooted = 0;
	}
	HAL_GPIO_WritePin(SHOOT_GPIO_Port, SHOOT_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(SHOOT_GPIO_Port, SHOOT_Pin, 0);
	HAL_Delay(200);
	already_shooted = 1;
	shoot_cnt = 0;

}
void rotate_and_shoot(){
	spin(1);
	v = 60;
	if(mf_cnt < 5){
		arrived_to_goal = 0;
		move(0);
		mf_cnt++;
	}
	else{
		look_dir = 'B';
		if(Heading < 135 && Heading > -135 && arrived_to_goal == 0){
			if(Heading < 0) motor_without_correction(80, 80, 80, 0);
			else 			motor_without_correction(0, -80, -80, -80);
		}
		else{
			if (arrived_to_goal == 0) {
				move(goal_angle);
				if(kaf[15] > LDR_Sens || kaf[14] > LDR_Sens || kaf[13] > LDR_Sens || kaf[9] > LDR_Sens || kaf[6] > LDR_Sens) arrived_to_goal = 1;
				stop_before_shoot_cnt = 0;
			}
			else {
				if(goal_angle < 5 || goal_angle > 355) {
//					if(stop_before_shoot_cnt < 6) {
//						fast_stop();
//						stop_before_shoot_cnt++;
//					}else{
					fast_stop();
					shoot();
//						if(last_turn_dir == 'L') rotation_fix += 15;
//						if(last_turn_dir == 'R') rotation_fix -= 15;
//					}
				}
				else if(goal_angle < 180) {
					motor_without_correction(80, 80, 80, 0);
//					last_turn_dir = 'R';
					stop_before_shoot_cnt = 0;
				}
				else if(goal_angle >=180) {
					motor_without_correction(0, -80, -80, -80);
//					last_turn_dir = 'L';
					stop_before_shoot_cnt = 0;
				}

			}
		}
	}
}
void shift(){
	if(ball_in_kicker){
//		shoot();
		rotate_and_shoot();
//		shoot_straight();
//		come_back_cnt = 0;
//		last_move_cnt = 0;
	}
	else if (is_ball){
//		come_back_cnt = 0;
//		last_move_cnt = 0;
		arrived_to_goal = 0;
//		gate_cnt = 0;
		mf_cnt = 0;
		look_dir = 'F';
//		v = (ball_dist - 20) * 110 / 80 + 90;
		if(ball_dist > 58){ //far
			v = 100;
			move(ball_angle);
			spin(0);
		}
		else {//near
			spin(1);
			v = 145;
			if 		(ball_angle<10 || ball_angle > 350)   move(ball_angle);
			else if (ball_angle<30)       				  move(ball_angle+20);
			else if (ball_angle<90)       				  move(ball_angle+50);
			else if (ball_angle<180)       				  move(ball_angle+80);
			else if (ball_angle<270)       				  move(ball_angle-80);
			else if (ball_angle<330)       				  move(ball_angle-25);
			else if (ball_angle<=360)       			  move(ball_angle-20);
		}
	}
	else {
		mf_cnt = 0;
		spin(0);
		fast_stop();
//		come_back();
		look_dir = 'F';
	}

}
void stop(){
	motor(0,0,0,0);
}
