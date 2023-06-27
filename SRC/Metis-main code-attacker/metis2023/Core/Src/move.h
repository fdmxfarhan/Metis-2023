/*
 * move.h
 *
 *  Created on: Jun 21, 2023
 *      Author: 98912
 */

#ifndef SRC_MOVE_H_
#define SRC_MOVE_H_



#endif /* SRC_MOVE_H_ */

void motor(int L1 , int L2 , int R2 , int R1);
void motor_without_correction(int L1 , int L2 , int R2 , int R1);
void fast_stop();
void move(int a);
void moveForSec(int angle, int sec);
void spin(int spin);
void shift();
void stop();
void shoot();
void move_without_correction(int a);
void rotate_and_shoot();
