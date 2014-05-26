/*
 * globals.h
 *
 *  Created on: 09/10/2013
 *      Author: rgap
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_


////////////////// ACTIONS
#define PARAR '1'
#define AVANZAR '2'
#define RETROCEDER '3'

#define  GIRO_DER '4'
#define  GIRO_IZQ '5'
#define  INICIAL_PALA '6'

#define  RECOGER_LATA '7'
#define  BUSCAR '8'
//////////////////

#define _DEBUG

#define SHOW_DISTANCE_POINT 1
#define SHOW_DEPTH_1C 1

//////////////////////////// OPTIONS
#define MOTOR_ABAJO 0
#define MOTOR_ARRIBA 1
////////////////////////////

#define MOTOR_ABAJO_ANGULO -102
#define MOTOR_ARRIBA_ANGULO -70

//////////////////////////// VENTANAS
#define SHOW_IM_INI 0
#define SHOW_IMENHANCED 0
#define SHOW_IMENHANCED_BLUR 0

#define SHOW_ACCELEROMETER 0

const float MAX_DEPTH = 700.0;

#endif /* GLOBALS_H_ */
