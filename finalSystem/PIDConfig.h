// COMMON SETTINGS

// speeds
#define Ksp 8
#define Ksi 5
#define Ksd 2

#define sIMax 50

// 14 desireable speed when motor running

#define NOM_VDES 16
#define MAX_VDES 19
int vDes = 16;


int prevDifference = 0;
int prevRightDifference = 0;

int iError = 0;
int iRError = 0;
#define angleTolerance 30


// PWM channel 
// right motor motor 1, left motor motor 2
#define leftPWM 23
#define rightPWM 13
