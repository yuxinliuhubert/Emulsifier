// COMMON SETTINGS

// speeds
#define Ksp 8
#define Ksi 3
#define Ksd 0

#define sIMax 50

// 14 desireable speed when motor running

#define NOM_VDES 14
#define MAX_VDES 44
int vDes = NOM_VDES;


int prevDifference = 0;
int prevRightDifference = 0;

int iError = 0;
int iRError = 0;
#define angleTolerance 30


// PWM channel 
// right motor motor 1, left motor motor 2
#define leftPWM 23
#define rightPWM 13
