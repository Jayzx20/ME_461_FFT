//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// To Move FFT DMA code to your Robot or Homework Projects look for the !!!!!!! comments
// For blocks of code to copy into your other project.  This will give you code to take
// FFT of the microphone connected to ADCINB4
//!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code to your global variable declarations
//*****************************************************************************
// the defines for FFT
//*****************************************************************************
#define RFFT_STAGES     10
#define RFFT_SIZE       (1 << RFFT_STAGES)

//*****************************************************************************
// the globals
//*****************************************************************************
#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(pwrSpec, "FFT_buffer_2")
#endif
float pwrSpec[(RFFT_SIZE/2)+1];
float maxpwr = 0;
int16_t maxpwrindex = 0;

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(test_output, "FFT_buffer_2")
#endif
float test_output[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_1")
#else
#pragma DATA_SECTION(fft_input, "FFT_buffer_1")
#endif
float fft_input[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(RFFTF32Coef,"FFT_buffer_2")
#endif //__cplusplus
//! \brief Twiddle Factors
//!
float RFFTF32Coef[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(AdcPingBufRaw, "FFT_buffer_2")
#endif
uint16_t AdcPingBufRaw[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(AdcPongBufRaw, "FFT_buffer_2")
#endif
uint16_t AdcPongBufRaw[RFFT_SIZE];


//! \brief Object of the structure RFFT_F32_STRUCT
//!
RFFT_F32_STRUCT rfft;

//! \brief Handle to the RFFT_F32_STRUCT object
//!
RFFT_F32_STRUCT_Handle hnd_rfft = &rfft;
uint16_t pingFFT = 0;
uint16_t pongFFT = 0;
uint16_t pingpongFFT = 1;
uint16_t iPingPong = 0;
int16_t DMAcount = 0;
__interrupt void DMA_ISR(void);
void InitDma(void);
//!!!!!!!!!!!!!!!!!!!!!!  End of Block


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);
void setupSpib(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
uint16_t updown = 1;
uint16_t updown_2 = 1;
uint16_t PWM1 = 0;
uint16_t PWM2 = 0;
float ADC_READ_1;
float ADC_READ_2;
int16_t dummy= 0;
int16_t ACCEL_X_RAW = 0;
int16_t ACCEL_Y_RAW = 0;
int16_t ACCEL_Z_RAW = 0;
int16_t TEMP = 0;
int16_t GYRO_X_RAW = 0;
int16_t GYRO_Y_RAW = 0;
int16_t GYRO_Z_RAW = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

float gAngleLeft;
float gAngleRight;
float gDistanceLeft;
float gDistanceRight;

float uLeft = 0;
float uRight = 0;

float PosLeft_K_1 = 0.0;
float PosLeft_K = 0.0;
float VLeftK;

float PosRight_K_1 = 0.0;
float PosRight_K = 0.0;
float VRightK;

float EK_L;
float Vref = 0;
float IK_L;
float IK_1_L;
float EK_1_L;
float KP_L = 3.0;
float Ki_L = 25.0;

float EK_R;
float IK_R;
float IK_1_R;
float EK_1_R;
float KP_R = 3.0;
float Ki_R = 25.0;

float KPturn = 3.0;
float Eturn;
float turn;

float voltsADCIND0 = 0;
float voltsADCINA2 = 0;
float voltsADCINA3 = 0;
uint32_t ADCAcount = 0;


//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
//float xk = 0;
//float xk_1 = 0;
//float xk_2 = 0;
//float xk_3 = 0;
//float xk_4 = 0;
//yk is the filtered value
float yk = 0;
float yka2 = 0;
float yka3 = 0;
float ykb4 = 0;
float XKa2[22]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float XKa3[22]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float b[22]={-2.3890045153263611e-03, -3.3150057635348224e-03, -4.6136191242627002e-03, -4.1659855521681268e-03, 1.4477422497795286e-03, 1.5489414225159667e-02,
             3.9247886844071371e-02, 7.0723964095458614e-02, 1.0453473887246176e-01, 1.3325672639406205e-01, 1.4978314227429904e-01, 1.4978314227429904e-01, 1.3325672639406205e-01,
             1.0453473887246176e-01, 7.0723964095458614e-02, 3.9247886844071371e-02, 1.5489414225159667e-02, 1.4477422497795286e-03, -4.1659855521681268e-03, -4.6136191242627002e-03,
             -3.3150057635348224e-03, -2.3890045153263611e-03};

uint16_t add = 0;

float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = 0.615; // Adjust every time for new battery
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0;
float LeftWheel_1 = 0;
float RightWheel = 0;
float RightWheel_1 = 0;
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;
float velRight = 0;
float velRight_1 = 0;
float velLeft = 0;
float velLeft_1 = 0;
float K1 = -60;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;
float ubal = 0;
float gyrorate_dot = 0;
float gyrorate_1 = 0;
float gyrorate_dot_1 = 0;

float WhlDiff = 0;
float turnref = 0;
float WhlDiff_1 = 0;
float vel_WhlDiff_1 = 0;
float vel_WhlDiff = 0;
float errorDiff = 0;
float errorDiff_1 = 0;
float intDiff = 0;
float intDiff_1 = 0;
float FwdBackOffset = 0;
float FwdBackOffset_1 = 0;
float Kp = 3.0;
float Ki = 20.0;
float Kd = 0.08;
float turnrate_1;
float turnrate;
float turnref_1;
float AvgWheelVel;
float AvgWheelVel_1;
float DVel;
float KpVel = 0.35;
float KiVel = 1.5;
float intDiffVel;
float intDiffVel_1;
float errorvel;
float errorvel_1;

// Copy from Lab 6
float LW = 0; // Predefine value use for lab 6 exercise 1
float RW = 0; // Predefine value use for lab 6 exercise 1
float LWF = 0; // Predefine value use for lab 6 exercise 1
float RWF = 0; // Predefine value use for lab 6 exercise 1
float uLeft6 = 5.0; // Predefine value use for lab 6 exercise 1
float uRight6 = 5.0; // Predefine value use for lab 6 exercise 1
float PLK = 0; // Predefine value use for lab 6 exercise 2
float PLK_1 = 0; // Predefine value use for lab 6 exercise 2
float VLK = 0; // Predefine value use for lab 6 exercise 2
float PRK = 0; // Predefine value use for lab 6 exercise 2
float PRK_1 = 0; // Predefine value use for lab 6 exercise 2
float VRK = 0; // Predefine value use for lab 6 exercise 2
float eKL = 0; // Predefine value use for lab 6 exercise 3
float eKR = 0; // Predefine value use for lab 6 exercise 3
float Vref6 = 0; // Predefine value use for lab 6 exercise 3
float vK = 0; // Predefine value use for lab 6 exercise 3
float IKL = 0; // Predefine value use for lab 6 exercise 3
float IKL1 = 0; // Predefine value use for lab 6 exercise 3
float eKL1 = 0; // Predefine value use for lab 6 exercise 3
float eKR1 = 0; // Predefine value use for lab 6 exercise 3
float IKR = 0; // Predefine value use for lab 6 exercise 3
float IKR1 = 0; // Predefine value use for lab 6 exercise
float uKL = 0; // Predefine value use for lab 6 exercise 3
float uKR = 0; // Predefine value use for lab 6 exercise 3
float Kp6 = 3; // Predefine value use for lab 6 exercise 3
float Ki6 = 25.0; // Predefine value use for lab 6 exercise 3
float KpT = 3; // Predefine value use for lab 6 exercise 4
float eT = 0; // Predefine value use for lab 6 exercise 4
float turn6 = 0;
// Copy from Lab 6 end

// state machine
uint16_t soundstate = 10;
float time10;
float time20;
float time30;
float time40;
float time50;
float time60;
float time70;
float time80;
float time90;
float time110;
float time120;
// state machine end

// Robot car mode
float circle = 0;
float square = 0;
uint16_t RSN = 0;
float FwdCount = 0;
float TurnCount = 0;
float backRight = 0;
float backLeft = 0;
float stop = 0;
float wallfollowing = 0;
//Segbot mode
float SegbotCircle = 0;
float segbot = 0;
int16_t wheelie = 1;
int16_t WheelieTime = 0;
int16_t accel_time = 1001;
int16_t stop_time = 1002;
int16_t balance_time = 1003;
int16_t end_time = 1004;
float uLeftMax = 8.5;
float uRightMax = 8.5;
float uLeftStop = 0;
float uRightStop = 0;
//Segbot mode end

// RCservo
float IR_angle = -10;
// RCservo end

// IR Sensor
uint16_t adca0result = 0;
uint16_t adca1result = 0;
uint16_t adca2result = 0;
float voltsADCINA_Left;
float voltsADCINA_Right;
float voltsADCINA_Front;
// IR Sensor end

// Global function
void PSM(void); // Program state machine function
int freIndex(float fre); // Frequency input to index converter
void RBSM(void);// Robot state machine
void setupSpib(void);
void setEPWM2A(float controleffort){ //takes control effort (a value -10 to 10) as a float
    if(controleffort >= 10.0){
        controleffort = 10.0; //if control effort is larger than 10, it saturates to the saturation limit of 10
    }
    if(controleffort <= -10.0){
        controleffort = -10.0; //if control effort is smaller than -10, it saturates to the saturation limit of -10
    }
    EPwm2Regs.CMPA.bit.CMPA = (EPwm2Regs.TBPRD/2) + controleffort * (EPwm2Regs.TBPRD/20); // control the duty cycle of the motor start from 0 as 50% duty cycle.
}

void setEPWM2B(float controleffort){ //takes control effort (a value -10 to 10) as a float
    if(controleffort >= 10.0){
        controleffort = 10.0; //if control effort is larger than 10, it saturates to the saturation limit of 10
    }
    if(controleffort <= -10.0){
        controleffort = -10.0; //if control effort is smaller than -10, it saturates to the saturation limit of -10
    }
    EPwm2Regs.CMPB.bit.CMPB = (EPwm2Regs.TBPRD/2) + controleffort * (EPwm2Regs.TBPRD/20); // control the duty cycle of the motor start from 0 as 50% duty cycle.
}

// Zehan
void setEPWM8A_RCServo(float angle){ // Exercise 3 Function that control the RC servo that connect to GPIO14, EPwm8A
    if (angle >= 90){ // When angle reach 90, the angle will not exceed 90
        angle = 90;
    }
    if (angle <= -90){ // When angle reach 90, the angle will not exceed 90
        angle = -90;
    }
    EPwm8Regs.CMPA.bit.CMPA = (angle+90)*(2500/180)+1250; // Start the angle to 0 degree which the duty cycle is 8%
}
// Zehan end

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (-raw*(PI/6000)); // (2pi/400 * 1/30)
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(PI/6000)); // (2pi/400 * 1/30)
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}


__interrupt void ADCA_ISR(void){
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;
    adca2result = AdcaResultRegs.ADCRESULT2;
    voltsADCINA_Left = 3.0 - adca0result*(3.0/4096.0);
    voltsADCINA_Right = 3.0 - adca1result*(3.0/4096.0);
    voltsADCINA_Front = 3.0 - adca2result*(3.0/4096.0); //will read lower number for closer object

    //    setDACA(voltsADCIND0);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; //Clear GPIO9 Low
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when twelve values are in the RX FIFO
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00)); //
    SpibRegs.SPITXBUF = 0; // ACCEL_X
    SpibRegs.SPITXBUF = 0; // ACCEL_Y
    SpibRegs.SPITXBUF = 0; // ACCEL_Z
    SpibRegs.SPITXBUF = 0;// TEMP
    SpibRegs.SPITXBUF = 0; // GYRO_X
    SpibRegs.SPITXBUF = 0; // GYRO_Y
    SpibRegs.SPITXBUF = 0; // GYRO_z

    ADCAcount++;

    XKa2[0] = voltsADCINA2;
    XKa3[0] = voltsADCINA3;

    //yk = b[0]*XK[0] + b[1]*XK[1]  + b[2]*XK[2]  + b[3]*XK[3]  + b[4]*XK[4] ;
    yka2 = 0;
    yka3 = 0;
    //    for(int i=0;i<5;i++){
    //        yk = yk + b[i]*XK[i];
    //    }
    for (int i=0; i<22; i++)
    {
        yka2 = yka2 + b[i]*XKa2[i];
        yka3 = yka3 + b[i]*XKa3[i];
    }

    //Save past states before exiting from the function so that next sample they are the older state
    //    XK[4]  = XK[3] ;
    //    XK[3]  = XK[2] ;
    //    XK[2]  = XK[1] ;
    //    XK[1]  = XK[0] ;

    //    for(int i=0;i<4;i++){
    //        XK[4-i] = XK[3-i];
    //    }

    for (int i=0; i<21; i++)
    {
        XKa2[21-i] = XKa2[20-i];
        XKa3[21-i] = XKa3[20-i];
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;


    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    //!!!!!!!!!!!!!!!!!!!!!!  Copy the Assignmnt of the DMA interrupt service routine
    PieVectTable.DMA_CH1_INT = &DMA_ISR;
    //!!!!!!!!!!!!!!!!!!!!!!  End of Block

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.SPIB_RX_INT = &SPIB_isr;// Set up PieVect Table for our interrupt
    PieVectTable.ADCA1_INT = &ADCA_ISR;// Set up PieVect Table for our interrupt

    EDIS;    // This is needed to disable write to EALLOW protected registers



    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    //!!!!!!!!!!!!!!!!!!!!!! DMAFFT Copy this block of code after your init_serial functions
    EALLOW;
    EPwm7Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm7Regs.ETSEL.bit.SOCASEL = 0x2; // Select Event when counter equal to PRD
    EPwm7Regs.ETPS.bit.SOCAPRD = 0x1; // Generate pulse on 1st event
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm7Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm7Regs.TBPRD = 5000; // Set Period to 0.1ms sample. Input clock is 50MHz.
    EPwm7Regs.ETSEL.bit.SOCAEN = 1; // Disable SOC on A group
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal EPwm7Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm7Regs.TBCTL.bit.CTRMODE = 0x00; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1; //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x11; // EPWM7 ADCSOCA or another trigger you choose will trigger SOC0
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag, originally = 1 (need to set to 1 for non-DMA)
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;     // Interrupt pulses regardless of flag state
    EDIS;
    // Initialize DMA
    InitDma();
    //!!!!!!!!!!!!!!!!!!!!!!  End of Block

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD (???)
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (pulse is the same as trigger) (???)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz. (TBPRD * 1/50000000 = 1/1000)
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    //    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode (???)
    EDIS;

    EALLOW;
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 13;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;

    // Zehan
    EPwm8Regs.TBCTL.bit.CTRMODE = 0; // Set TBCTL in Up count mode.
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2; // Let the PWM go free run when I set a breakpoint.
    EPwm8Regs.TBCTL.bit.PHSEN = 0; // disable the phase loading.
    EPwm8Regs.TBCTL.bit.CLKDIV = 5; // Clock divide by 32.
    EPwm8Regs.TBCTR = 0; // Start the timer at zero.
    EPwm8Regs.TBPRD = 15000; // Set peri od to 50Hz, 31250/50000000/32=1/50
    EPwm8Regs.CMPA.bit.CMPA = 2500; // Start duty cycle 8%.
    EPwm8Regs.CMPB.bit.CMPB = 2500; // Start duty cycle 8%.
    EPwm8Regs.AQCTLA.bit.CAU = 1; // Set TBCTR to clear when reach CMPA value.
    EPwm8Regs.AQCTLA.bit.ZRO = 2; // Set TBCTR to set when TBCTR is zero.
    EPwm8Regs.AQCTLB.bit.CBU = 1; // Set TBCTR for to clear when reach CMPA value. EPWM2B
    EPwm8Regs.AQCTLB.bit.ZRO = 2; // Set TBCTR to set when TBCTR is zero.EPWM2B
    EPwm8Regs.TBPHS.bit.TBPHS = 0; // Set phase to zero.

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); // Set GPIO14 is EPWM8A output pin.////////////////////////////////////////////////////////////////
    // Zehan end

    //EPWM2
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; //Count Up Mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; //Free soft emulation to free run
    EPwm2Regs.TBCTL.bit.PHSEN = 0; //Disable phase loading
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; //Clock divide by 1

    EPwm2Regs.TBCTR = 0;

    EPwm2Regs.TBPRD = 2500; //TBPRD * 1/50000000 = 1/20000 -> 2500 solve for TBPRD

    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;

    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;

    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index

    init_eQEPs();
    setupSpib();

    //    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    //    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    //    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    //    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    //    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    //    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    //    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    //    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    //    EALLOW;
    //    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    //    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    //    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    //    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    EDIS;
    //    // ---------------------------------------------------------------------------
    //    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset(???)
    //    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    //    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    //    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master (???)
    //    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
    //    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission (???)
    //    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    //    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt (???)
    //    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // (???) Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    //    // 50MHZ. And this setting divides that base clock to create SCLKs period
    //    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    //    SpibRegs.SPIFFTX.bit.SPIRST = 1;// (???) Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    //    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // (???) Enable SPI FIFO enhancements
    //    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    //    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    //    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    //    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // (???) Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    //    SpibRegs.SPIFFCT.bit.TXDLY = 0; // (???) Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    //    SpibRegs.SPICCR.bit.SPISWRESET = 1; // (???) Pull the SPI out of reset
    //    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // (???) Release transmit FIFO from reset.
    //    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    //    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I dont think this is needed. Need to Test
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 10; //(???) Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below


    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode (???)
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable SWI in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    //!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code right before your EINT; line of code
    int16_t i = 0;
    float samplePeriod = 0.0001;

    // Clear input buffers:
    for(i=0; i < RFFT_SIZE; i++){
        fft_input[i] = 0.0f;
    }

    for (i=0;i<RFFT_SIZE;i++) {
        fft_input[i] = sin(125*2*PI*i*samplePeriod)+2*sin(2400*2*PI*i*samplePeriod);
    }
    hnd_rfft->FFTSize   = RFFT_SIZE;
    hnd_rfft->FFTStages = RFFT_STAGES;
    hnd_rfft->InBuf     = &fft_input[0];  //Input buffer
    hnd_rfft->OutBuf    = &test_output[0];  //Output buffer
    hnd_rfft->MagBuf    = &pwrSpec[0];  //Magnitude buffer

    hnd_rfft->CosSinBuf = &RFFTF32Coef[0];  //Twiddle factor buffer
    RFFT_f32_sincostable(hnd_rfft);         //Calculate twiddle factor

    for (i=0; i < RFFT_SIZE; i++){
        test_output[i] = 0;               //Clean up output buffer
    }

    for (i=0; i <= RFFT_SIZE/2; i++){
        pwrSpec[i] = 0;                //Clean up magnitude buffer
    }


    int16_t tries = 0;
    while(tries < 10*0) {  // Get ride of the 0 in 10*0 if you want to run this while loop and test out the FFT function with these sin waves
        RFFT_f32(hnd_rfft);                     //Calculate real FFT

#ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project
        // properties
        RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
#else
        RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
#endif
        maxpwr = 0;
        maxpwrindex = 0;

        for (i=0;i<(RFFT_SIZE/2);i++) {
            if (pwrSpec[i]>maxpwr) {
                maxpwr = pwrSpec[i];
                maxpwrindex = i;
            }
        }

        tries++;
        for (i=0;i<RFFT_SIZE;i++) {
            fft_input[i] = sin((125 + tries*125)*2*PI*i*samplePeriod)+2*sin((2400-tries*200)*2*PI*i*samplePeriod);
        }
    }
    //!!!!!!!!!!!!!!!!!!!!!!  End of Block
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        //        if (UARTPrint == 1 ) {
        //            serial_printf(&SerialA,"ACCEL_X: %.2f ACCEL_Y: %.2f ACCEL_Z: %.2f GYRO_X: %.2f GYRO_Y: %.2f GYRO_Z: %.2f\r\n",ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z);
        //            UARTPrint = 0;
        //        }
        if (UARTPrint == 1 ) {
            // serial_printf(&SerialA,"AngL:%.2f AngR:%.2f gyrox:%.2f accelz:%.2f ADCA2:%.2f ADCA3:%.2f tiltval:%.2f gyroval:%.2f LeftWheel:%.2f RightWheel:%.2f\r\n",gAngleLeft,gAngleRight,gyrox,accelz,voltsADCINA2,voltsADCINA3, tilt_value, gyro_value, LeftWheel, RightWheel);
            serial_printf(&SerialA, "Power: %.3f Frequency: %.0f \r\n", maxpwr, maxpwrindex*10000.0/1024.0);
            UARTPrint = 0;
        }
        //!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code after your UARTPrint == 1 while loop as above
        if ( (pingFFT == 1) || (pongFFT == 1) ) {
            if (pingFFT == 1) {
                pingFFT = 0;
                // Raw ADC data
                for(i=0; i<RFFT_SIZE; i++) {
                    //--- Read the ADC results:
                    fft_input[i] = AdcPingBufRaw[i]*3.0/4095.0;  // ping data
                }
            } else if (pongFFT == 1) {
                pongFFT = 0;
                // Raw ADC data
                for(i=0; i<RFFT_SIZE; i++) {
                    //--- Read the ADC result:
                    fft_input[i] = AdcPongBufRaw[i]*3.0/4095.0;  // pong data
                }
                //                hnd_rfft->InBuf     = &fft_input[0];  //Input buffer
            }

            RFFT_f32(hnd_rfft);

#ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project
            // properties
            RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
#else
            RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
#endif

            maxpwr = 0;
            maxpwrindex = 0;

            for (i=5;i<(RFFT_SIZE/2);i++) {
                if (pwrSpec[i]>maxpwr) {
                    maxpwr = pwrSpec[i];
                    maxpwrindex = i;
                }
            }

            UARTPrint = 1;
        }
        //!!!!!!!!!!!!!!!!!!!!!!  End of Block
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    switch(soundstate){

    case 10:
        time10++;
        if(time10 > 500 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 4500 && maxpwrindex*10000.0/1024.0 < 5000){ // Wheelie 10-->20-->30
                soundstate = 20;
                time20 = 0;
            }
            else if (maxpwrindex*10000.0/1024.0 > 3500 && maxpwrindex*10000.0/1024.0 < 4000){ // Segbot Circle 10-->40-->50
                soundstate = 40;
                time40 = 0;
            }
            else if (maxpwrindex*10000.0/1024.0 > 2500 && maxpwrindex*10000.0/1024.0 < 3000){ // Robot Circle 10-->60-->70
                soundstate = 60;
                time60 = 0;
            }
            else if (maxpwrindex*10000.0/1024.0 > 1500 && maxpwrindex*10000.0/1024.0 < 2500){ // Robot square 10-->80-->90
                soundstate = 80;
                time80 = 0;
            }
            else if (maxpwrindex*10000.0/1024.0 > 3000 && maxpwrindex*10000.0/1024.0 < 3500){ // Robot Stop 10-->110-->120
                soundstate = 110;
                time110 = 0;
            }

        }
        break;

        // Wheelie 10-->20-->30
    case 20:
        time20++;
        if(time20 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 3000 && maxpwrindex*10000.0/1024.0 < 4500){
                soundstate = 30;
                time30 = 0;
            }
        }
        else if (time20 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;

    case 30:
        time30++;
        if(time30 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 2500 && maxpwrindex*10000.0/1024.0 < 3500){
                segbot = 1;
                wheelie = 1;
                SegbotCircle = 0;
                circle = 0;
                stop = 0;
                soundstate = 10;
                time10 = 0;
                time30 = 0;
            }
        }
        else if (time30 > 655) {
            soundstate = 10;
            time10 = 0;
        }
        break;

        // Segbot Circle 10-->40-->50
    case 40:
        time40++;
        if(time40 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 3000 && maxpwrindex*10000.0/1024.0 < 3500){
                soundstate = 50;
                time50 = 0;
            }
        }
        else if (time40 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;

    case 50:
        time50++;
        if(time50 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 750 && maxpwrindex*10000.0/1024.0 < 1750){
                SegbotCircle = 1;
                segbot = 1;
                soundstate = 10;
                circle = 0;
                stop = 0;
                time10 = 0;
                stop = 0;
            }
        }
        else if (time50 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;

        // Robot Circle 10-->60-->70
    case 60:
        time60++;
        if(time60 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 2000 && maxpwrindex*10000.0/1024.0 < 2500){
                soundstate = 70;
                time70 = 0;
            }
        }
        else if (time60 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;
    case 70:
        time70++;
        if(time70 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 3500 && maxpwrindex*10000.0/1024.0 < 4500){
                segbot = 0;
                circle = 1;
                square = 0;
                SegbotCircle = 0;
                wheelie = 0;
                stop = 0;
                soundstate = 10;
                time10 = 0;
            }
        }
        else if (time70 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;

        // Robot square 10-->80-->90
    case 80:
        time80++;
        if(time80 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 2250 && maxpwrindex*10000.0/1024.0 < 3250){
                soundstate = 90;
                time90 = 0;
            }
        }
        else if (time80 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;
    case 90:
        time90++;
        if(time90 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 3500 && maxpwrindex*10000.0/1024.0 < 4500){
                segbot = 0;
                circle = 0;
                square = 1;
                RSN = 0;
                stop = 0;
                SegbotCircle = 0;
                wheelie = 0;
                soundstate = 10;
                time10 = 0;
            }
        }
        else if (time90 > 650){
            soundstate = 10;
            time10 = 0;
        }
        break;

    case 110:
        time110++;
        if(time110 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 1000 && maxpwrindex*10000.0/1024.0 < 1500){
                soundstate = 120;
                time120 = 0;
            }
        }
        else if (time110 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;
    case 120:
        time120++;
        if(time120 < 650 && maxpwr > 50){
            if(maxpwrindex*10000.0/1024.0 > 1000 && maxpwrindex*10000.0/1024.0 < 1500){
                stop = 1;
                segbot = 0;
                circle = 0;
                square = 0;
                SegbotCircle = 0;
                wheelie = 0;
                soundstate = 10;
                time10 = 0;
            }
        }
        else if (time120 > 655){
            soundstate = 10;
            time10 = 0;
        }
        break;
    }
    // Zehan Servo
    if ((GpioDataRegs.GPADAT.bit.GPIO5)== 1) {
        setEPWM8A_RCServo(IR_angle);
    }
    if ((GpioDataRegs.GPADAT.bit.GPIO5)== 0) {
        setEPWM8A_RCServo(-90.0);
    }
    // Zehan Servo End

    if(segbot == 0){
        // Robot Speed control
        LW = readEncLeft(); // in radiant
        RW = readEncRight(); // in radiant
        if (GpioDataRegs.GPADAT.bit.GPIO4 == 1 && GpioDataRegs.GPADAT.bit.GPIO6 == 1 && backRight == 0 && backLeft == 0){
            if (circle == 1){
                Vref6 = 1;//1
                turn6 = 1;//1
            }
            else if (square == 1)
            {
                switch(RSN){

                // For PSM() Case 3

                case 0: // Forward
                    turn6 = 0;
                    Vref6 = 0;
                    FwdCount = 0;
                    RSN = 2;
                    break;

                case 1: // Turn Left
                    turn6 = 0;
                    Vref6 = 0;
                    TurnCount = 0;
                    RSN = 3;
                    break;

                case 2: // Forward
                    FwdCount++;
                    Vref6 = 1.25;//1.25
                    if (FwdCount > 750){ // Change to turn command after move about 3s
                        RSN = 1;
                    }
                    else{ // Stay in Fwd command
                        RSN = 2;
                    }
                    break;

                case 3: //Left Turn
                    TurnCount++;
                    turn6 = 1.571;//1.571
                    if (TurnCount > 165){ // Change to Fwd command after turn about 0.5s
                        RSN = 0;
                    }
                    else{ // Stay in turn command
                        RSN = 3;
                    }
                    break;
                }
            }
            else if(stop == 1) {
                Vref6 = 0;
                turn6 = 0;
                turnrate = 0;
                uLeft = 0;
                uRight = 0;
            }
            else if(wallfollowing == 1){
                // Right Wall Following
                if(voltsADCINA_Front > 1.9 && voltsADCINA_Right < 0.9 && voltsADCINA_Left > 1.8){// Forward Only
                    Vref6 = 0.8;
                }
                else if(voltsADCINA_Front > 1.9 && voltsADCINA_Right >= 0.9 && voltsADCINA_Right <= 1.9 && voltsADCINA_Left > 1.8){
                    Vref6 = 0.8;
                    turn6 = -0.25; // Turn Right
                }
                else if(voltsADCINA_Front > 1.9 && voltsADCINA_Right > 1.9 && voltsADCINA_Left > 1.8){
                    Vref6 = 0.8;
                    turn6 = -0.5; // Turn Right
                }
                else if (voltsADCINA_Front <= 1.9 && voltsADCINA_Right < 0.9 && voltsADCINA_Left > 1.8){
                    Vref6 = 0.25;
                    turn6 = 0.5; // Turn Left
                }
                else if (voltsADCINA_Front <= 1.9 && voltsADCINA_Right >= 0.9 && voltsADCINA_Left > 1.8){
                    Vref6 = 0.25;
                    turn6 = 0.25; // Turn Left
                }
            }
        }
        else{
            if((GpioDataRegs.GPADAT.bit.GPIO4) == 0 || backRight > 0){ //Right
                backRight++;
                Vref6 = -0.75;
                turn6 = 1;
                if (backRight >= 250){
                    backRight = 0;
                }

            }
            if ((GpioDataRegs.GPADAT.bit.GPIO6) == 0 || backLeft > 0){ //Left
                backLeft++;
                Vref6 = -0.75;
                turn6 = -1;
                if (backLeft >= 250){
                    backLeft = 0;
                }
            }

        }

        PLK = LW/4.95;
        VLK = (PLK - PLK_1)/0.004;
        PLK_1 = PLK;
        PRK = RW/4.95;
        VRK = (PRK - PRK_1)/0.004;
        PRK_1 = PRK;
        eT = turn6 +(VLK - VRK);
        eKL = Vref6 - VLK - KpT * eT;
        eKR = Vref6 - VRK + KpT * eT;
        IKL = IKL1 + 0.004 * (eKL+eKL1)/2;
        uKL = Kp6 * eKL + Ki6 * IKL;
        IKR = IKR1 + 0.004 * (eKR+eKR1)/2;
        uKR = Kp * eKR + Ki * IKR;
        eKL1 = eKL;
        eKR1 = eKR;
        if(uKL >= 10){
            uKL = 10;
        }
        else if(uKL <= -10){
            uKL = -10;
        }
        else{
            IKL1 = IKL;
        }
        if(uKR >= 10){
            uKR = 10;
        }
        else if(uKR <= -10){
            uKR = -10;
        }
        else{
            IKR1 = IKR;
        }


        setEPWM2A(uKR);
        setEPWM2B(-uKL);
    }
    // End
    else{
        // Segbot

        velLeft = 0.6*velLeft_1 + 100*LeftWheel - 100*LeftWheel_1;
        velRight = 0.6*velRight_1 + 100*RightWheel - 100*RightWheel_1;

        gyrorate_dot = 0.6*gyrorate_dot_1 + 100*gyro_value - 100*gyrorate_1;

        ubal = -K1*tilt_value - K2*gyro_value - K3*(velLeft + velRight)/2.0 - K4*gyrorate_dot;
        // Insert SWI ISR Code here.......

        AvgWheelVel = (velLeft + velRight)/2;
        errorvel = DVel-AvgWheelVel;
        intDiffVel = intDiffVel_1 + 0.004*(errorvel + errorvel_1)/2;
        FwdBackOffset = errorvel * KpVel + KiVel * intDiffVel;

        WhlDiff = LeftWheel - RightWheel;
        vel_WhlDiff = (166.667 * WhlDiff) - (166.667 * WhlDiff_1) + (0.333 * vel_WhlDiff_1);

        errorDiff = turnref - WhlDiff;

        intDiff = intDiff_1 + 0.004*(errorDiff + errorDiff_1)*0.5;

        turnref = turnref_1 + 0.004*(turnrate + turnrate_1)*0.5;

        turn = Kp*errorDiff+Ki*intDiff - Kd*vel_WhlDiff;

        if(fabs(FwdBackOffset) > 3){
            FwdBackOffset = FwdBackOffset_1;
        }
        else{
            FwdBackOffset_1 = FwdBackOffset;
        }

        if(FwdBackOffset > 4){
            FwdBackOffset = 4;
        }
        if(FwdBackOffset < -4){
            FwdBackOffset = -4;
        }

        if(fabs(turn) > 3){
            intDiff = intDiff_1;
        } else{
            intDiff_1 = intDiff;
        }

        if(turn > 4){
            turn = 4;
        }
        if(turn < -4){
            turn = -4;
        }
        uLeft = ubal/2 + turn - FwdBackOffset;
        uRight = ubal/2 - turn - FwdBackOffset;

        WheelieTime++;
        if (wheelie == 1){
            if(WheelieTime <1000){
                uLeft = 0;
                uRight = 0;
            } else if(WheelieTime < accel_time && WheelieTime > 1000){
                uLeft = uLeftMax;
                uRight = uRightMax;
            } else if(WheelieTime < stop_time && WheelieTime > accel_time){
                uLeft = uLeftStop;
                uRight = uRightStop;
            } else if (WheelieTime < balance_time && WheelieTime > stop_time){
                uLeft = ubal/2;
                uRight = ubal/2;
            } else if (WheelieTime < end_time && WheelieTime > balance_time){
                wheelie = 0;
                WheelieTime = 0;
                turnrate = 0;
            }
        }

        if(SegbotCircle == 1){
            turnrate = 1;
            // DVel = 0.2;
        }

        setEPWM2A(uRight); //2A is the right wheel
        setEPWM2B(-uLeft); //2B is the left wheel

        velLeft_1 = velLeft;
        velRight_1 = velRight;
        LeftWheel_1 = LeftWheel;
        RightWheel_1 = RightWheel;
        gyrorate_dot_1 = gyrorate_dot;
        gyrorate_1 = gyro_value;
        errorDiff_1 = errorDiff;
        vel_WhlDiff_1 = vel_WhlDiff;
        AvgWheelVel_1 = AvgWheelVel;
        intDiff_1 = intDiff;
        turnref_1 = turnref;
        turnrate_1 = turnrate;
        intDiffVel_1 = intDiffVel;
        errorvel_1 = errorvel;
    }
    // End

    numSWIcalls++;

    DINT;

}

__interrupt void SPIB_isr(void){
    add++;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummy = SpibRegs.SPIRXBUF;
    ACCEL_X_RAW = SpibRegs.SPIRXBUF; // ACCEL_X
    ACCEL_Y_RAW = SpibRegs.SPIRXBUF; // ACCEL_Y
    ACCEL_Z_RAW = SpibRegs.SPIRXBUF; // ACCEL_Z
    TEMP = SpibRegs.SPIRXBUF;// TEMP
    GYRO_X_RAW = SpibRegs.SPIRXBUF; // GYRO_X
    GYRO_Y_RAW = SpibRegs.SPIRXBUF; // GYRO_Y
    GYRO_Z_RAW = SpibRegs.SPIRXBUF; // GYRO_z


    accelx = ACCEL_X_RAW*4.0/32767.0; //Scale to the correct units
    accely = ACCEL_Y_RAW*4.0/32767.0; //Scale to the correct unit
    accelz = ACCEL_Z_RAW*4.0/32767.0; //Scale to the correct unit

    gyrox = GYRO_X_RAW*250.0/32767.0; //Scale to the correct unit
    gyroy = GYRO_Y_RAW*250.0/32767.0; //Scale to the correct unit
    gyroz = GYRO_Z_RAW*250.0/32767.0; //Scale to the correct unit

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

    gAngleLeft = readEncLeft();
    gAngleRight = readEncRight();

    if ((add % 200) == 0) {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1;
    }

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    numTimer0calls++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;

    gDistanceLeft = gAngleLeft*(1/5.06);
    gDistanceRight = gAngleRight*(1/5.06);

    PosLeft_K = gDistanceLeft;
    VLeftK = (PosLeft_K - PosLeft_K_1)/0.004;

    PosRight_K = gDistanceRight;
    VRightK = (PosRight_K - PosRight_K_1)/0.004;

    Eturn = turn + (VLeftK - VRightK);

    EK_L = Vref - VLeftK - KPturn*Eturn;
    IK_L = IK_1_L + 0.004*(EK_L+EK_1_L)/2.0;
    uLeft = KP_L*EK_L + Ki_L*IK_L;

    EK_R = Vref - VRightK + KPturn*Eturn;
    IK_R = IK_1_R + 0.004*(EK_R+EK_1_R)/2.0;
    uRight = KP_R*EK_R + Ki_R*IK_R;

    if(fabs(uRight)>10){
        IK_R = IK_1_R;
    }
    if(fabs(uLeft)>10){
        IK_L = IK_1_L;
    }

    PosRight_K_1 = PosRight_K;
    PosLeft_K_1 = PosLeft_K;
    EK_1_L = EK_L;
    EK_1_R = EK_R;
    IK_1_L = IK_L;
    IK_1_R = IK_R;

}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    CpuTimer2.InterruptCount++;
}


void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3.
    //Make sure the TXdelay in between each transfer to 0. Also dont forget to cut
    //and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset(???)
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master (???)
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission (???)
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt (???)
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // (???) Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLKs period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// (???) Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // (???) Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // (???) Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0; // (???) Set delay between transmits to 0 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // (???) Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // (???) Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I dont think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 10; //(???) Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below


    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    //-----------------------------------------------------------------------------------------------------------------
    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for
    //all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    SpibRegs.SPITXBUF = 0x1300;// To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x0000;// Auto address 00x14 and 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000;// Auto address 00x16 and 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0013;// Auto address 00x18 and 00x19 write 0x00 and write 0x13
    SpibRegs.SPITXBUF = 0x0200;// Auto address 00x1A and 00x1B write 0x02 and write 0x00
    SpibRegs.SPITXBUF = 0x0806;// Auto address 00x1C and 00x1D write 0x08 and write 0x06
    SpibRegs.SPITXBUF = 0x0000;// Auto address 00x1E and 00x1F write 0x00 and write 0x00
    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPITXBUF = 0x2300;// To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x408C;// Auto address 00x24 and 00x25 write 0x40 and 0x8C
    SpibRegs.SPITXBUF = 0x0288;// Auto address 00x26 and 00x27 write 0x02 and 0x88
    SpibRegs.SPITXBUF = 0x0C0A;// Auto address 00x28 and 00x29 write 0x0C and 0x0A


    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 1;
    SpibRegs.SPITXBUF = 0x2A81;// Write to address 0x2A the value 0x81
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0017); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0064); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00EE); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0062); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0016); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0076); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}


//!!!!!!!!!!!!!!!!!!!!!!  Copy these two function to the end of your C file
interrupt void DMA_ISR(void)                    // PIE7.1 @ 0x000DA0  DMA channel 1 interrupt
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;         // Must acknowledge the PIE group

    GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;

    //--- Process the ADC data
    if(iPingPong == 0)  // Ping buffer filling, process Pong bugger
    {
        // Manage the DMA registers
        EALLOW;                                                     // Enable EALLOW protected register access
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPongBufRaw;      // Adjust DST start address for Pong buffer
        EDIS;                                                       // Disable EALLOW protected register access

        // Don't run ping FFT first time around
        if (DMAcount > 0) {
            pingFFT = 1;
        }
        iPingPong = 1;
    }
    else    // Pong buffer filling, process Ping buffer
    {
        // Manage the DMA registers
        EALLOW;                                                     // Enable EALLOW protected register access
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPingBufRaw;                    // Adjust DST start address for Ping buffer
        EDIS;                                                       // Disable EALLOW protected register access

        pongFFT = 1;
        iPingPong = 0;
    }

    DMAcount += 1;

}

void InitDma(void)
{
    EALLOW;

    //---------------------------------------------------------------------
    //--- Overall DMA setup
    //---------------------------------------------------------------------
    DmaRegs.DMACTRL.bit.HARDRESET = 1;          // Reset entire DMA module
    asm(" NOP");                                // 1 cycle delay for HARDRESET to take effect

    DmaRegs.DEBUGCTRL.bit.FREE = 1;             // 1 = DMA unaffected by emulation halt
    DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 0;  // Not using CH1 Priority mode

    //---------------------------------------------------------------------
    //--- Configure DMA channel 1 to read the ADC results
    //---------------------------------------------------------------------
    DmaRegs.CH1.MODE.all = 0x8901; //good
    // bit 15        1:      CHINTE, 0=interrupt disabled, 1=interrupt enabled
    // bit 14        0:      DATASIZE, 0=16-bit, 1=32-bit
    // bit 13-12     00:     reserved
    // bit 11        1:      CONTINUOUS, 0=stop, 1=re-init after transfer complete
    // bit 10        0:      ONESHOT, 0=one burst on trigger, 1=all bursts on trigger
    // bit 9         0:      CHINTMODE, 0=start of transfer, 1=end of transfer
    // bit 8         1:      PERINTE, peripheral interrupt trigger enable, 0=disabled, 1=enabled
    // bit 7         0:      OVRINTE, overflow interrupt enable, 0=disabled, 1=enabled
    // bit 6-5       00:     reserved
    // bit 4-0       00001:  Set to channel number

    //--- Select DMA interrupt source                 /******** TRIGGER SOURCE FOR EACH DMA CHANNEL (unlisted numbers are reserved) ********/
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = 6;    // 0=none       6=ADCBINT1  12=ADCCINT2  18=ADCDINT3  32=XINT4      40=EPWM3SOCA  46=EPWM6SOCA  52=EPWM9SOCA   58=EPWM12SOCA  72=MREVTA    98=SD1FLT4    110=SPIRXDMAA     132=USBA_EPx_TX1
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = 0;    // 1=ADCAINT1   7=ADCBINT2  13=ADCCINT3  19=ADCDINT4  33=XINT5      41=EPWM3SOCB  47=EPWM6SOCB  53=EPWM9SOCB   59=EPWM12SOCB  73=MXEVTB    99=SD2FLT1    111=SPITXDMAB     133=USBA_EPx_RX2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = 0;    // 2=ADCAINT2   8=ADCBINT3  14=ADCCINT4  20=ADCDEVT   36=EPWM1SOCA  42=EPWM4SOCA  48=EPWM7SOCA  54=EPWM10SOCA  68=TINT0       74=MREVTB   100=SD2FLT2    112=SPIRXDMAB     134=USBA_EPx_TX2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH4 = 0;    // 3=ADCAINT3   9=ADCBINT4  15=ADCCEVT   29=XINT1     37=EPWM1SOCB  43=EPWM4SOCB  49=EPWM7SOCB  55=EPWM10SOCB  69=TINT1       95=SD1FLT1  101=SD2FLT3    113=SPITXDMAC     135=USBA_EPx_RX3
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH5 = 0;    // 4=ADCAINT4  10=ADCBEVT   16=ADCDINT1  30=XINT2     38=EPWM2SOCA  44=EPWM5SOCA  50=EPWM8SOCA  56=EPWM11SOCA  70=TINT2       96=SD1FLT2  102=SD2FLT4    114=SPIRXDMAC     136=USBA_EPx_TX3
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH6 = 0;    // 5=ADCAEVT   11=ADCCINT1  17=ADCDINT2  31=XINT3     39=EPWM2SOCB  45=EPWM5SOCB  51=EPWM8SOCB  57=EPWM11SOCB  71=MXEVTA      97=SD1FLT3  109=SPITXDMAA  131=USBA_EPx_RX1

    //--- DMA trigger source lock
    DmaClaSrcSelRegs.DMACHSRCSELLOCK.bit.DMACHSRCSEL1 = 0;              // Write a 1 to lock (cannot be cleared once set)
    DmaClaSrcSelRegs.DMACHSRCSELLOCK.bit.DMACHSRCSEL2 = 0;              // Write a 1 to lock (cannot be cleared once set)


    DmaRegs.CH1.BURST_SIZE.bit.BURSTSIZE = 0;                           // 0 means 1 word per burst
    DmaRegs.CH1.TRANSFER_SIZE = RFFT_SIZE-1;                          // RFFT_SIZE bursts per transfer

    DmaRegs.CH1.SRC_TRANSFER_STEP = 0;                                  // 0 means add 0 to pointer each burst in a transfer
    DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32)&AdcbResultRegs.ADCRESULT0;   // SRC start address

    DmaRegs.CH1.DST_TRANSFER_STEP = 1;                                  // 1 = add 1 to pointer each burst in a transfer
    DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPingBufRaw;                    // DST start address Ping buffer


    DmaRegs.CH1.CONTROL.all = 0x0091; //good
    // bit 15        0:      reserved
    // bit 14        0:      OVRFLG, overflow flag, read-only
    // bit 13        0:      RUNSTS, run status, read-only
    // bit 12        0;      BURSTSTS, burst status, read-only
    // bit 11        0:      TRANSFERSTS, transfer status, read-only
    // bit 10-9      00:     reserved
    // bit 8         0:      PERINTFLG, read-only
    // bit 7         1:      ERRCLR, error clear, 0=no action, 1=clear SYNCERR bit
    // bit 6-5       00:     reserved
    // bit 4         1:      PERINTCLR, periph event clear, 0=no action, 1=clear periph event
    // bit 3         0:      PERINTFRC, periph event force, 0=no action, 1=force periph event
    // bit 2         0:      SOFTRESET, 0=no action, 1=soft reset the channel
    // bit 1         0:      HALT, 0=no action, 1=halt the channel
    // bit 0         1:      RUN, 0=no action, 1=enable the channel

    //--- Finish up
    EDIS;

    //--- Enable the DMA interrupt
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;  // Enable DINTCH1 in PIE group 7
    IER |= M_INT7;                      // Enable INT7 in IER to enable PIE group


} // end InitDma()
//!!!!!!!!!!!!!!!!!!!!!!  End of Bloc
