//#############################################################################
// FILE:   fftDMAstarter_main.c
//
// TITLE:  FFT DMA Starter
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
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


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

// Global Varible
float Gpw = 0; // Store the Power of the sound
float Gfre = 0; // Store the frequency of the sound
uint32_t SN = 0; // State number use for case
uint32_t RSN; // Robot state number

float PSMC1C = 0; // Program state machine case 1 counter
float PSMC2C = 0; // Program state machine case 2 counter
float PSMC3C = 0; // Program state machine case 3 counter
float PSMC4C = 0; // Program state machine case 4 counter
float C1S = 0; // Case 1 start index
float C2S = 0; // Case 2 start index
float C3S = 0; // Case 3 start index
float C4S = 0; // Case 4 start index

// Case 1 Global Variable

// Case 1 Global Variable End

// Case 2 Global Variable

// Case 2 Global Variable End

// Case 3 Global Variable
float FwdCount = 0;
float TurnCount = 0;
// Case 3 Global Variable End

// Case 4 Global Variable

// Case 4 Global Variable End

// Case 5 Global Variable
float BUC1 = 1500;
float BUC2 = 1500;
// Case 5 Global Variable End

// Test Value
float tv = 0;

// Copy from Lab 6
float LW = 0; // Predefine value use for lab 6 exercise 1
float RW = 0; // Predefine value use for lab 6 exercise 1
float LWF = 0; // Predefine value use for lab 6 exercise 1
float RWF = 0; // Predefine value use for lab 6 exercise 1
float uLeft = 5.0; // Predefine value use for lab 6 exercise 1
float uRight = 5.0; // Predefine value use for lab 6 exercise 1
float PLK = 0; // Predefine value use for lab 6 exercise 2
float PLK_1 = 0; // Predefine value use for lab 6 exercise 2
float VLK = 0; // Predefine value use for lab 6 exercise 2
float PRK = 0; // Predefine value use for lab 6 exercise 2
float PRK_1 = 0; // Predefine value use for lab 6 exercise 2
float VRK = 0; // Predefine value use for lab 6 exercise 2
float eKL = 0; // Predefine value use for lab 6 exercise 3
float eKR = 0; // Predefine value use for lab 6 exercise 3
float Vref = 0; // Predefine value use for lab 6 exercise 3
float vK = 0; // Predefine value use for lab 6 exercise 3
float IKL = 0; // Predefine value use for lab 6 exercise 3
float IKL1 = 0; // Predefine value use for lab 6 exercise 3
float eKL1 = 0; // Predefine value use for lab 6 exercise 3
float eKR1 = 0; // Predefine value use for lab 6 exercise 3
float IKR = 0; // Predefine value use for lab 6 exercise 3
float IKR1 = 0; // Predefine value use for lab 6 exercise
float uKL = 0; // Predefine value use for lab 6 exercise 3
float uKR = 0; // Predefine value use for lab 6 exercise 3
float Kp = 3; // Predefine value use for lab 6 exercise 3
float Ki = 25.0; // Predefine value use for lab 6 exercise 3
float KpT = 3; // Predefine value use for lab 6 exercise 4
float eT = 0; // Predefine value use for lab 6 exercise 4
float turn = 0;
// Copy from Lab 6 end

// Global function
void PSM(void); // Program state machine function
int freIndex(float fre); // Frequency input to index converter
void RBSM(void);// Robot state machine

// Copy from Lab 6
void setupSpib(void);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
// Copy from Lab 6 end

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);


void serialRXA(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LuanchPad
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

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //GPIO 52
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

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
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    //!!!!!!!!!!!!!!!!!!!!!!  Copy the Assignmnt of the DMA interrupt service routine
    PieVectTable.DMA_CH1_INT = &DMA_ISR;
    //!!!!!!!!!!!!!!!!!!!!!!  End of Block
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 2000000); // 1000 = 1ms, 1s
    ConfigCpuTimer(&CpuTimer1, 200, 2000000); // 1s
    ConfigCpuTimer(&CpuTimer2, 200, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
    //    init_serial(&SerialC,115200,serialRXC);
    //    init_serial(&SerialD,115200,serialRXD);

    // Copy from Lab 6
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Set TBCTL in Up count mode.
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 3; // Let the PWM go free run when I set a breakpoint.
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // disable the phase loading.
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Clock divide by 1.
    EPwm2Regs.TBCTR = 0; // Start the timer at zero.
    EPwm2Regs.TBPRD = 2500; // Set period to 20kHz, 9C4 = 2500.
    EPwm2Regs.CMPA.bit.CMPA = 0; // Start duty cycle 0%.
    EPwm2Regs.CMPB.bit.CMPB = 0; // Start duty cycle 0%.
    EPwm2Regs.AQCTLA.bit.CAU = 1; // Set TBCTR to clear when reach CMPA value. EPWM2A
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // Set TBCTR to set when TBCTR is zero. EPWM2A
    EPwm2Regs.AQCTLB.bit.CBU = 1; // Set TBCTR for to clear when reach CMPA value. EPWM2B
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // Set TBCTR to set when TBCTR is zero.EPWM2B
    EPwm2Regs.TBPHS.bit.TBPHS = 0; // Set phase to zero.

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); // Set GPIO2 to be the output pin for EPWM2A
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); // Set GPIO3 to be the output pin for EPWM2B
    // Copy from Lab 6 end

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

    // Copy from Lab 6
    setupSpib();
    init_eQEPs();
    // Copy from Lab 6 end

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Copy from Lab 6
    // Enable SPIB_RX in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    // Copy from Lab 6 end

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

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA, "Power: %.3f Frequency: %.0f SN: %ld TV: %.0f \r\n", maxpwr, maxpwrindex*10000.0/1024.0,SN,tv);
            Gpw = maxpwr;
            Gfre = maxpwrindex*10000.0/1024.0;

            UARTPrint = 0;
        }

        //********** Detect Frequency **********//

        //********** End Detect **********//

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

    // Insert SWI ISR Code here.......

    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    SN = (SN * 10 + freIndex(Gfre)) % 100000;

    numTimer0calls++;

    // if ((numTimer0calls%50) == 0) {
    //     PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI  
    // }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    CpuTimer1.InterruptCount++;
    if (SN != 82112|| SN != 81128 || SN != 84283 || SN != 81188 || SN != 83388){
        PSM();
    }

}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    // Blink a number of LEDS
    CpuTimer2.InterruptCount++;
    //  if ((CpuTimer2.InterruptCount % 50) == 0) {
    //      UARTPrint = 1;
    //  }

    // Copy from Lab 6
    LW = readEncLeft(); // in radiant
    RW = readEncRight(); // in radiant
    PLK = LW/4.95;
    VLK = (PLK - PLK_1)/0.004;
    PLK_1 = PLK;
    PRK = RW/4.95;
    VRK = (PRK - PRK_1)/0.004;
    PRK_1 = PRK;
    eT = turn +(VLK - VRK);
    eKL = Vref - VLK - KpT * eT;
    eKR = Vref - VRK + KpT * eT;
    IKL = IKL1 + 0.004 * (eKL+eKL1)/2;
    uKL = Kp * eKL + Ki * IKL;
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
    // Copy from Lab 6 end

    if (GpioDataRegs.GPADAT.bit.GPIO4 == 1 && GpioDataRegs.GPADAT.bit.GPIO6 == 1 && (BUC1 >= 1500 || BUC2 >= 1500)){
        // Program state Machine Counter
        //else{
        // Sound Track 1 Movement
        if (C1S == 1 && PSMC1C < 10000){
            PSMC1C++; //Increase program state machine case 1 counter
            RBSM();
        }
        else if (PSMC1C >= 10000){
            PSMC1C = 0;  // Reset program state machine case 1 counter
            C1S = 0;
        }
        // Sound Track 1 Movement End

        // Sound Track 2 Movement
        if (C2S == 1 && PSMC2C < 10000){ // 10000 is the timer
            PSMC2C++; // Program state machine case 2 counter
        }
        // Sound Track 2 Movement End

        // Sound Track 3 Movement
        if (C3S == 1 && PSMC3C < 14000){
            PSMC3C++;
            RBSM();
        }
        else if(PSMC3C >= 14000){
            PSMC3C = 0;
            C3S = 0;
        }
        // Sound Track 3 Movement End

        // Sound Track 4 Movement
        if (C4S == 1 && PSMC4C < 3000){ // 10000 is the timer
            PSMC4C++; // Program state machine case 4 counter
            RBSM();
        }
        else if (PSMC4C >= 3000){
            PSMC4C = 0;
            C4S = 0;
        }

        // Sound Track 4 Movement End
        //}

    }
    else{
        if((GpioDataRegs.GPADAT.bit.GPIO4) == 0){
            BUC1 = 0;
        }
        if (BUC1 < 1500){
            Vref = -0.15;
            turn = -0.05;
            BUC1++;
        }


        if ((GpioDataRegs.GPADAT.bit.GPIO6) == 0){
            BUC2 = 0;
        }
        if (BUC2 < 1500){
            Vref = -0.15;
            turn = 0.05;
            BUC2++;
        }
    }
}
// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

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
//!!!!!!!!!!!!!!!!!!!!!!  End of Block

// PSM() Function Start

void PSM(void){ // Project State Machine

    switch(SN){

    // Case 1
    case 84283: // Circle movement soundtrack1 76573
        if (C1S == 0 && PSMC1C < 10000){
            RSN = 4; // Robot State Machine
            C1S = 1;
            tv = 1;
        }
        break;

        // Case 2
    case 82132: // Wheelie ST2
        if (C2S == 0 && PSMC2C < 10000){
            // Wheelie Code
            C2S = 1;
            tv = 2;
        }
        break;

        // Case 3
    case 81188: // Square Movement soundtrack3 73377
        if (C3S == 0 && PSMC3C < 14000){
            RSN = 0;
            C3S = 1;
            tv = 3;
        }
        break;
    case 83388: // Square Movement soundtrack3 73377
        if (C3S == 0 && PSMC3C < 14000){
            RSN = 0;
            C3S = 1;
            tv = 3;
        }
        break;
        // Case 4
    case 81128: // 8 movement
        if (C4S == 0 && PSMC4C < 5000){
            RSN = 5;
            C4S = 1;
            tv = 4;
        }
        break;

        // Default Case
    default:// Wall following
        tv = 5;
        turn = 0;
        Vref = 0;
        break;

    }
}

// PSM() Function End

void RBSM(void){

    switch(RSN){

    // For PSM() Case 3

    case 0: // Forward
        turn = 0;
        Vref = 0;
        FwdCount = 0;
        RSN = 2;
        break;

    case 1: // Turn Left
        turn = 0;
        Vref = 0;
        TurnCount = 0;
        RSN = 3;
        break;

    case 2: // Forward Counter
        FwdCount++;
        // Fwd command
        Vref = 0.15;
        // Fwd command end
        if (FwdCount > 3000){ // Change to turn command after move about 3s
            RSN = 1;
        }
        else{ // Stay in Fwd command
            RSN = 2;
        }
        break;

    case 3: //Left Turn Counter
        TurnCount++;
        // Left Turn Command
        turn = 0.5;
        // Left Turn Command end
        if (TurnCount > 500){ // Change to Fwd command after turn about 0.5s
            RSN = 0;
        }
        else{ // Stay in turn command
            RSN = 3;
        }
        break;

        // For PSM() Case 3 End

        // For PSM() Case 1

    case 4: // Circle Movement
        turn = 0.15;
        Vref = 0.2;
        break;

    case 5: // Back up
        // Fwd command
        Vref = -0.15;
        turn = 0;
    }

    // For PSM() Case 1 End
}

// Frequency to State Number Converter
int freIndex(float fre){

    int SNR = 0;

    if (fre < 1000){
        SNR = 1;
    }

    else if (fre > 1000 && fre < 1500){
        SNR = 2;
    }

    else if (fre > 1500 && fre < 2000){
        SNR = 3;
    }

    else if (fre > 2000 && fre < 2500){
        SNR = 4;
    }

    else if (fre > 2500 && fre < 3000){
        SNR = 5;
    }

    else if (fre > 3000 && fre < 3500){
        SNR = 6;
    }

    else if (fre > 3500 && fre < 4000){
        SNR = 7;
    }

    else if (fre > 4000 && fre < 4500){
        SNR = 8;
    }

    else if (fre > 4500){
        SNR = 9;
    }

    return SNR;

}

// Copy from Lab 6
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    // Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    // between each transfer to 0. Also dont forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    // 66 which are also a part of the SPIB setup.

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
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x31; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLKs period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL

    SpibRegs.SPIFFCT.bit.TXDLY = 0x10; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip

    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I dont think this is needed. Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more received into FIFO causes
    //
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    //-----------------------------------------------------------------------------------------------------------------

    // Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    // sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x1300; // select register 19 and write 00
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000; // write 00 to register 20, write 00 to register 21
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0000; // write 00 to register 22, write 00 to register 23
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0013; // write 00 to register 24, write 13 to register 25
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0200; // write 02 to register 26, write 00 to register 27
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0806; // write 08 to register 28, write 06 to register 29
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x0000; // write 00 to register 30, write 00 to register 31

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7); // 7 parts need to received in to RX FIFO
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    // clear the flags (read the Rx buffer)
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x2300; // select register 35 and write 00
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C; // write 40 to register 36, write 8C to register 37
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288; // write 02 to register 38, write 88 to register 39
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A; // write 0C to register 40, write 0A to register 41

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4); // 4 parts need to received in to RX FIFO
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    // clear the flags (read the Rx buffer)
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81; // Exercise 4
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
    SpibRegs.SPITXBUF = (0x7700 | 0x0016); // 0x7700 use the correct offset to make X accleration close to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x00D0); // 0x7800  use the correct offset to make X accleration close to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E9); // 0x7A00  use the correct offset to make Y accleration close to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x000F); // 0x7B00  use the correct offset to make Y accleration close to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001C); // 0x7D00  use the correct offset to make Z accleration close to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00  use the correct offset to make Z accleration close to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
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

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(-2*PI/12000));
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2*PI/12000));
}

// Copy from lab 3
void setEPWM2A(float controleffort){ // Function that can control the left wheel of the small car in exercise 2
    if (controleffort >= 10){ // when controleffort larger than or equal to 10 controleffort = 10
        controleffort = 10;
    }
    if (controleffort <= -10){ // when controleffort less than or equal to -10 controleffort = -10
        controleffort = -10;
    }

    EPwm2Regs.CMPA.bit.CMPA = (EPwm2Regs.TBPRD/2) + controleffort * (EPwm2Regs.TBPRD/20); // control the duty cycle of the motor start from 0 as 50% duty cycle.

}

void setEPWM2B(float controleffort){ // Function that can control the right wheel of the small car in exercise 2
    if (controleffort >= 10){ // when controleffort larger than or equal to 10 controleffort = 10
        controleffort = 10;
    }
    if (controleffort <= -10){ // when controleffort less than or equal to -10 controleffort = -10
        controleffort = -10;
    }

    EPwm2Regs.CMPB.bit.CMPB = (EPwm2Regs.TBPRD/2) + controleffort * (EPwm2Regs.TBPRD/20); // control the duty cycle of the motor start from 0 as 50% duty cycle.

}
// Copy from Lab 6 end
