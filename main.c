#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Timer_A2_PWM.h"
#include "inc/Bumper_Switches.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/LPF.h"
#include "inc/Analog_Distance_Sensors.h"
#include "inc/Nokia5110_LCD.h"

// Initialize a global variable for SysTick to keep track of elapsed time in milliseconds
uint32_t SysTick_ms_elapsed = 0;

// Global flag that gets set in Bumper_Switches_Handler.
// This is used to detect if any object occurred when any one of the bumper switches are pressed.
uint16_t Object_detected = 0;
uint32_t Object_detected_count = 0;

#define CONTROLLER_1    1
//#define DEBUG_ACTIVE    2


// Initialize constant distance values (in mm)
#define TOO_CLOSE_DISTANCE  130
#define TOO_FAR_DISTANCE    400
#define DESIRED_DISTANCE    175

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL         2500
#define PWM_SWING           1000
#define PWM_MIN             (PWM_NOMINAL - PWM_SWING)
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

// Declare global variables used to store filtered distance values from the Analog Distance Sensor
uint32_t Filtered_Distance_Left;
uint32_t Filtered_Distance_Center;
uint32_t Filtered_Distance_Right;

// Declare global variables used to store converted distance values from the Analog Distance Sensor
int32_t Converted_Distance_Left;
int32_t Converted_Distance_Center;
int32_t Converted_Distance_Right;

// Declare global variable used to store the amount of error
int32_t Error;
int16_t Objective_detect;

// Proportional Controller Gain
int32_t Kp = 4;

// Initialize set point to 250 mm
int32_t Set_Point = 250;

// Declare global variables used to update PWM duty cycle values for the motors
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;

void Sample_Analog_Distance_Sensor()
{
    // Declare local variables for the Sharp GP2Y0A21YK0F Analog Distance Sensors
    // before passing their addresses
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    // Start conversion of Analog Distance Sensor raw values
    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);

    // Apply low-pass filter to raw values
    Filtered_Distance_Right = LPF_Calc(Raw_A17);
    Filtered_Distance_Center = LPF_Calc2(Raw_A14);
    Filtered_Distance_Left = LPF_Calc3(Raw_A16);

    // Convert filtered distance values using the calibration formula
    Converted_Distance_Left = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Left);
    Converted_Distance_Center = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Center);
    Converted_Distance_Right = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Right);
}

void Controller_1()
{

    // Check if both the left and right distance sensor readings are greater than the desired distance
    if ((Converted_Distance_Left > DESIRED_DISTANCE) && (Converted_Distance_Right > DESIRED_DISTANCE))
    {
        // Calculate the set point as the average of the left and right sensor distance readings
        Set_Point = (Converted_Distance_Left + Converted_Distance_Right) / 2;
    }
    else
    {
        // If at least one distance sensor reading is below the desired distance, assign the set point to the desired distance
        Set_Point = DESIRED_DISTANCE;
    }

    // Calculate the error based on the sensor readings
    if (Converted_Distance_Left < Converted_Distance_Right)
    {
        Error = Converted_Distance_Left - Set_Point;
    }
    else
    {
        Error = Set_Point - Converted_Distance_Right;
    }

    // Calculate the new duty cycle for the right motor based on the error and proportional constant (Kp)
    Duty_Cycle_Right = PWM_NOMINAL + (Kp * Error);

    // Calculate the new duty cycle for the left motor based on the error and proportional constant (Kp)
    Duty_Cycle_Left  = PWM_NOMINAL - (Kp * Error);

    // Ensure that the duty cycle for the right motor does not go below the minimum PWM value
    if (Duty_Cycle_Right < PWM_MIN) Duty_Cycle_Right = PWM_MIN;

    // Ensure that the duty cycle for the right motor does not exceed the maximum PWM value
    if (Duty_Cycle_Right > PWM_MAX) Duty_Cycle_Right = PWM_MAX;

    // Ensure that the duty cycle for the left motor does not go below the minimum PWM value
    if (Duty_Cycle_Left  < PWM_MIN) Duty_Cycle_Left  = PWM_MIN;

    // Ensure that the duty cycle for the left motor does not exceed the maximum PWM value
    if (Duty_Cycle_Left  > PWM_MAX) Duty_Cycle_Left  = PWM_MAX;

#ifndef DEBUG_ACTIVE
    if(Object_detected == 0){
        if(Converted_Distance_Center <= TOO_CLOSE_DISTANCE | Converted_Distance_Right <= TOO_CLOSE_DISTANCE | Converted_Distance_Left <= TOO_CLOSE_DISTANCE  ){
            Motor_Backward(Duty_Cycle_Left,Duty_Cycle_Right);
            Clock_Delay1ms(1000);
            Motor_Right(1500, 1500);
            Clock_Delay1ms(4000);

        } else {
            Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
    }
    }


#endif
}

//#ifndef DEBUG_ACTIVE
//    // Apply the updated PWM duty cycle values to the motors
//
//    if((Converted_Distance_Left < TOO_CLOSE_DISTANCE) || (Converted_Distance_Right < TOO_CLOSE_DISTANCE))
//    {
//
//       Motor_Stop();
//   }
//    else if ((Converted_Distance_Left < 300) ^ (Converted_Distance_Right < 300) ^ (Converted_Distance_Center < 300))
//   {
//   Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
//}
//    else {
//        Motor_Stop();
//    }
//
//#endif

void Bumper_Switches_Handler(uint8_t bumper_switch_state)
{
    if(Object_detected == 0){
        Object_detected = 1;
    }
}
void SysTick_Handler(void)
{
#if defined CONTROLLER_1

    Controller_1();

#else
    #error "Define  one of the options: CONTROLLER_1."
#endif

    SysTick_ms_elapsed++;

    if (Object_detected == 0)
    {
        if (SysTick_ms_elapsed >= 500)
        {
            P8->OUT &= ~0xC0;
            P8->OUT ^= 0x21;
            SysTick_ms_elapsed = 0;
        }
    }
    else
    {
        P8->OUT |= 0xC0;
        P8->OUT &= ~0x21;
    }
}

void Timer_A1_Periodic_Task(void)
{
    Sample_Analog_Distance_Sensor();
}

void Handle_Object_Dection()
{
    Motor_Stop();
    Clock_Delay1ms(1000);
    Motor_Backward(2500, 2500);
    Clock_Delay1ms(2000);
    Motor_Stop();
    Clock_Delay1ms(1000);
    Motor_Right(1500, 1500);
    Clock_Delay1ms(2000);
    Motor_Stop();
}

int main(void)
{

    // Declare local variables for the Sharp GP2Y0A21YK0F Analog Distance Sensors
    // before passing their addresses
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
    LED1_Init();
    LED2_Init();

    // Initialize the buttons on the MSP432 microcontroller
    Buttons_Init();

    // Initialize the EUSCI_A0_UART module
    //EUSCI_A0_UART_Init_Printf();

    // Initialize the Nokia5110 LCD
    Nokia5110_Init();

    // Clear the Nokia5110 buffer and the LCD
    Nokia5110_ClearBuffer();
    Nokia5110_Clear();
    //Nokia5110_Set_Contrast(0xDf);

    Nokia5110_SetCursor(0, 1);
    Nokia5110_OutString("ObjectDetect");

    Nokia5110_SetCursor(0, 3);
    Nokia5110_OutUDec(Object_detected_count);

    // Initialize the EUSCI_A0_UART module
    EUSCI_A0_UART_Init_Printf();
    // Initialize the DC motors
    Motor_Init();
    // Initialize motor duty cycle values
    Duty_Cycle_Left  = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;
    // Initialize the Analog Distance Sensor using the ADC14 module
    Analog_Distance_Sensor_Init();

    // Start conversion of Analog Distance Sensor raw values
    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);

    // Initialize low-pass filters for the Analog Distance Sensor
    LPF_Init(Raw_A17, 64);
    LPF_Init2(Raw_A14, 64);
    LPF_Init3(Raw_A16, 64);

    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 with interrupts enabled and an interrupt rate of 2 kHz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);
    // Initialize Timer A2 with a period of 50 Hz
    // Timer A2 will be used to drive two servos
    Timer_A2_PWM_Init(TIMER_A2_PERIOD_CONSTANT, 0, 0);

    Bumper_Switches_Init(&Bumper_Switches_Handler);

     //Enable the interrupts used by the modules
    EnableInterrupts();

    while(1)
    {
        if (Object_detected == 1)
        {
            Object_detected_count = Object_detected_count + 1;
            Nokia5110_SetCursor(0, 3);
            Nokia5110_OutUDec(Object_detected_count);
            Handle_Object_Dection();
            Object_detected = 0;
        }
    }

    }



