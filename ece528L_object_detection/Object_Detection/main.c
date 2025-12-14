/**
 * @file main.c
 * @brief Self-Sustaining Environmental Patrol Robot
 *
 * Board:  MSP432P401R LaunchPad
 *
 * Features:
 *  - Reads temperature and humidity from DHT22 sensor on P1.7
 *  - Irrigation state machine drives PWM LED (P2.4) and prints to UART
 *  - Uses three Sharp GP2Y0A21YK0F distance sensors for wall-following
 *  - Wall-follower controller runs at 100 Hz (SysTick)
 *  - Distance sensors sampled at 2 kHz (Timer A1)
 *
 * @author Juan Zendejas
 *
 */

#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/LPF.h"
#include "inc/Analog_Distance_Sensors.h"


#define CONTROLLER_1    1
// #define CONTROLLER_2  1
// #define CONTROLLER_3  1

//#define DEBUG_ACTIVE   1

// Distance thresholds (mm)
#define TOO_CLOSE_DISTANCE  200
#define TOO_FAR_DISTANCE    400
#define DESIRED_DISTANCE    250

// Motor PWM constants (Timer A0 from lab)
#define PWM_NOMINAL         2500
#define PWM_SWING           1000
#define PWM_MIN             (PWM_NOMINAL - PWM_SWING)
#define PWM_MAX             (PWM_NOMINAL + PWM_SWING)

// Filtered ADC values
uint32_t Filtered_Distance_Left;
uint32_t Filtered_Distance_Center;
uint32_t Filtered_Distance_Right;

// Calibrated distances (mm)
int32_t Converted_Distance_Left;
int32_t Converted_Distance_Center;
int32_t Converted_Distance_Right;

// Controller variables
int32_t Error;
int32_t Kp = 4;
int32_t Set_Point = 250;
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;


/**
 * @brief Sample three analog distance sensors and update global distances.
 */
void Sample_Analog_Distance_Sensor(void)
{
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);

    // Low-pass filter
    Filtered_Distance_Right  = LPF_Calc(Raw_A17);
    Filtered_Distance_Center = LPF_Calc2(Raw_A14);
    Filtered_Distance_Left   = LPF_Calc3(Raw_A16);

    // Calibrate to distance in mm
    Converted_Distance_Left   = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Left);
    Converted_Distance_Center = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Center);
    Converted_Distance_Right  = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Right);
}

/**
 * @brief Lab 6 Controller_1 — wall follower.
 */
void Controller_1(void)
{
    // Same logic as lab: use left/right distances to follow wall

    if ((Converted_Distance_Left > DESIRED_DISTANCE) &&
        (Converted_Distance_Right > DESIRED_DISTANCE))
    {
        Set_Point = (Converted_Distance_Left + Converted_Distance_Right) / 2;
    }
    else
    {
        Set_Point = DESIRED_DISTANCE;
    }

    if (Converted_Distance_Left < Converted_Distance_Right)
    {
        Error = Converted_Distance_Left - Set_Point;
    }
    else
    {
        Error = Set_Point - Converted_Distance_Right;
    }

    Duty_Cycle_Right = PWM_NOMINAL + (Kp * Error);
    Duty_Cycle_Left  = PWM_NOMINAL - (Kp * Error);

    if (Duty_Cycle_Right < PWM_MIN) Duty_Cycle_Right = PWM_MIN;
    if (Duty_Cycle_Right > PWM_MAX) Duty_Cycle_Right = PWM_MAX;
    if (Duty_Cycle_Left  < PWM_MIN) Duty_Cycle_Left  = PWM_MIN;
    if (Duty_Cycle_Left  > PWM_MAX) Duty_Cycle_Left  = PWM_MAX;

#ifndef DEBUG_ACTIVE
    Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
#endif
}

// DHT22 on P1.7
#define DHT22_BIT          BIT7       // P1.7

// LED PWM on P2.4 (TA0.1)
#define LED_PWM_BIT        BIT4       // P2.4
#define PWM_PERIOD_TICKS   48000      // ~250 Hz at 12 MHz SMCLK

// Irrigation state machine
typedef enum {
    IRR_STATE_STARTUP = 0,
    IRR_STATE_OK,
    IRR_STATE_DRY,
    IRR_STATE_VERY_DRY
} IrrigationState;

#define HUMIDITY_VERY_DRY  30.0f
#define HUMIDITY_DRY       50.0f

static IrrigationState current_state = IRR_STATE_STARTUP;

// DHT22 raw bytes
static uint8_t humidity_int, humidity_dec, temp_int, temp_dec, checksum;

// Simple delay wrappers
static void Delay_us(uint32_t us)
{
    Clock_Delay1us(us);
}

static void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Clock_Delay1us(1000);
    }
}

// DHT22 GPIO helpers
static void DHT22_Pin_Output(void)
{
    P1->DIR |= DHT22_BIT;
}

static void DHT22_Pin_Input(void)
{
    P1->DIR &= ~DHT22_BIT;
}

static void DHT22_Pin_Low(void)
{
    P1->OUT &= ~DHT22_BIT;
}

static void DHT22_Pin_High(void)
{
    P1->OUT |= DHT22_BIT;
}

// Initialize GPIO for DHT22 on P1.7
void DHT22_Init(void)
{
    P1->SEL0 &= ~DHT22_BIT;
    P1->SEL1 &= ~DHT22_BIT;
    P1->REN  &= ~DHT22_BIT;
    DHT22_Pin_High();
    DHT22_Pin_Output();
}

// Start signal
void DHT22_Begin(void)
{
    DHT22_Pin_Output();
    DHT22_Pin_Low();
    Delay_ms(3);        // pull low for at least 1 ms
    DHT22_Pin_High();
    Delay_us(30);
    DHT22_Pin_Input();  // release line
}

uint8_t DHT22_ReadByte(void)
{
    uint8_t i, result = 0;

    for (i = 0; i < 8; i++)
    {
        uint32_t timeout = 10000;

        // wait for line to go high
        while (((P1->IN & DHT22_BIT) == 0) && timeout--)
            Delay_us(1);

        // wait ~35 us then sample
        Delay_us(35);

        result <<= 1;
        if (P1->IN & DHT22_BIT)
            result |= 1;

        // wait for line to go low again
        timeout = 10000;
        while ((P1->IN & DHT22_BIT) && timeout--)
            Delay_us(1);
    }

    return result;
}

// Read full frame from DHT22
int DHT22_Read(float *temp_c, float *humidity)
{
    uint16_t raw_humidity, raw_temp;
    uint8_t sum;

    DHT22_Begin();

    // Sync with sensor response
    uint32_t timeout = 10000;
    while ((P1->IN & DHT22_BIT) && timeout--) Delay_us(1);
    timeout = 10000;
    while (!(P1->IN & DHT22_BIT) && timeout--) Delay_us(1);
    timeout = 10000;
    while ((P1->IN & DHT22_BIT) && timeout--) Delay_us(1);

    humidity_int = DHT22_ReadByte();
    humidity_dec = DHT22_ReadByte();
    temp_int     = DHT22_ReadByte();
    temp_dec     = DHT22_ReadByte();
    checksum     = DHT22_ReadByte();

    sum = humidity_int + humidity_dec + temp_int + temp_dec;
    if (sum != checksum)
    {
        return -1;
    }

    raw_humidity = (humidity_int << 8) | humidity_dec;
    raw_temp     = (temp_int << 8) | temp_dec;

    *humidity = raw_humidity / 10.0f;

    if (raw_temp & 0x8000)
    {
        raw_temp &= 0x7FFF;
        *temp_c = -1.0f * (raw_temp / 10.0f);
    }
    else
    {
        *temp_c = raw_temp / 10.0f;
    }

    return 0;
}

void LED_PWM_Init(void)
{
    P2->DIR  |= LED_PWM_BIT;
    P2->SEL0 |= LED_PWM_BIT;
    P2->SEL1 &= ~LED_PWM_BIT;

    TIMER_A0->CTL  = TIMER_A_CTL_SSEL__SMCLK |
                     TIMER_A_CTL_MC__UP |
                     TIMER_A_CTL_CLR;
    TIMER_A0->CCR[0]  = PWM_PERIOD_TICKS - 1;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A0->CCR[1]  = 0;
}

void LED_PWM_SetDutyPercent(float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t duty = (uint32_t)((percent / 100.0f) * (float)PWM_PERIOD_TICKS);
    TIMER_A0->CCR[1] = duty;
}


void Irrigation_UpdateState(float humidity)
{
    if (humidity < HUMIDITY_VERY_DRY)
    {
        current_state = IRR_STATE_VERY_DRY;
    }
    else if (humidity < HUMIDITY_DRY)
    {
        current_state = IRR_STATE_DRY;
    }
    else
    {
        current_state = IRR_STATE_OK;
    }
}

void Irrigation_ApplyOutputs(float temp_c, float humidity)
{
    switch (current_state)
    {
        case IRR_STATE_VERY_DRY:
            LED_PWM_SetDutyPercent(100.0f);
            printf("[STATE: VERY DRY] Irrigation needed. ");
            break;

        case IRR_STATE_DRY:
            LED_PWM_SetDutyPercent(60.0f);
            printf("[STATE: DRY] Monitor and prepare to irrigate. ");
            break;

        case IRR_STATE_OK:
            LED_PWM_SetDutyPercent(15.0f);
            printf("[STATE: OK] Moisture level acceptable. ");
            break;

        case IRR_STATE_STARTUP:
        default:
            LED_PWM_SetDutyPercent(0.0f);
            printf("[STATE: STARTUP] Waiting for stable readings. ");
            break;
    }

    printf("Temp: %.1f C, Humidity: %.1f %%\r\n", temp_c, humidity);
}


/**
 * @brief SysTick interrupt at 100 Hz – runs controller.
 */
void SysTick_Handler(void)
{
#if defined CONTROLLER_1

    Controller_1();

#elif defined CONTROLLER_2
  #if defined CONTROLLER_1 || defined CONTROLLER_3
    #error "Only one controller can be active at a time."
  #endif
    Controller_2();

#elif defined CONTROLLER_3
  #if defined CONTROLLER_1 || defined CONTROLLER_2
    #error "Only one controller can be active at a time."
  #endif
    Controller_3();

#else
  #error "Define CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3."
#endif
}

/**
 * @brief Timer A1 periodic task at 2 kHz – samples distance sensors.
 */
void Timer_A1_Periodic_Task(void)
{
    Sample_Analog_Distance_Sensor();
}


int main(void)
{
    // Distance sensor init locals
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    // DHT22 / irrigation variables
    float temp_c, humidity;
    int status;

    // Stop watchdog
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // 48 MHz system clock
    Clock_Init48MHz();

    // Disable interrupts during setup
    DisableInterrupts();

    // UART for printf
    EUSCI_A0_UART_Init_Printf();

    // Motors and distance sensors (Lab 6 hardware)
    Motor_Init();
    Analog_Distance_Sensor_Init();

    // First ADC conversion for LPF initialization
    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);
    LPF_Init(Raw_A17, 64);
    LPF_Init2(Raw_A14, 64);
    LPF_Init3(Raw_A16, 64);

    // SysTick at 100 Hz
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Timer A1 at 2 kHz for distance sensor sampling
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // DHT22 + LED PWM
    DHT22_Init();
    LED_PWM_Init();

    current_state = IRR_STATE_STARTUP;

    // Enable global interrupts
    EnableInterrupts();

    printf("Environmental Patrol Robot + DHT22 Irrigation Monitor Started\r\n");

    while (1)
    {
        // Periodic DHT22 reading
        status = DHT22_Read(&temp_c, &humidity);

        if (status == 0)
        {
            Irrigation_UpdateState(humidity);
            Irrigation_ApplyOutputs(temp_c, humidity);
        }
        else
        {
            current_state = IRR_STATE_STARTUP;
            LED_PWM_SetDutyPercent(0.0f);
            printf("[STATE: SENSOR ERROR] DHT22 read failed\r\n");
        }

        // 2-second period between environmental samples
        Delay_ms(2000);
        // During this delay, SysTick and Timer A1 interrupts continue
        // running the wall-following controller and distance sampling.
    }
}
