/**
 * @file main.c
 * @brief Self-Sustaining Environmental Patrol Robot
 *
 * Wall-following robot using three Sharp IR distance sensors
 * + DHT22-based environmental monitor with UART and LED irrigation state machine.
 *
 * Peripherals:
 *  - 3x Sharp GP2Y0A21YK0F analog distance sensors (10–80 cm)
 *  - 2x DC motors driven by PWM (Motor.h support code)
 *  - DHT22 sensor on P1.7 (single-wire)
 *  - PWM LED on P2.4 (TA0.1) to show irrigation state
 *  - UART over USB to PC terminal for displaying sensor and state
 *
 *  Author Juan Zendejas
 */

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "msp.h"
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
//#define CONTROLLER_2    1
//#define CONTROLLER_3    1

//#define DEBUG_ACTIVE    1

//Distance control (Lab 6)

// Distance thresholds (mm)
#define TOO_CLOSE_DISTANCE   200
#define TOO_FAR_DISTANCE     400
#define DESIRED_DISTANCE     250

// PWM for motors
#define PWM_NOMINAL          2500
#define PWM_SWING            1000
#define PWM_MIN              (PWM_NOMINAL - PWM_SWING)
#define PWM_MAX              (PWM_NOMINAL + PWM_SWING)

// Extra safety: front turn and escape timing
#define FRONT_TURN_DISTANCE  300   // start turning when wall is closer than ~30 cm
#define ESCAPE_TURN_TICKS     35   // ~0.35 s of escape turn at 100 Hz

// Filtered ADC readings
uint32_t Filtered_Distance_Left;
uint32_t Filtered_Distance_Center;
uint32_t Filtered_Distance_Right;

// Converted distance values (mm)
int32_t Converted_Distance_Left;
int32_t Converted_Distance_Center;
int32_t Converted_Distance_Right;

// Control variables
int32_t Error;
int32_t Kp = 4;          // proportional gain
int32_t Set_Point = 250; // desired distance

// Motor PWM duty cycles
uint16_t Duty_Cycle_Left;
uint16_t Duty_Cycle_Right;


// DHT22 + LED irrigation monitor

// Pin and PWM definitions for DHT22 + LED
#define DHT22_BIT        BIT7       // P1.7
#define LED_PWM_BIT      BIT4       // P2.4 (TA0.1)
#define PWM_PERIOD_TICKS 48000      // with SMCLK=12MHz, ~250 Hz

// Irrigation state machine
typedef enum {
    IRR_STATE_STARTUP = 0,
    IRR_STATE_OK,
    IRR_STATE_DRY,
    IRR_STATE_VERY_DRY
} IrrigationState;

// Humidity thresholds (tuneable)
#define HUMIDITY_VERY_DRY  30.0f  // below this -> VERY_DRY
#define HUMIDITY_DRY       50.0f  // below this (but above VERY_DRY) -> DRY

static IrrigationState current_state = IRR_STATE_STARTUP;

// DHT22 raw frame bytes
static uint8_t humidity_int, humidity_dec, temp_int, temp_dec, checksum;

// Delay wrappers (use lab Clock driver)
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

// DHT22 helpers (GPIO + timing)
static void DHT22_Pin_Output(void)
{
    P1->DIR  |= DHT22_BIT;   // output
}

static void DHT22_Pin_Input(void)
{
    P1->DIR  &= ~DHT22_BIT;  // input
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

// Send start signal to DHT22
void DHT22_Begin(void)
{
    DHT22_Pin_Output();
    DHT22_Pin_Low();
    Delay_ms(3);          // pull low for at least 1 ms
    DHT22_Pin_High();
    Delay_us(30);
    DHT22_Pin_Input();    // release line, wait for sensor response
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

// Read full frame from DHT22, return 0 on success, -1 on checksum error
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
        return -1;  // checksum failed
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

// LED PWM on P2.4 (TA0.1)
void LED_PWM_Init(void)
{
    // Configure P2.4 as TA0.1 output
    P2->DIR  |= LED_PWM_BIT;
    P2->SEL0 |= LED_PWM_BIT;
    P2->SEL1 &= ~LED_PWM_BIT;

    TIMER_A0->CTL  = TIMER_A_CTL_SSEL__SMCLK |  // SMCLK
                     TIMER_A_CTL_MC__UP     |   // Up mode
                     TIMER_A_CTL_CLR;
    TIMER_A0->CCR[0]  = PWM_PERIOD_TICKS - 1;   // Period
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; // Reset/set
    TIMER_A0->CCR[1]  = 0;                      // Start at 0% duty
}

void LED_PWM_SetDutyPercent(float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t duty = (uint32_t)((percent / 100.0f) * (float)PWM_PERIOD_TICKS);
    TIMER_A0->CCR[1] = duty;
}

// Irrigation state machine
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
            // Very dry: LED bright, strong warning
            LED_PWM_SetDutyPercent(100.0f);
            printf("[STATE: VERY DRY] Irrigation needed. ");
            break;

        case IRR_STATE_DRY:
            // Dry: medium brightness, early warning
            LED_PWM_SetDutyPercent(60.0f);
            printf("[STATE: DRY] Monitor and prepare to irrigate. ");
            break;

        case IRR_STATE_OK:
            // OK: dim LED, acceptable moisture
            LED_PWM_SetDutyPercent(15.0f);
            printf("[STATE: OK] Moisture level acceptable. ");
            break;

        case IRR_STATE_STARTUP:
        default:
            // Startup or unknown: LED off, just print data
            LED_PWM_SetDutyPercent(0.0f);
            printf("[STATE: STARTUP] Waiting for stable readings. ");
            break;
    }

    // Common part of the message: sensor values
    printf("Temp: %.1f C, Humidity: %.1f %%\r\n", temp_c, humidity);
}


// Distance sensor sampling

/**
 * @brief Sample and filter the three Sharp IR distance sensors.
 */
void Sample_Analog_Distance_Sensor(void)
{
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    // Start conversion of Analog Distance Sensor raw values
    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);

    // Apply low-pass filter to raw values
    Filtered_Distance_Right  = LPF_Calc(Raw_A17);
    Filtered_Distance_Center = LPF_Calc2(Raw_A14);
    Filtered_Distance_Left   = LPF_Calc3(Raw_A16);

    // Convert filtered distance values using the calibration formula
    Converted_Distance_Left   = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Left);
    Converted_Distance_Center = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Center);
    Converted_Distance_Right  = Analog_Distance_Sensor_Calibrate(Filtered_Distance_Right);
}


// Controller_1 (with front safety)

/**
 * @brief Wall follower with front safety/escape turn.
 *
 * Normal behavior:
 *   - Original Lab 6 Controller_1 wall follower using LEFT/RIGHT sensors.
 *
 * Safety behavior:
 *   - If CENTER sees a wall closer than FRONT_TURN_DISTANCE, enter an
 *     "escape turn" state for ESCAPE_TURN_TICKS SysTick periods so we
 *     actually swing away from the wall instead of bumping into it.
 */
void Controller_1(void)
{
    static uint8_t escape_ticks = 0;    // counts remaining escape-turn ticks

    int32_t left   = Converted_Distance_Left;
    int32_t right  = Converted_Distance_Right;
    int32_t center = Converted_Distance_Center;

    // 0. ESCAPE TURN STATE: keep turning for a short time once triggered
    if (escape_ticks > 0)
    {
        escape_ticks--;

        // Turn toward the side with more free space
        if (left > right)
        {
            // More space on LEFT -> turn LEFT
            Duty_Cycle_Left  = PWM_NOMINAL / 4;   // slow left wheel
            Duty_Cycle_Right = PWM_NOMINAL;       // fast right wheel
        }
        else
        {
            // More space on RIGHT -> turn RIGHT
            Duty_Cycle_Left  = PWM_NOMINAL;       // fast left wheel
            Duty_Cycle_Right = PWM_NOMINAL / 4;   // slow right wheel
        }

        // Clamp PWM values
        if (Duty_Cycle_Left < PWM_MIN)   Duty_Cycle_Left  = PWM_MIN;
        if (Duty_Cycle_Left > PWM_MAX)   Duty_Cycle_Left  = PWM_MAX;
        if (Duty_Cycle_Right < PWM_MIN)  Duty_Cycle_Right = PWM_MIN;
        if (Duty_Cycle_Right > PWM_MAX)  Duty_Cycle_Right = PWM_MAX;

    #ifndef DEBUG_ACTIVE
        Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
    #endif
        return;   // skip normal wall-follow this tick
    }

    // 1. FRONT SAFETY TRIGGER: start escape turn if wall is in front
    if (center < FRONT_TURN_DISTANCE)
    {
        escape_ticks = ESCAPE_TURN_TICKS;   // enter escape-turn mode

        if (left > right)
        {
            // More space on LEFT -> turn LEFT
            Duty_Cycle_Left  = PWM_NOMINAL / 4;
            Duty_Cycle_Right = PWM_NOMINAL;
        }
        else
        {
            // More space on RIGHT -> turn RIGHT
            Duty_Cycle_Left  = PWM_NOMINAL;
            Duty_Cycle_Right = PWM_NOMINAL / 4;
        }

        // Clamp PWM values
        if (Duty_Cycle_Left < PWM_MIN)   Duty_Cycle_Left  = PWM_MIN;
        if (Duty_Cycle_Left > PWM_MAX)   Duty_Cycle_Left  = PWM_MAX;
        if (Duty_Cycle_Right < PWM_MIN)  Duty_Cycle_Right = PWM_MIN;
        if (Duty_Cycle_Right > PWM_MAX)  Duty_Cycle_Right = PWM_MAX;

    #ifndef DEBUG_ACTIVE
        Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
    #endif
        return;
    }


    // Check if both the left and right distance sensor readings are greater than the desired distance
    if ((left > DESIRED_DISTANCE) && (right > DESIRED_DISTANCE))
    {
        // Calculate the set point as the average of the left and right sensor distance readings
        Set_Point = (left + right) / 2;
    }
    else
    {
        // If at least one distance sensor reading is below the desired distance, assign the set point
        Set_Point = DESIRED_DISTANCE;
    }

    // Calculate the error based on the sensor readings
    if (left < right)
    {
        Error = left - Set_Point;
    }
    else
    {
        Error = Set_Point - right;
    }

    // Calculate the new duty cycle for the right motor based on the error and proportional constant (Kp)
    Duty_Cycle_Right = PWM_NOMINAL + (Kp * Error);

    // Calculate the new duty cycle for the left motor based on the error and proportional constant (Kp)
    Duty_Cycle_Left  = PWM_NOMINAL - (Kp * Error);

    // Clamp duty cycles
    if (Duty_Cycle_Right < PWM_MIN) Duty_Cycle_Right = PWM_MIN;
    if (Duty_Cycle_Right > PWM_MAX) Duty_Cycle_Right = PWM_MAX;
    if (Duty_Cycle_Left  < PWM_MIN) Duty_Cycle_Left  = PWM_MIN;
    if (Duty_Cycle_Left  > PWM_MAX) Duty_Cycle_Left  = PWM_MAX;

#ifndef DEBUG_ACTIVE
    Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);
#endif
}


// Interrupt handlers
/**
 * @brief SysTick Handler at 100 Hz.
 */
void SysTick_Handler(void)
{
#if defined CONTROLLER_1

    Controller_1();

#elif defined CONTROLLER_2
    #if defined CONTROLLER_1 || defined CONTROLLER_3
        #error "Only CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3 can be active at the same time."
    #endif

    // Controller_2();  // not used

#elif defined CONTROLLER_3
    #if defined CONTROLLER_1 || defined CONTROLLER_2
        #error "Only CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3 can be active at the same time."
    #endif

    // Controller_3();  // not used

#else
    #error "Define either one of the options: CONTROLLER_1, CONTROLLER_2, or CONTROLLER_3."
#endif
}

/**
 * @brief Timer A1 periodic task at 2 kHz: sample distance sensors.
 */
void Timer_A1_Periodic_Task(void)
{
    Sample_Analog_Distance_Sensor();
}


int main(void)
{
    // Distance sensor raw values (first read for LPF init)
    uint32_t Raw_A17;
    uint32_t Raw_A14;
    uint32_t Raw_A16;

    // DHT22 / irrigation variables
    float temp_c, humidity;
    int status;

    // Stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Initialize 48 MHz Clock
    Clock_Init48MHz();

    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize UART for printf to terminal
    EUSCI_A0_UART_Init_Printf();

    // Initialize DC motors
    Motor_Init();

    // Initialize motor duty cycle values
    Duty_Cycle_Left  = PWM_NOMINAL;
    Duty_Cycle_Right = PWM_NOMINAL;

    // Initialize Analog Distance Sensors (ADC14)
    Analog_Distance_Sensor_Init();

    // First conversion to get initial values for LPF
    Analog_Distance_Sensor_Start_Conversion(&Raw_A17, &Raw_A14, &Raw_A16);

    // Initialize low-pass filters for distance sensors
    LPF_Init(Raw_A17, 64);
    LPF_Init2(Raw_A14, 64);
    LPF_Init3(Raw_A16, 64);

    // Initialize SysTick for 100 Hz controller ticks
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 for 2 kHz sensor sampling
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize DHT22 + LED PWM for irrigation state machine
    DHT22_Init();
    LED_PWM_Init();

    current_state = IRR_STATE_STARTUP;

    // Enable global interrupts so SysTick and Timer A1 start running
    EnableInterrupts();

    printf("Self-Sustaining Environmental Patrol Robot started\r\n");

    // Main loop: environmental monitor runs here
    while (1)
    {
        status = DHT22_Read(&temp_c, &humidity);

        if (status == 0)
        {
            // Update irrigation state and outputs (LED + UART)
            Irrigation_UpdateState(humidity);
            Irrigation_ApplyOutputs(temp_c, humidity);
        }
        else
        {
            // Sensor error: log and go to a safe state
            current_state = IRR_STATE_STARTUP;
            LED_PWM_SetDutyPercent(0.0f);
            printf("[STATE: SENSOR ERROR] DHT22 read failed\r\n");
        }

        // Take a reading every 2 seconds
        Delay_ms(2000);
        // During this delay, SysTick_Handler and Timer_A1_Periodic_Task
        // are still running in the background, so the robot keeps wall-following.
    }
}
