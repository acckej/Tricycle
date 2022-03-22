#pragma once

#define PLED_PIN0 12
#define PLED_PIN1 13
#define PLED_PIN2 10

#define BLED_PIN0 8
#define BLED_PIN1 7
#define BLED_PIN2 6

#define MOTOR_PIN 11 //3

#define BTN_ONE_PIN 4
#define BTN_TWO_PIN 5
#define VOLTAGE_PIN A1

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3 //9
#define CURRENT_PIN A2

#define ENC_TYPE 0

#define MAX_THROTTLE_POS 35
#define MIN_THROTTLE_POS 0
#define THROTTLE_OFFSET 10

#define MAIN_TIMER_INTERVAL (uint32_t)50
#define ACCEL_PERIOD (uint32_t)1200
#define DECEL_PERIOD (uint32_t)1001
#define HOLD_TIMEOUT 2000

#define SPEED_COEFFICIENT (uint32_t)1000
#define POWER_MODE1 (uint32_t)77
#define POWER_MODE2 (uint32_t)153
#define POWER_MODE3 (uint32_t)255
#define MILLISECONDS_COEFFICIENT (uint32_t)1000
#define PWM_FREQUENCY 20000

#define MIN_VOLTAGE_IDLE 9.0
#define MIN_VOLTAGE_GOING 7.0
#define ANALOG_COEFFICIENT 0.0048828125
#define VOLTAGE_COEFFICIENT 2.8
#define MAX_PEAK_CURRENT 35.0f
#define MAX_CONT_CURRENT 25.0f
#define CURRENT_MIDDLE_POINT 2.5f
#define CURRENT_COEFFICIENT 0.066f

#define BATT_THR_THREE 11.9
#define BATT_THR_TWO 10.7
#define BATT_THR_ONE 9.5

#define OVERLOAD_DURATION 4000
#define OVERLOAD_BLINK_PERIOD 500
#define LOW_BATTERY_BLINK_PERIOD 1000

#define INIT_ADDRESS 0
#define FLAG_ADDRESS 1

enum Mode : byte
{
	ACCEL,
	DECEL,
	IDLE,
	GOING,
	HALT
};

enum BatteryState : byte
{
	ONE,
	TWO,
	THREE,
	CUTOFF	
};

enum PowerState : byte
{
	POW1 = 1,
	POW2 = 2,
	POW3 = 3,
	OVERLOAD = 4,
	OVERLOAD_PEAK = 5
};

int GetThrottlePos();

double GetVoltage();

double GetCurrent();

void SetMotorPower(unsigned long speed);

void ShowBatteryState(enum BatteryState state);

void ShowPowerState(enum PowerState state);

void CheckCurrent();

void UpdateCurrentThrottle();

void CheckVoltage();

void SetPowerState(bool increment);

unsigned long CalculateThrottleSpeed(int throttlePos);

void SetSpeedDifference(unsigned long dSpeed, enum Mode mode, unsigned long throttleSpeed);

void ChangeSpeed(enum Mode mode, unsigned long transitionPeriodMax, unsigned long throttleSpeed);

void CheckUi();

void UpdateMaxSpeed();

void Halt();

void EncoderInterrupt();

void InitEeprom();

enum BatteryState GetBatteryState(double voltage);

int CalculateCurrentThrottle();





