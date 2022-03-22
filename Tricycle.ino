#include <Wire.h>
#include <EEPROM.h>
#include <GyverPWM.h>
#include <GyverTimers.h>
#include <GyverButton.h>
#include "Definitions.h"
#include "AS5600.h"

AMS_5600 ams5600;

volatile enum Mode current_mode = IDLE;
volatile unsigned long current_speed = 0;
volatile unsigned long speed_change_step = 0;
volatile enum BatteryState battery_state = THREE;
volatile enum PowerState power_state = POW3;

volatile int current_throttle;
int initial_throttle = 0;

volatile unsigned long batt_blink_millis = 0;
volatile unsigned long overload_start_millis = 0;
volatile unsigned long pow_blink_millis = 0;
volatile bool batt_blink_low = false;
volatile bool pow_blink_low = false;

volatile unsigned long current_max_speed = POWER_MODE3 * SPEED_COEFFICIENT;

GButton butt1(BTN_ONE_PIN);
GButton butt2(BTN_TWO_PIN);

void setup()
{
	Timer1.setPeriod(MAIN_TIMER_INTERVAL * MILLISECONDS_COEFFICIENT);
	Timer1.enableISR();

	//Serial.begin(9600);

	pinMode(MOTOR_PIN, OUTPUT);
	pinMode(PLED_PIN0, OUTPUT);
	pinMode(PLED_PIN1, OUTPUT);
	pinMode(PLED_PIN2, OUTPUT);
	pinMode(BLED_PIN0, OUTPUT);
	pinMode(BLED_PIN1, OUTPUT);
	pinMode(BLED_PIN2, OUTPUT);
	pinMode(BTN_ONE_PIN, INPUT);
	pinMode(BTN_TWO_PIN, INPUT);

	butt1.setTimeout(HOLD_TIMEOUT);
	butt2.setTimeout(HOLD_TIMEOUT);

	butt1.setType(HIGH_PULL);
	butt2.setType(HIGH_PULL);

	butt1.setDirection(NORM_OPEN);
	butt2.setDirection(NORM_OPEN);	

	//InitEeprom();

	PWM_frequency(MOTOR_PIN, PWM_FREQUENCY, CORRECT_PWM);

	Wire.begin();

	initial_throttle = CalculateCurrentThrottle();	
}

auto loop() -> void
{
	CheckCurrent();
	CheckVoltage();
	CheckUi();

	UpdateCurrentThrottle();	
}

int CalculateCurrentThrottle()
{
	const auto newAngle = ams5600.getRawAngle();
	
	/* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
	const double retVal = newAngle * 0.087;	
	return round(retVal);
}

void UpdateCurrentThrottle()
{
	if(current_mode == HALT)
	{
		current_throttle = MIN_THROTTLE_POS;
		return;
	}

	auto curr = CalculateCurrentThrottle();

	if (curr <= initial_throttle)
	{
		current_throttle = MIN_THROTTLE_POS;
		return;
	}

	curr = curr - initial_throttle;

	if(curr < THROTTLE_OFFSET)
	{
		current_throttle = MIN_THROTTLE_POS;
		return;
	}

	if (curr > MAX_THROTTLE_POS)
	{
		curr = MAX_THROTTLE_POS;
	}

	current_throttle = curr;
}

ISR(TIMER1_A)
{
	if (current_mode == HALT)
	{
		return;
	}

	const auto throttle = GetThrottlePos();
	const auto thSpd = CalculateThrottleSpeed(throttle);

	if (thSpd < current_speed)
	{
		current_mode = DECEL;
		SetSpeedDifference(current_speed - thSpd, current_mode, thSpd);
	}
	else if (thSpd == current_speed)
	{
		if (speed_change_step > 0)
		{
			speed_change_step = speed_change_step <= 1
				? 0
				: speed_change_step - 1;

			const auto transition_period_max = current_mode == ACCEL ? ACCEL_PERIOD : DECEL_PERIOD;
			ChangeSpeed(current_mode, transition_period_max, thSpd);
		}
		else
		{
			if (thSpd == 0)
			{
				speed_change_step = 0;
				current_mode = IDLE;
				SetMotorPower(0);
				return;
			}

			current_mode = GOING;
		}

		SetMotorPower(current_speed);
	}
	else
	{
		current_mode = ACCEL;
		SetSpeedDifference(thSpd - current_speed, current_mode, thSpd);
	}
}

void Halt()
{
	Timer1.stop();
	current_mode = HALT;
	speed_change_step = 0;
	current_speed = 0;
	SetMotorPower(0);
}

auto CheckVoltage() -> void
{
	const auto volt = GetVoltage();

	const bool halt = current_mode == IDLE || current_mode == HALT ?
		                  volt < MIN_VOLTAGE_IDLE :
		                  volt < MIN_VOLTAGE_GOING;

	if(halt)
	{		
		Halt();
		battery_state = CUTOFF;
	}
	else if(current_mode == IDLE || current_mode == HALT)
	{
		battery_state = GetBatteryState(volt);
	}	

	ShowBatteryState(battery_state);
}

auto CheckCurrent() -> void
{
	if(current_mode == HALT)
	{
		return;
	}

	const double curr = GetCurrent();	

	if (curr > MAX_PEAK_CURRENT)
	{
		Halt();
		power_state = OVERLOAD_PEAK;
	}
	else if (curr > MAX_CONT_CURRENT)
	{
		auto currTime = millis();

		if (overload_start_millis == 0)
		{
			overload_start_millis = currTime;
		}
		else if (currTime - overload_start_millis > OVERLOAD_DURATION)
		{
			Halt();
			overload_start_millis = 0;			
			power_state = OVERLOAD;
		}
	}
	else
	{
		overload_start_millis = 0;
	}

	ShowPowerState(power_state);
}

unsigned long CalculateThrottleSpeed(int throttlePos)
{
	const auto dSpeed = current_max_speed / MAX_THROTTLE_POS;
	return dSpeed * throttlePos;
}

void SetSpeedDifference(unsigned long dSpeed, enum Mode mode, unsigned long throttleSpeed)
{
	const auto transitionPeriodMax = mode == ACCEL ? ACCEL_PERIOD : DECEL_PERIOD;
	const auto transitionSpeed = current_max_speed / transitionPeriodMax;
	const auto transitionTime = transitionSpeed == 0 ? 0 : dSpeed / transitionSpeed;
	const auto numberOfTicks = transitionTime / MAIN_TIMER_INTERVAL;

	speed_change_step = numberOfTicks;

	ChangeSpeed(mode, transitionPeriodMax, throttleSpeed);
	SetMotorPower(current_speed);
}

void ChangeSpeed(enum Mode mode, unsigned long transitionPeriodMax, unsigned long throttleSpeed)
{
	const auto speedIncrement = current_max_speed / (transitionPeriodMax / MAIN_TIMER_INTERVAL);	
	
	if (mode == ACCEL)
	{
		current_speed += speedIncrement; //current > throttle 
		if (current_speed > throttleSpeed)
		{
			current_speed = throttleSpeed;
		}
	}
	else
	{
		if (current_speed < speedIncrement)
		{
			current_speed = 0;
		}
		else
		{
			current_speed -= speedIncrement;
		}		
	}	
}

void SetMotorPower(unsigned long speed)
{	
	PWM_set(MOTOR_PIN, static_cast<unsigned>(speed <= 0
		                                         ? 0
		                                         : speed / SPEED_COEFFICIENT));
}

void CheckUi()
{
	butt1.tick();
	butt2.tick();

	/*if (butt1.isHold() && butt2.isHold())
	{
	}	*/

	if (butt2.isClick())
	{
		SetPowerState(true);		
	}

	if (butt1.isClick())
	{
		SetPowerState(false);
	}
}

int GetThrottlePos()
{	
	return current_throttle;
}

void InitEeprom()
{
	const auto flag = EEPROM.read(INIT_ADDRESS);

	//power_state = POW3;

	if (flag == 0)
	{
		power_state = static_cast<PowerState>(EEPROM.read(FLAG_ADDRESS));

		UpdateMaxSpeed();

		return;
	}
	
	EEPROM.write(INIT_ADDRESS, 0);
	EEPROM.write(FLAG_ADDRESS, power_state);
}

void SetPowerState(bool increment)
{	
	if(increment)
	{
		switch (power_state)
		{
		case POW1:
			power_state = POW2;
			break;
		case POW2:
			power_state = POW3;
			break;
		case POW3:
		case OVERLOAD:
		case OVERLOAD_PEAK:
			break;		
		}
	}
	else
	{
		switch (power_state)
		{
		case POW3:
			power_state = POW2;
			break;
		case POW2:
			power_state = POW1;
			break;
		case POW1:
		case OVERLOAD:
		case OVERLOAD_PEAK:
			break;
		}
	}

	UpdateMaxSpeed();

	current_speed = 0;
	speed_change_step = 0;
	current_mode = IDLE;

	if (power_state != OVERLOAD_PEAK && power_state != OVERLOAD)
	{
		EEPROM.write(FLAG_ADDRESS, power_state);
	}
}

void UpdateMaxSpeed()
{
	switch (power_state)
	{
	case POW1:
		current_max_speed = POWER_MODE1 * SPEED_COEFFICIENT;
		break;
	case POW2:
		current_max_speed = POWER_MODE2 * SPEED_COEFFICIENT;
		break;
	case POW3:
		current_max_speed = POWER_MODE3 * SPEED_COEFFICIENT;
		break;
	case OVERLOAD:
	case OVERLOAD_PEAK:
		current_max_speed = 0;
	}
}

void ShowBatteryState(enum BatteryState state)
{
	switch (state)
	{
	case ONE:
	{
		digitalWrite(BLED_PIN0, HIGH);
		digitalWrite(BLED_PIN1, LOW);
		digitalWrite(BLED_PIN2, LOW);
	}break;
	case TWO:
	{
		digitalWrite(BLED_PIN0, HIGH);
		digitalWrite(BLED_PIN1, HIGH);
		digitalWrite(BLED_PIN2, LOW);
	}break;
	case THREE:
	{
		digitalWrite(BLED_PIN0, HIGH);
		digitalWrite(BLED_PIN1, HIGH);
		digitalWrite(BLED_PIN2, HIGH);
	}break;
	case CUTOFF:
	{
		auto curr = millis();

			if(batt_blink_low)
			{
				digitalWrite(BLED_PIN0, LOW);
				digitalWrite(BLED_PIN1, LOW);
				digitalWrite(BLED_PIN2, LOW);				
			}
			else
			{
				digitalWrite(BLED_PIN0, HIGH);
				digitalWrite(BLED_PIN1, HIGH);
				digitalWrite(BLED_PIN2, HIGH);				
			}

			if (curr - batt_blink_millis >= LOW_BATTERY_BLINK_PERIOD)
			{
				batt_blink_low = !batt_blink_low;
				batt_blink_millis = curr;
			}
	}
	break;
	}
}

void ShowPowerState(enum PowerState state)
{
	switch (state)
	{
	case POW1:
	{
		digitalWrite(PLED_PIN0, HIGH);
		digitalWrite(PLED_PIN1, LOW);
		digitalWrite(PLED_PIN2, LOW);
	} break;
	case POW2:
	{
		digitalWrite(PLED_PIN0, HIGH);
		digitalWrite(PLED_PIN1, HIGH);
		digitalWrite(PLED_PIN2, LOW);
	} break;
	case POW3:
	{
		digitalWrite(PLED_PIN0, HIGH);
		digitalWrite(PLED_PIN1, HIGH);
		digitalWrite(PLED_PIN2, HIGH);
	} break;
	case OVERLOAD:
	{
		auto curr = millis();

		if (pow_blink_low)
		{
			digitalWrite(PLED_PIN0, LOW);
			digitalWrite(PLED_PIN1, HIGH);
			digitalWrite(PLED_PIN2, LOW);
		}
		else
		{
			digitalWrite(PLED_PIN0, HIGH);
			digitalWrite(PLED_PIN1, LOW);
			digitalWrite(PLED_PIN2, HIGH);
		}

		if (curr - pow_blink_millis >= OVERLOAD_BLINK_PERIOD)
		{
			pow_blink_low = !pow_blink_low;
			pow_blink_millis = curr;
		}
	} break;
	case OVERLOAD_PEAK:
	{
		auto curr = millis();

		if (pow_blink_low)
		{
			digitalWrite(PLED_PIN0, LOW);
			digitalWrite(PLED_PIN1, LOW);
			digitalWrite(PLED_PIN2, LOW);
		}
		else
		{
			digitalWrite(PLED_PIN0, HIGH);
			digitalWrite(PLED_PIN1, HIGH);
			digitalWrite(PLED_PIN2, HIGH);
		}

		if (curr - pow_blink_millis >= OVERLOAD_BLINK_PERIOD)
		{
			pow_blink_low = !pow_blink_low;
			pow_blink_millis = curr;
		}
	} break;
	}
}

enum BatteryState GetBatteryState(double voltage)
{
	if (battery_state == CUTOFF)
	{
		return battery_state;
	}

	if (voltage <= BATT_THR_ONE)
	{
		return ONE;
	}
	if (voltage <= BATT_THR_TWO)
	{
		return TWO;
	}

	return THREE;
}

double GetVoltage()
{
	const auto vlt = analogRead(VOLTAGE_PIN);
	const auto calculated = static_cast<double>(vlt) * ANALOG_COEFFICIENT * VOLTAGE_COEFFICIENT;

	return calculated;
}

double GetCurrent()
{
	const auto curr = analogRead(CURRENT_PIN);
	const auto voltage = ANALOG_COEFFICIENT * static_cast<double>(curr);
	const auto current = fabs(CURRENT_MIDDLE_POINT - voltage) / CURRENT_COEFFICIENT;
	return current;
}
