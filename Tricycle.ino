#include <Boards.h>
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
volatile enum PowerState power_state_save = POW3;

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

	pinMode(MOTOR_PIN, OUTPUT);
	pinMode(PLED_PIN0, OUTPUT);
	pinMode(PLED_PIN1, OUTPUT);
	pinMode(PLED_PIN2, OUTPUT);
	pinMode(BLED_PIN0, OUTPUT);
	pinMode(BLED_PIN1, OUTPUT);
	pinMode(BLED_PIN2, OUTPUT);
	pinMode(BTN_ONE_PIN, INPUT);
	pinMode(BTN_TWO_PIN, INPUT);
	pinMode(CURRENT_PIN, INPUT);

	pinMode(D1, INPUT);
	pinMode(D2, INPUT);
	
	butt1.setTimeout(HOLD_TIMEOUT);
	butt2.setTimeout(HOLD_TIMEOUT);

	butt1.setType(HIGH_PULL);
	butt2.setType(HIGH_PULL);

	butt1.setDirection(NORM_OPEN);
	butt2.setDirection(NORM_OPEN);	

	InitEeprom();

	PWM_frequency(MOTOR_PIN, PWM_FREQUENCY, CORRECT_PWM);

	Wire.begin();

	initial_throttle = CalculateCurrentThrottle();

	Serial.begin(9600);
}

auto loop() -> void
{	
	//CheckCurrent();
	//CheckVoltage();
	CheckUi();
		
	UpdateCurrentThrottle();	

	//delay(100);

	//SetMotorPower(255000);
}

int CalculateCurrentThrottle()
{
	auto newAngle = ams5600.getRawAngle();

	/*if (millis() > 5000)
	{
		newAngle = 90;
	}

	if (millis() > 10000)
	{
		newAngle = 190;
	}

	if (millis() > 15000)
	{
		newAngle = 287;
	}

	if (millis() > 20000)
	{
		newAngle = 190;
	}

	if (millis() > 25000)
	{
		newAngle = 90;
	}

	if (millis() > 30000)
	{
		newAngle = 0;
	}*/

	/* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
	const double retVal = newAngle * 0.087;
	auto result = round(retVal);

	
	return result;
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
		SetMotorPower(0);
		return;
	}

	const unsigned long throttle = GetThrottlePos();
	const unsigned long thSpd = CalculateThrottleSpeed(throttle);

	if (thSpd < current_speed || thSpd == 0)
	{
		current_mode = DECEL;
		SetSpeedDifference(current_speed - thSpd, current_mode, thSpd);
	}
	else if (thSpd == current_speed)
	{
		if (speed_change_step >= 1)
		{
			speed_change_step -= 1;

			const unsigned long transition_period_max = current_mode == ACCEL ? ACCEL_PERIOD : DECEL_PERIOD;
			ChangeSpeed(current_mode, transition_period_max, thSpd);
		}
		else
		{
			speed_change_step = 0;
			if (thSpd == 0)
			{				
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
	pow_blink_millis = 0;
	batt_blink_millis = 0;

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
		ShowPowerState(power_state);
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

unsigned long CalculateThrottleSpeed(unsigned long throttlePos)
{
	if(throttlePos == 0)
	{
		return 0;
	}

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

	Serial.print("__");
	Serial.print(transitionPeriodMax);
	Serial.print("__");
	Serial.print(transitionSpeed);
	Serial.print("__");
	Serial.print(transitionTime);
	Serial.print("__");
	Serial.print(numberOfTicks);
	Serial.print("__");
	Serial.print(dSpeed);
	Serial.print("__");
	Serial.print(throttleSpeed);
	Serial.print("__");
	Serial.print(current_speed);
	Serial.println();

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

	if (current_speed > current_max_speed)
	{
		current_speed = current_max_speed;
	}
}

void SetMotorPower(unsigned long speed)
{
	//Serial.println("++");
	//Serial.print(speed);
	//Serial.print("++");
	//Serial.println();

	PWM_set(MOTOR_PIN, static_cast<unsigned>(speed <= 0
		                                         ? 0
		                                         : speed / SPEED_COEFFICIENT));
}

void CheckUi()
{
	const auto state = IsWorkingPowerState();

	if (state && IsD1())
	{
		Halt();
		power_state_save = power_state;
		power_state = RC_HALT;
		return;
	}

	if (power_state == RC_HALT && IsD2())
	{
		power_state = power_state_save;
		current_mode = IDLE;
		Timer1.restart();
		return;
	}

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

unsigned long GetThrottlePos()
{	
	return (unsigned long)current_throttle;
}

void InitEeprom()
{
	const auto flag = EEPROM.read(INIT_ADDRESS);	

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
	/*if(power_state == OVERLOAD_PEAK || power_state == OVERLOAD)
	{
		return;
	}*/

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
		case RC_HALT:
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
		case RC_HALT:
			break;
		}
	}

	UpdateMaxSpeed();

	current_speed = 0;
	speed_change_step = 0;
	current_mode = IDLE;

	if (IsWorkingPowerState())
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
	case RC_HALT:
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

		if (batt_blink_low)
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

		if (batt_blink_millis == 0 || curr - batt_blink_millis >= LOW_BATTERY_BLINK_PERIOD)
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

		if (pow_blink_millis == 0 || curr - pow_blink_millis >= OVERLOAD_BLINK_PERIOD)
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

		if (pow_blink_millis == 0 || curr - pow_blink_millis >= OVERLOAD_BLINK_PERIOD)
		{
			pow_blink_low = !pow_blink_low;
			pow_blink_millis = curr;
		}
	} break;
	case RC_HALT:
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
			digitalWrite(PLED_PIN1, HIGH);			
		}

		if (pow_blink_millis == 0 || curr - pow_blink_millis >= RCHALT_BLINK_PERIOD)
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

bool IsD1()
{
	auto res = analogRead(D1);
	return res >= ANALOG_BIT_THRESHOLD;
}

bool IsD2()
{
	auto res = analogRead(D2);
	return res >= ANALOG_BIT_THRESHOLD;
}

bool IsWorkingPowerState()
{
	return power_state == POW1 || power_state == POW2 || power_state == POW3;
}

double GetDoubleFromEeeprom(short address) 
{
	unsigned char temp[4];

	temp[0] = EEPROM.read(address);
	temp[1] = EEPROM.read(static_cast<short>(address + 1));
	temp[2] = EEPROM.read(static_cast<short>(address + 2));
	temp[3] = EEPROM.read(static_cast<short>(address + 3));

	const auto result = *reinterpret_cast<double*>(temp);
	return result;
}

void SaveDoubleToEeprom(double val, short address) 
{
	const auto value = reinterpret_cast<unsigned char*>(&val);

	EEPROM.write(address, value[0]);
	EEPROM.write(static_cast<short>(address + 1), value[1]);
	EEPROM.write(static_cast<short>(address + 2), value[2]);
	EEPROM.write(static_cast<short>(address + 3), value[3]);
}
