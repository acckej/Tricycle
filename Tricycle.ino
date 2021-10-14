#include <Wire.h>
#include <EEPROM.h>
#include <GyverPWM.h>
#include <GyverTimers.h>
#include <GyverEncoder.h>
#include <GyverButton.h>
#include <AnalogKey.h>
#include "Definitions.h"

volatile Mode _mode = IDLE;
volatile unsigned long _currentSpeed = 0;
volatile unsigned long _speedChangeStep = 0;
volatile PowerState _powerState = POW3;
volatile BatteryState _batteryState = THREE;

volatile int _currentThrottle;
volatile boolean _state0, _lastState, _turnFlag;

volatile unsigned long _battBlinkMillis = 0;
volatile unsigned long _powBlinkMillis = 0;
volatile bool _battBlinkLow = false;
volatile bool _powBlinkLow = false;

volatile unsigned long _currentMaxSpeed = POWER_MODE3 * SPEED_COEFFICIENT;

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

	attachInterrupt(0, EncoderInterrupt, CHANGE);

	InitEeprom();

	PWM_frequency(MOTOR_PIN, 2000, CORRECT_PWM);
}

void loop()
{
	CheckCurrent();
	CheckVoltage();
	CheckUi();

	/*char* buf = "                                  ";	
	sprintf(buf, "%d %lu\n", _currentThrottle, _currentSpeed);
	Serial.write(buf);*/

	/*digitalWrite(PLED_PIN0, HIGH);
	digitalWrite(PLED_PIN1, HIGH);
	digitalWrite(PLED_PIN2, HIGH);*/

	//digitalWrite(MOTOR_PIN, HIGH);

	//SetMotorPower(255 / MAX_THROTTLE_POS * _currentThrottle * SPEED_COEFFICIENT);

	/*auto mil = millis();

	if(mil > 20000)
	{
		SetMotorPower(255 * SPEED_COEFFICIENT);
	}
	else if(mil > 15000)
	{
		SetMotorPower(128 * SPEED_COEFFICIENT);
	}
	else if(mil > 10000)
	{
		SetMotorPower(12 * SPEED_COEFFICIENT);
	}
	else if(mil > 5000)
	{
		SetMotorPower(2 * SPEED_COEFFICIENT);
	}*/

}

void EncoderInterrupt()
{
	_state0 = bitRead(PIND, ENCODER_PIN_A);
	if (_state0 != _lastState)
	{
#if (ENC_TYPE == 1)
		_turnFlag = !_turnFlag;
		if (turnFlag)
			_currentThrottle += (bitRead(PIND, ENCODER_PIN_B) != _lastState) ? -1 : 1;
#else
		_currentThrottle += (bitRead(PIND, ENCODER_PIN_B) != _lastState) ? -1 : 1;
#endif
		_lastState = _state0;
	}

	if (_currentThrottle <= 0)
	{
		_currentThrottle = 0;
	}
	else if (_currentThrottle >= MAX_THROTTLE_POS)
	{
		_currentThrottle = MAX_THROTTLE_POS;
	}
}

ISR(TIMER1_A)
{	
	if (_mode == HALT)
	{		
		SetMotorPower(0);
		_speedChangeStep = 0;
		_currentSpeed = 0;
		return;
	}

	const auto throttle = GetThrottlePos();
	const auto thSpd = CalculateThrottleSpeed(throttle);	

	if (thSpd < _currentSpeed)
	{
		_mode = DECEL;
		SetSpeedDifference(_currentSpeed - thSpd, _mode, thSpd);
	}
	else if (thSpd == _currentSpeed)
	{
		if (_speedChangeStep > 0)
		{
			_speedChangeStep--;
			const auto transitionPeriodMax = _mode == ACCEL ? ACCEL_PERIOD : DECEL_PERIOD;
			ChangeSpeed(_mode, transitionPeriodMax, thSpd);
		}
		else
		{
			if (thSpd == 0)
			{
				_mode = IDLE;
				SetMotorPower(0);
				return;
			}

			_mode = GOING;
		}

		SetMotorPower(_currentSpeed);
	}
	else
	{
		_mode = ACCEL;
		SetSpeedDifference(thSpd - _currentSpeed, _mode, thSpd);
	}
}

void CheckVoltage()
{
	const auto volt = GetVoltage();

	const bool halt = _mode == IDLE || _mode == HALT ?
		                  volt < MIN_VOLTAGE_IDLE :
		                  volt < MIN_VOLTAGE_GOING;

	if(halt)
	{		
		Timer1.stop();
		_mode = HALT;
		SetMotorPower(0);
		_batteryState = CUTOFF;
	}
	else if(_mode == IDLE || _mode == HALT)
	{
		_batteryState = GetBatteryState(volt);
	}	

	ShowBatteryState(_batteryState);
}

void CheckCurrent()
{
	const auto curr = GetCurrent();

	if (curr > MAX_CURRENT)
	{
		Timer1.stop();
		_mode = HALT;
		SetMotorPower(0);		
		_powerState = OVERLOAD;		
	}

	ShowPowerState(_powerState);
}

unsigned long CalculateThrottleSpeed(int throttlePos)
{
	const auto dSpeed = _currentMaxSpeed / MAX_THROTTLE_POS;
	return dSpeed * throttlePos;
}

void SetSpeedDifference(unsigned long dSpeed, Mode mode, unsigned long throttleSpeed)
{
	const auto transitionPeriodMax = mode == ACCEL ? ACCEL_PERIOD : DECEL_PERIOD;
	const auto transitionSpeed = _currentMaxSpeed / transitionPeriodMax;
	const auto transitionTime = transitionSpeed == 0 ? 0 : dSpeed / transitionSpeed;
	const auto numberOfTicks = transitionTime / MAIN_TIMER_INTERVAL;

	_speedChangeStep = numberOfTicks;

	ChangeSpeed(mode, transitionPeriodMax, throttleSpeed);
	SetMotorPower(_currentSpeed);
}

void ChangeSpeed(Mode mode, unsigned long transitionPeriodMax, unsigned long throttleSpeed)
{
	const auto speedIncrement = _currentMaxSpeed / (transitionPeriodMax / MAIN_TIMER_INTERVAL);	
	
	if (mode == ACCEL)
	{
		_currentSpeed += speedIncrement; //current > throttle 
		if (_currentSpeed > throttleSpeed)
		{
			_currentSpeed = throttleSpeed;
		}
	}
	else
	{
		if (_currentSpeed < speedIncrement)
		{
			_currentSpeed = 0;
		}
		else
		{
			_currentSpeed -= speedIncrement;
		}		
	}	
}

void SetMotorPower(unsigned long speed)
{	
	PWM_set(MOTOR_PIN, speed <= 0 ? 0 : speed / SPEED_COEFFICIENT);
}

void CheckUi()
{
	butt1.tick();
	butt2.tick();

	/*if (butt1.isHold() && butt2.isHold())
	{
	}	*/

	if (butt1.isClick())
	{
		SetPowerState(false);		
	}

	if (butt2.isClick())
	{
		SetPowerState(true);		
	}
}

int GetThrottlePos()
{
	return _currentThrottle;
}

void InitEeprom()
{
	const auto flag = EEPROM.read(INIT_ADDRESS);

	if (flag == 0)
	{
		_powerState = (PowerState)EEPROM.read(FLAG_ADDRESS);
		return;
	}
	
	EEPROM.write(INIT_ADDRESS, 0);
	EEPROM.write(FLAG_ADDRESS, _powerState);
}

void SetPowerState(bool increment)
{	
	if(increment)
	{
		switch (_powerState)
		{
		case POW1:
			_powerState = POW2;
			break;
		case POW2:
			_powerState = POW3;
			break;
		case POW3:
		case OVERLOAD:
			break;
		}
	}
	else
	{
		switch (_powerState)
		{
		case POW3:
			_powerState = POW2;
			break;
		case POW2:
			_powerState = POW1;
			break;
		case POW1:
		case OVERLOAD:
			break;
		}
	}

	switch (_powerState)
	{
	case POW1:	
		_currentMaxSpeed = POWER_MODE3 * SPEED_COEFFICIENT;	
		break;
	case POW2:	
		_currentMaxSpeed = POWER_MODE2 * SPEED_COEFFICIENT;	
		break;
	case POW3:
		_currentMaxSpeed = POWER_MODE1 * SPEED_COEFFICIENT;
	case OVERLOAD:
		_currentMaxSpeed = 0;		
	}

	EEPROM.write(FLAG_ADDRESS, _powerState);	
}

void ShowBatteryState(BatteryState state)
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

			if(_battBlinkLow)
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

			if (curr - _battBlinkMillis >= LOW_BATTERY_BLINK_PERIOD)
			{
				_battBlinkLow = !_battBlinkLow;
				_battBlinkMillis = curr;
			}
	}
	break;
	}
}

void ShowPowerState(PowerState state)
{
	switch (state)
	{
	case POW1:
	{
		digitalWrite(PLED_PIN0, HIGH);
		digitalWrite(PLED_PIN1, LOW);
		digitalWrite(PLED_PIN2, LOW);
	}break;
	case POW2:
	{
		digitalWrite(PLED_PIN0, HIGH);
		digitalWrite(PLED_PIN1, HIGH);
		digitalWrite(PLED_PIN2, LOW);
	}break;
	case POW3:
	{
		digitalWrite(PLED_PIN0, HIGH);
		digitalWrite(PLED_PIN1, HIGH);
		digitalWrite(PLED_PIN2, HIGH);
	}break;
	case OVERLOAD:
	{
		auto curr = millis();

		if (_powBlinkLow)
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

		if (curr - _powBlinkMillis >= OVERLOAD_BLINK_PERIOD)
		{
			_powBlinkLow = !_powBlinkLow;
			_powBlinkMillis = curr;
		}
	}
	break;
	}
}

BatteryState GetBatteryState(double voltage)
{
	if (_batteryState == CUTOFF)
	{
		return _batteryState;
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
