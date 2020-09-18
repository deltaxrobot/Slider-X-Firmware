#define ENDSTOP_PIN A0

#define DIR_PIN 10
#define STEP_PIN 11
#define EN_PIN 12

#define LED_PIN 13

#define COMPARE_VALUE_TIMER OCR1A

#define TurnOnTimer1 (TIMSK1 |= (1 << OCIE1A))
#define TurnOffTimer1 (TIMSK1 &= ~(1 << OCIE1A))

#define COMMAND_PORT Serial

#define DEFAULT_ACCELERATION 1000 // mm/s2

#define MAX_SPEED 81 // mm/s
#define DEFAULT_SPEED 30
#define HOMING_SPEED 20

#define MAX_POSITION 300

#define STEP_PER_MM 240

#define SPEED_TO_CYCLE(x) (1000000.0 / (STEP_PER_MM * x))

#include "MultiThread.h"

String inputString;
bool stringComplete;
float DesireSpeed;
float OldSpeed;
float LinearSpeed;
float Accel;
float DesirePosition;
float CurrentPosition;
long DesireSteps;
long PassedSteps;
unsigned long PassedTime;
long AccelSteps;
float TempCycle;
unsigned long CycleOffset;

bool isEnding = false;
bool isHoming = false;
bool isMoving = false;
bool blink = false;

MultiThread LedBlinkScheduler;

void setup()
{
	COMMAND_PORT.begin(115200);
	IOInit();
	setValue();
	TimerInit();
}

void loop()
{
	Home();
	SerialExecute();
	CaculateTempCycle();
	SliderExecute();
	LedBlink();
}

void IOInit()
{
	pinMode(DIR_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(EN_PIN, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(EN_PIN, 1);
}

void setValue()
{
	COMMAND_PORT.println("Begin:");
	DesireSpeed = OldSpeed = DEFAULT_SPEED;
	Accel = DEFAULT_ACCELERATION;

	DesirePosition = 0;
	CurrentPosition = 0;
	DesireSteps = 0;
	PassedSteps = 0;
}

void TimerInit()
{
	noInterrupts();

	// Reset register relate to Timer 1
	// Reset register relate
	TCCR1A = TCCR1B = TCNT1 = 0;
	// Set CTC mode to Timer 1
	TCCR1B |= (1 << WGM12);
	// Set prescaler 1 to Timer 1
	TCCR1B |= (1 << CS10);
	//Normal port operation, OCxA disconnected
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0));

	interrupts();
}

void LedBlink()
{
	if (!blink)
	{
		digitalWrite(LED_BUILTIN, 0);
		return;
	}

	RUN_EVERY(LedBlinkScheduler, COMPARE_VALUE_TIMER / 16);
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void Home()
{
	if (isHoming && !digitalRead(ENDSTOP_PIN))
	{
		TurnOffTimer1;
		isHoming = false;
		isMoving = false;
		DesirePosition = 0;
		CurrentPosition = 0;
		DesireSteps = 0;
		PassedSteps = 0;
		AccelSteps = 0;
		digitalWrite(EN_PIN, 0);
		COMMAND_PORT.println("Ok");
	}
}

void SliderExecute()
{

	//speed
	if (DesireSpeed < 0.01 && DesireSpeed > -0.01)
	{
		DesireSpeed = 0;
	}

	if (DesireSpeed != 0)
	{
		DesireSpeed = abs(DesireSpeed);
	}

	if (DesireSpeed > MAX_SPEED)
	{
		DesireSpeed = MAX_SPEED;
	}

	//position
	if (DesirePosition > MAX_POSITION) DesirePosition = MAX_POSITION;

	if (DesirePosition == CurrentPosition)
		return;

	isMoving = true;

	if (DesirePosition - CurrentPosition > 0)
	{
		digitalWrite(DIR_PIN, 1);
		DesireSteps = roundf((DesirePosition - CurrentPosition) * STEP_PER_MM);
	}
	else
	{
		digitalWrite(DIR_PIN, 0);
		DesireSteps = roundf((CurrentPosition - DesirePosition) * STEP_PER_MM);
	}

	FinishMoving();
	TempCycle = SPEED_TO_CYCLE(LinearSpeed);
	setIntCycle(TempCycle);

	digitalWrite(EN_PIN, 0);
	TurnOnTimer1;
}

//intCycle us
void setIntCycle(float intCycle)
{
	int prescaler;

	if (intCycle > 4000)
	{
		TCCR1B |= (1 << CS11);
		TCCR1B &= ~(1 << CS10);
		prescaler = 8;
	}
	else
	{
		TCCR1B &= ~(1 << CS11);
		TCCR1B |= (1 << CS10);
		prescaler = 1;
	}

	COMPARE_VALUE_TIMER = roundf(intCycle * 16 / prescaler) - 1;
	//COMPARE_VALUE_TIMER = roundf(intCycle * F_CPU / (1000000.0 * prescaler)) - 1;
}

ISR(TIMER1_COMPA_vect)
{
	if (isMoving)
	{
		if (PassedSteps == DesireSteps) return;

		digitalWrite(STEP_PIN, 0);
		delayMicroseconds(3);
		digitalWrite(STEP_PIN, 1);

		PassedSteps++;
		PassedTime += TempCycle;
		if (LinearSpeed < DesireSpeed)
			AccelSteps++;
	}
}

void FinishMoving()
{
	if (isMoving && PassedSteps == DesireSteps)
	{
		CurrentPosition = DesirePosition;
		AccelSteps = 0;
		PassedTime = 0;
		LinearSpeed = 0;
		DesireSteps = 0;
		PassedSteps = 0;
		AccelSteps = 0;
		isMoving = false;
		isEnding = false;
		blink = false;
		TurnOffTimer1;
		COMMAND_PORT.println("Ok");
	}
}

void CaculateTempCycle()
{
	if (DesireSteps - PassedSteps < AccelSteps)
	{
		if (!isEnding) 
		{
			isEnding = true;
			PassedTime = 0;
			if (DesireSpeed > LinearSpeed) DesireSpeed = LinearSpeed;
		}
		LinearSpeed = DesireSpeed - Accel * PassedTime / 1000000;
		return;
	}

	if (LinearSpeed < DesireSpeed)
	{
		LinearSpeed = DesireSpeed / 5 + Accel * PassedTime / 1000000;
	}
	else
	{
		LinearSpeed = DesireSpeed;
	}
}

void SerialExecute()
{
	while (COMMAND_PORT.available())
	{
		char inChar = (char)COMMAND_PORT.read();

		if (inChar == '\n')
		{
			stringComplete = true;
			break;
		}

		inputString += inChar;
	}

	if (!stringComplete)
		return;

	String messageBuffer = inputString.substring(0, 4);

	if (messageBuffer == "M320")
	{
		isHoming = true;
		//isMoving = true;
		DesireSpeed = HOMING_SPEED;
		DesirePosition = -1000;
	}

	if (messageBuffer == "M321")
	{
		DesireSpeed = inputString.substring(5).toFloat();
		OldSpeed = DesireSpeed;
		COMMAND_PORT.println("Ok");
	}

	if (messageBuffer == "M322")
	{
		DesirePosition = inputString.substring(5).toFloat();

		if (DesirePosition < 0) DesirePosition = 0;
		if (DesirePosition == CurrentPosition)
		{
			COMMAND_PORT.println("Ok");
		}
		else
		{
			LinearSpeed = DesireSpeed / 5;
			TempCycle = DesireSpeed * SPEED_TO_CYCLE(DesireSpeed) / LinearSpeed;
			setIntCycle(TempCycle);
		}

		blink = true;
		DesireSpeed = OldSpeed;
	}

	if (messageBuffer == "M323")
	{
		digitalWrite(EN_PIN, 1);
		TurnOffTimer1;
		COMMAND_PORT.println("Ok");
	}

	inputString = "";
	stringComplete = false;
}
