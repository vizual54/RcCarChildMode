#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>

// Assign your channel in pins
#define THROTTLE_IN_PIN 6
#define STEERING_IN_PIN 5
#define AUX_IN_PIN 7

// Assign your channel out pins
#define THROTTLE_OUT_PIN 9
#define STEERING_OUT_PIN 8
#define AUX_OUT_PIN 10

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_FRAME_SPACE 3

// Assign leds
#define GREEN_LED 11
#define ORANGE_LED 12
#define RED_LED 13

// Assign potentiometers and gyro
#define GYRO_PIN 0
#define GYRO_GAIN_PIN 1
#define POT1_PIN 2
#define POT1_PIN 3

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4

#define THROTTLE_THRESHOLD 1500
#define MODE_SWITCH_DURATION 4000

volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAuxInShared;
uint16_t unThrottleInStart;
uint16_t unSteeringInStart;
uint16_t unAuxInStart;

uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;
uint16_t unRotationCenter = 276;
// gyro center voltage 1.35 volts 
// full range is 0 to 2*1.35 = 2.7V
// Using the Arduino voltage reference of 3.3 volts gives a center point of (1.35/(5.0/1024)) = 419

void setup()
{
	Serial.begin(115200);

	Serial.println("multiChannels");

	// attach servo objects, these will generate the correct
	// pulses for driving Electronic speed controllers, servos or other devices
	// designed to interface directly with RC Receivers
	CRCArduinoFastServos::attach(SERVO_THROTTLE, THROTTLE_OUT_PIN);
	CRCArduinoFastServos::attach(SERVO_STEERING, STEERING_OUT_PIN);
	CRCArduinoFastServos::attach(SERVO_AUX, AUX_OUT_PIN);

	// lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
	CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE, 7 * 2000);

	CRCArduinoFastServos::begin();

	// using the PinChangeInt library, attach the interrupts
	// used to read the channels
	PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
	PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering, CHANGE);
	PCintPort::attachInterrupt(AUX_IN_PIN, calcAux, CHANGE);

	pinMode(GREEN_LED, OUTPUT);
	pinMode(ORANGE_LED, OUTPUT);
	pinMode(RED_LED, OUTPUT);
	
	/*
	analogRead(GYRO_PIN);
	uint32_t ulTotal = 0;
	for (int nCount = 0; nCount < 50; nCount++)
	{
		ulTotal += analogRead(GYRO_PIN);
	}
	unRotationCenter = ulTotal / 50;
	*/

	digitalWrite(GREEN_LED, HIGH);
}

void loop()
{
	static uint16_t unThrottleIn;
	static uint16_t unSteeringIn;
	static uint16_t unAuxIn;
	// local copy of update flags
	static uint8_t bUpdateFlags;

	if (bUpdateFlagsShared)
	{
		noInterrupts();

		bUpdateFlags = bUpdateFlagsShared;

		if (bUpdateFlags & THROTTLE_FLAG)
		{
			unThrottleIn = unThrottleInShared;
		}

		if (bUpdateFlags & STEERING_FLAG)
		{
			unSteeringIn = unSteeringInShared;
		}

		if (bUpdateFlags & AUX_FLAG)
		{
			unAuxIn = unAuxInShared;
		}

		bUpdateFlagsShared = 0;

		interrupts();
	}

	// Get gyro value and calc steering
	if (bUpdateFlags & THROTTLE_FLAG)
	{
		CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE, unThrottleIn);
	}
	if (bUpdateFlags & STEERING_FLAG)
	{
		//CRCArduinoFastServos::writeMicroseconds(SERVO_STEERING, recalculateSteering(unSteeringIn));
		CRCArduinoFastServos::writeMicroseconds(SERVO_STEERING, recalculateSteering(1500));
	}
	if (bUpdateFlags & AUX_FLAG)
	{
		CRCArduinoFastServos::writeMicroseconds(SERVO_AUX, unAuxIn);
	}

	bUpdateFlags = 0;
}

uint16_t recalculateSteering(uint16_t steeringIn)
{
	static uint16_t unRotation;
	unRotation = analogRead(GYRO_PIN);
	uint16_t unSteeringIntervention = 0;
	uint8_t bInvert = false;

	static uint16_t unSteeringSensitivity;
	analogRead(GYRO_GAIN_PIN);
	unSteeringSensitivity = constrain(analogRead(GYRO_GAIN_PIN), 1, 1023);

	if (unRotation != unRotationCenter)
	{
		uint32_t ulRotationWithGain = 0;
		if (unRotation > unRotationCenter)
		{
			bInvert = true;
			ulRotationWithGain = (long)(unRotation - unRotationCenter)*unSteeringSensitivity;
			unSteeringIntervention = map(ulRotationWithGain, 0, (long)unRotationCenter * 128L, 0, 500);
		}
		else
		{
			ulRotationWithGain = (long)(unRotationCenter - unRotation)*unSteeringSensitivity;
			unSteeringIntervention = map(ulRotationWithGain, 0, (long)unRotationCenter * 128L, 0, 500);
		}

		Serial.print("Steering intervention: ");
		Serial.println(unSteeringIntervention);

		if (bInvert)
		{
			return constrain(steeringIn - unSteeringIntervention, 1000, 2000);
		}
		else
		{
			return constrain(steeringIn + unSteeringIntervention, 1000, 2000);
		}

	}

	return steeringIn;
}

void calcThrottle()
{
	if (PCintPort::pinState)
	{
		unThrottleInStart = TCNT1;
	}
	else
	{
		unThrottleInShared = (TCNT1 - unThrottleInStart) >> 1;
		bUpdateFlagsShared |= THROTTLE_FLAG;
	}
}

void calcSteering()
{
	if (PCintPort::pinState)
	{
		unSteeringInStart = TCNT1;
	}
	else
	{
		unSteeringInShared = (TCNT1 - unSteeringInStart) >> 1;

		bUpdateFlagsShared |= STEERING_FLAG;
	}
}

void calcAux()
{
	if (PCintPort::pinState)
	{
		unAuxInStart = TCNT1;
	}
	else
	{
		unAuxInShared = (TCNT1 - unAuxInStart) >> 1;
		bUpdateFlagsShared |= AUX_FLAG;
	}
}