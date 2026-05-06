#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// --- HARDWARE PIN DEFINITIONS ---
// Stepper Motor Pins (28BYJ-48 with ULN2003 driver)
const int STEPS_PER_REVOLUTION = 2048; 	// Common for 28BYJ-48 stepper
// Changed from 60 degrees (2048/6 = 341) to 36 degrees (2048/10 = 204.8, using 205)
const int THIRTY_SIX_DEGREES = 205; 	

// Ultrasonic sensor pins (HC-SR04)
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// LED Pins (Digital Pins 6 and 7)
const int RED_LED_PIN = 13;
const int BLUE_LED_PIN = 12;

// --- CONFIGURATION CONSTANTS ---
// Distance threshold (in cm). Item must be closer than this to be considered "present".
const float MAX_DISTANCE = 5.0; 	
// Time window (in milliseconds) to confirm item presence during the "return" sequence.
const unsigned long MONITORING_TIME_MS = 5000; 
// Buffer time (in milliseconds) after a sequence completes to prevent double-scanning.
const unsigned long POST_SEQUENCE_BUFFER_MS = 3000;
// Timing for Blinking and Polling
const unsigned long BLINK_INTERVAL_MS = 250; // LED state changes every 250ms
// Increased polling rate: Ultrasonic sensor checks every 5ms (previously 10ms)
const unsigned long POLLING_INTERVAL_MS = 5; 

// --- SYSTEM STATE VARIABLES ---
// 0 = Item is available/returned; 1 = Item is currently borrowed.
int redBorrow = 0;
int blueBorrow = 0;

// Initialize with pins for stepper motor (In1, In2, In3, In4)
Stepper myStepper(STEPS_PER_REVOLUTION, 2, 4, 3, 5);

// Initialize color sensor (TCS34725)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Function to measure distance using ultrasonic sensor
float measureDistance() {
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);
	
	// 15ms timeout is enough for roughly 250 cm
	long duration = pulseIn(ECHO_PIN, HIGH, 15000); 	
	
	if (duration == 0) {
		return -1; // Indicate no valid echo received
	}
	
	// Convert duration to distance (cm)
	float distance = duration * 0.034 / 2;
	
	// Filter out unrealistic readings (too far or too close)
	return (distance > 400 || distance < 2) ? -1 : distance;
}

// Function to handle the borrowing/checking sequence for a given color
void handleBorrowSequence(String color) {
	// Determine which state variables and pins to use
	int* currentBorrowCount;
	int ledPin;
	
	if (color == "RED") {
		currentBorrowCount = &redBorrow;
		ledPin = RED_LED_PIN;
	} else if (color == "BLUE") {
		currentBorrowCount = &blueBorrow;
		ledPin = BLUE_LED_PIN;
	} else {
		return; 
	}

	Serial.print("\nStarting ");
	Serial.print(color);
	Serial.println(" sequence...");

	// --- SCENARIO 1: BORROW ACTION (Count is 0, motor turns 36 degrees) ---
	if (*currentBorrowCount == 0) {
		Serial.println("STATUS: Item AVAILABLE. Performing BORROW action.");
		myStepper.setSpeed(10); // Set motor speed (RPM)
		myStepper.step(THIRTY_SIX_DEGREES); // Turn motor 36 degrees
		*currentBorrowCount = 1; 	 	// Set status to BORROWED
		Serial.print(color);
		Serial.println(" borrow count increased to 1.");
		
	} 
	// --- SCENARIO 2: CHECK/RETURN ACTION (Count is 1, ultrasonic check for 5s) ---
	else if (*currentBorrowCount == 1) {
		Serial.println("STATUS: Item BORROWED. Checking for RETURN (5s monitoring period)...");

		unsigned long startTime = millis();
		unsigned long lastBlinkTime = 0;
		unsigned long lastPollTime = 0;
		bool checkFailed = false; 
		
		// Ensure LED is ON at the start of the check sequence (will blink below)
		digitalWrite(ledPin, HIGH);

		while (millis() - startTime < MONITORING_TIME_MS) { 	// Run for 5 seconds
			
			// --- BLINKING LOGIC (Non-Blocking) ---
			if (millis() - lastBlinkTime >= BLINK_INTERVAL_MS) {
				digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED state
				lastBlinkTime = millis();
			}

			// --- POLLING LOGIC (Fast Rate: 5ms) ---
			if (millis() - lastPollTime >= POLLING_INTERVAL_MS) {
				float distance = measureDistance();
				lastPollTime = millis(); // Reset poll timer
				
				Serial.print("Distance: ");
				
				if (distance >= 0) {
					Serial.print(distance);
					Serial.println(" cm");
					
					// Check if distance is over threshold
					if (distance > MAX_DISTANCE) {
						Serial.println("CHECK FAILED: Distance exceeds 5cm threshold (Item Removed).");
						checkFailed = true;
						break; // End monitoring early
					}
				} else {
					// Check for "no valid reading" (Item removed or sensor error)
					if ((millis() - startTime) % 1000 < POLLING_INTERVAL_MS * 2) { 
						Serial.println("CHECK FAILED: No valid reading received (Item Removed).");
					}
					checkFailed = true;
					break; // End monitoring early
				}
			}
		}
		
		// If the check failed, the item was removed at some point.
		if (checkFailed) {
			*currentBorrowCount = 0; // Item is now considered returned/available
			Serial.print(color);
			Serial.println(" borrow count decreased to 0 due to failed check.");
		} else {
			// Check succeeded (item remained present for 5s)
			Serial.println("CHECK SUCCESSFUL: Item remained present. No change in status.");
		}
	}

	// 3. Update the LED to reflect the FINAL borrow status (0=OFF, 1=ON)
	if (*currentBorrowCount == 1) {
		digitalWrite(ledPin, HIGH); 
	} else {
		digitalWrite(ledPin, LOW);
	}

	// --- DEBUGGING OUTPUT ---
	Serial.print("Sequence FINISHED. Final ");
	Serial.print(color);
	Serial.print(" Borrow Count: ");
	Serial.println(*currentBorrowCount);
}

// Function to get color name from RGB values
String getColorName(uint16_t r, uint16_t g, uint16_t b) {
	// Check for minimum brightness
	if (r < 50 && g < 50 && b < 50) {
		return "NO OBJECT";
	}

	// Calculate percentages
	float sum = r + g + b;
	float redPercent = (r / sum) * 100;
	float greenPercent = (g / sum) * 100;
	float bluePercent = (b / sum) * 100;

	// Debug output
	Serial.print("Raw - R: "); Serial.print(r);
	Serial.print(" G: "); Serial.print(g);
	Serial.print(" B: "); Serial.println(b);
	
	Serial.print("Percentages - R: "); Serial.print(redPercent, 1);
	Serial.print("% G: "); Serial.print(greenPercent, 1);
	Serial.print("% B: "); Serial.print(bluePercent, 1);
	Serial.println("%");

	// Simplified color detection: Red requires > 35%, Blue now also requires > 35%
	if (redPercent > 35 && redPercent > greenPercent && redPercent > bluePercent) {
		return "RED";
	}
	// --- BLUE THRESHOLD INCREASED to 35% ---
	if (bluePercent > 35 && bluePercent > redPercent && bluePercent > greenPercent) {
		return "BLUE";
	}
	return "OTHER"; 	
}

void setup() {
	Serial.begin(9600);
	Serial.println("Starting up...");
	
	// Set motor speed (RPM) - Only needs to be done once in setup
	myStepper.setSpeed(10);
	
	// Initialize ultrasonic sensor pins
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	digitalWrite(TRIG_PIN, LOW);
	
	// Initialize LED pins
	pinMode(RED_LED_PIN, OUTPUT);
	pinMode(BLUE_LED_PIN, OUTPUT);
	
	// Set initial LED states based on initial borrow counts (which are 0)
	digitalWrite(RED_LED_PIN, redBorrow == 1 ? HIGH : LOW); 
	digitalWrite(BLUE_LED_PIN, blueBorrow == 1 ? HIGH : LOW);
	
	// Initialize color sensor
	if (!tcs.begin()) {
		Serial.println("Error: TCS34725 not found");
		while (1);
	}
	Serial.println("Found TCS34725 sensor");
	Serial.println("System ready! (Set Serial Monitor to 9600 baud)");
	
	// Initial borrow counts
	Serial.println("Initial borrow counts:");
	Serial.print("Red: "); Serial.println(redBorrow);
	Serial.print("Blue: "); Serial.println(blueBorrow);
}

void loop() {
	// Read color sensor
	uint16_t clear, red, green, blue;
	tcs.getRawData(&red, &green, &blue, &clear);
	
	String color = getColorName(red, green, blue);
	Serial.print("\nDetected Color: ");
	Serial.println(color);
	
	// Handle red or blue detection
	if (color == "RED" || color == "BLUE") {
		// Only proceed if an object is present (not just NO OBJECT)
		handleBorrowSequence(color);
		
		// --- Post-sequence buffer to prevent double-scanning ---
		Serial.print("Sequence complete. Waiting buffer period (");
		Serial.print(POST_SEQUENCE_BUFFER_MS / 1000);
		Serial.println("s) before scanning again.");
		delay(POST_SEQUENCE_BUFFER_MS); 
	}
	
	// Small delay before next reading (only 500ms if no sequence ran)
	delay(500);
}
