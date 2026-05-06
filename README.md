# Smart Library Item Tracker

Arduino-based tracking system that monitors borrowing and returning of items using color detection and proximity verification.

## How It Works

### Detection Phase
The TCS34725 color sensor continuously scans for colored objects (red or blue tokens). When detected, the system determines the appropriate action based on current borrow status.

### Borrow Logic (Two-State System)

**State 0 (Available):**
- When a colored token is detected, the stepper motor rotates 36° (dispensing action)
- System transitions to State 1 (borrowed)
- Corresponding LED turns ON

**State 1 (Borrowed):**
- When the same color token is detected again, enters 5-second verification mode
- LED blinks during verification
- Ultrasonic sensor polls every 5ms to confirm object remains within 5cm
- If object stays present for full 5 seconds → remains State 1 (check passed)
- If object removed during check → transitions to State 0 (returned)
- LED reflects final state

### Key Parameters
- `MAX_DISTANCE`: 5cm threshold for object detection
- `MONITORING_TIME_MS`: 5000ms verification window
- `POLLING_INTERVAL_MS`: 5ms ultrasonic sampling rate
- `BLINK_INTERVAL_MS`: 250ms LED toggle for visual feedback
- `POST_SEQUENCE_BUFFER_MS`: 3000ms cooldown to prevent double-scanning

### Color Detection Algorithm
RGB values from TCS34725 are converted to percentages. Color classification requires:
- Red: >35% red component, dominant over green/blue
- Blue: >35% blue component, dominant over red/green
- Minimum brightness threshold to reject "no object" readings

## Hardware
- Arduino Uno/Nano
- TCS34725 RGB color sensor (I2C)
- HC-SR04 ultrasonic sensor
- 28BYJ-48 stepper motor + ULN2003 driver (2048 steps/revolution)
- 2× status LEDs

## Pin Assignments
- Stepper: Pins 2, 3, 4, 5
- Ultrasonic: Trig=9, Echo=10
- LEDs: Red=13, Blue=12
- Color sensor: I2C (SDA/SCL)
