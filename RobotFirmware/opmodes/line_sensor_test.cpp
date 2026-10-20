#include "NRL.h"

// ============================================================================
// 1. PIN DEFINITIONS & HARDWARE DECLARATIONS
// ============================================================================
// Connections:
//   S0  -> DIGITAL_5 (GPIO 35, Digital Port 1 - D5)
//   S1  -> DIGITAL_6 (GPIO 36, Digital Port 2 - D6)
//   S2  -> DIGITAL_7 (GPIO 37, Digital Port 2 - D7)
//   Sig -> DIGITAL_4 (GPIO 16, Digital Port 2 - D4)
//   VCC -> 3.3V (Recommended for ESP32-S3 logic compatibility)
//   GND -> GND
// ============================================================================

constexpr int PIN_LINE_S0  = DIGITAL_5; // Address bit 0
constexpr int PIN_LINE_S1  = DIGITAL_6; // Address bit 1
constexpr int PIN_LINE_S2  = DIGITAL_7; // Address bit 2
constexpr int PIN_LINE_SIG = DIGITAL_4; // Mux signal output

// Drive Motors
static HexaDCMotor leftMotor {
    { .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true }
};

static HexaDCMotor rightMotor {
    { .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = false }
};

static TankDrive drive(leftMotor, rightMotor);

// Optional Gripper Servos (at mechanical zero = 90 deg)
static HexaServo servo1 {
    { .signalPin = SERVO_1, .startAngle = 90.0f, .settleMs = 0 }
};

static HexaServo servo2 {
    { .signalPin = SERVO_2, .startAngle = 90.0f, .settleMs = 0 }
};

// ============================================================================
// 2. 8-CHANNEL LINE SENSOR DRIVER CLASS
// ============================================================================

class LineSensor8 {
public:
    int raw[8] = {0}; // Sensor readings: index 0 (far-left) to 7 (far-right)

    void begin() {
        pinMode(PIN_LINE_S0, OUTPUT);
        pinMode(PIN_LINE_S1, OUTPUT);
        pinMode(PIN_LINE_S2, OUTPUT);
        pinMode(PIN_LINE_SIG, INPUT);
    }

    // Read all 8 channels by scanning through multiplexer addresses 0 to 7
    void read() {
        for (int ch = 0; ch < 8; ch++) {
            // Set 3-bit binary address on select lines
            digitalWrite(PIN_LINE_S0, (ch & 0x01) ? HIGH : LOW);
            digitalWrite(PIN_LINE_S1, (ch & 0x02) ? HIGH : LOW);
            digitalWrite(PIN_LINE_S2, (ch & 0x04) ? HIGH : LOW);

            // Allow the multiplexer circuit to settle (5 microseconds)
            delayMicroseconds(5);

            // Read the signal line for this channel
            raw[ch] = digitalRead(PIN_LINE_SIG);
        }
    }

    /**
     * Calculates the error / offset of the line relative to the center of the sensor.
     * Weights: -3.5 (far left, ch 0) to +3.5 (far right, ch 7).
     * @param lineIsHigh True if detecting line reads HIGH (1); False if line reads LOW (0).
     * @param hasLine Output boolean indicating if any sensor currently sees the line.
     * @return Error offset between -3.5 and +3.5 (0.0 means centered).
     */
    float getLineError(bool lineIsHigh, bool &hasLine) {
        float weightedSum = 0.0f;
        int activeCount = 0;

        for (int i = 0; i < 8; i++) {
            bool onLine = lineIsHigh ? (raw[i] == 1) : (raw[i] == 0);
            if (onLine) {
                float weight = (float)i - 3.5f; // [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]
                weightedSum += weight;
                activeCount++;
            }
        }

        if (activeCount == 0) {
            hasLine = false;
            return 0.0f;
        }

        hasLine = true;
        return weightedSum / (float)activeCount;
    }

    // Formats a visual binary string for controller telemetry (e.g., "00110000")
    void formatTelemetry(char* outBuf, size_t bufSize) {
        snprintf(outBuf, bufSize, "%d%d%d%d%d%d%d%d",
                 raw[0], raw[1], raw[2], raw[3],
                 raw[4], raw[5], raw[6], raw[7]);
    }
};

static LineSensor8 lineSensor;

// ============================================================================
// 3. OPMODE IMPLEMENTATION
// ============================================================================

class LineSensorOpMode : public NRLOpMode {
private:
    char sensorDisplay[12] = {0};

    // Polarity: set to true if line gives 1, false if line gives 0
    bool lineIsActiveHigh = true;

    // Line Following Tuning Parameters
    const float BASE_SPEED = 0.40f; // 40% forward speed when tracking
    const float KP         = 0.15f; // Steering proportional gain (adjust if under/over steering)

    // Last known error for keeping turn direction if line is momentarily lost
    float lastError = 0.0f;

public:
    void init() override {
        // Initialize motors, servos and line sensor
        leftMotor.begin();
        rightMotor.begin();
        servo1.begin();
        servo2.begin();

        lineSensor.begin();

        drive.setScale(1.0f);
        telemetry.addData("Status", "LineOp Ready");
    }

    void loop() override {
        // --------------------------------------------------------------------
        // 1. READ LINE SENSOR
        // --------------------------------------------------------------------
        lineSensor.read();
        lineSensor.formatTelemetry(sensorDisplay, sizeof(sensorDisplay));

        bool lineDetected = false;
        float error = lineSensor.getLineError(lineIsActiveHigh, lineDetected);
        if (lineDetected) {
            lastError = error;
        }

        // --------------------------------------------------------------------
        // 2. TOGGLE SENSOR POLARITY WITH BUTTON Y
        // --------------------------------------------------------------------
        // If your sensor outputs LOW on the black line instead of HIGH,
        // press Y on the controller to flip polarity in real time.
        if (gamepad1.justPressed(BTN_Y)) {
            lineIsActiveHigh = !lineIsActiveHigh;
        }

        // --------------------------------------------------------------------
        // 3. DRIVE / LINE FOLLOWING
        // --------------------------------------------------------------------
        // Hold Right Trigger (RT) or Button A to enable Auto Line Following.
        // Release to drive manually with Joysticks.
        bool autoFollow = gamepad1.pressed(BTN_RT) || gamepad1.pressed(BTN_A);

        if (autoFollow) {
            if (lineDetected) {
                // Proportional steering towards line center
                float turn = error * KP;
                drive.drive(BASE_SPEED, turn);
                telemetry.addData("Mode", "Tracking Line");
            } else {
                // Line lost: spin gently towards the side it was last seen
                float recoveryTurn = (lastError > 0.0f) ? 0.25f : -0.25f;
                drive.drive(0.15f, recoveryTurn);
                telemetry.addData("Mode", "Searching Line");
            }
        } else {
            // Manual Driving (Left stick: Forward/Back, Right stick: Turn)
            float forward = gamepad1.leftY();
            float turn    = gamepad1.rightX();
            drive.drive(forward, turn);
            telemetry.addData("Mode", "Manual (Hold RT)");
        }

        // --------------------------------------------------------------------
        // 4. TELEMETRY DISPLAY
        // --------------------------------------------------------------------
        telemetry.addData("Sensors", sensorDisplay); // e.g. "00110000"
        telemetry.addData("Err", error);
        telemetry.addData("Polarity", lineIsActiveHigh ? "HIGH=Line" : "LOW=Line");
    }

    void stop() override {
        drive.stop();
    }
};

// Register this OpMode so it appears on the Controller UI
REGISTER_OPMODE(LineSensorOpMode, "Line Follower", TELEOP);