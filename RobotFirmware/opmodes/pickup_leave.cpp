#include "NRL.h"

// ============================================================================
// 1. HARDWARE DECLARATIONS (BoardPins.h)
// ============================================================================

static HexaDCMotor leftMotor {
    { .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }
};

static HexaDCMotor rightMotor {
    { .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }
};

static TankDrive drive(leftMotor, rightMotor);

static HexaServo armServo {
    { .signalPin = SERVO_1, .startAngle = 30.0f }
};

static HexaServo clawServo {
    { .signalPin = SERVO_2, .startAngle = 70.0f } // Starts OPEN
};


// ============================================================================
// 2. LINE SENSOR PINS
// ============================================================================

const int PIN_SIG = DIGITAL_4;  // GPIO 16 (Sig)
const int PIN_S0  = DIGITAL_5;  // GPIO 35 (S0)
const int PIN_S1  = DIGITAL_6;  // GPIO 36 (S1)
const int PIN_S2  = DIGITAL_7;  // GPIO 37 (S2)


// ============================================================================
// 3. TUNING & SPEED SETTINGS (Tuned for Thin Tape on Red Tiles)
// ============================================================================

const float ARM_UP_ANGLE      = 30.0f;
const float ARM_DOWN_ANGLE    = 98.0f;
const float CLAW_OPEN_ANGLE   = 70.0f;   // Open claw
const float CLAW_CLOSED_ANGLE = 160.0f;  // Closed claw

// Controlled speeds to prevent overshooting thin tape on turns
const float BASE_SPEED        = 0.38f;  
const float MIN_TURN_SPEED    = 0.18f;  
const float SHARP_PIVOT_SPEED = 0.42f;  
const float TURN_180_SPEED    = 0.40f;  

// Responsive PD gains for thin tape
const float KP = 0.18f;
const float KD = 0.10f;

// Timers (in milliseconds)
const unsigned long ARM_DOWN_TIME_MS   = 800;
const unsigned long CLAW_CLOSE_TIME_MS = 600;
const unsigned long ARM_UP_TIME_MS     = 800;
const unsigned long CLAW_OPEN_TIME_MS  = 600;
const unsigned long TURN_BLIND_TIME_MS = 400; // Time to clear current line before searching for 180° re-entry


// ============================================================================
// 4. AUTONOMOUS STATE MACHINE
// ============================================================================

enum AutoStep {
    WAIT_FOR_START,
    STEP_FOLLOW_TO_END,
    STEP_ARM_DOWN,
    STEP_CLAW_CLOSE,
    STEP_ARM_UP,
    STEP_TURN_180_FIND_LINE,
    STEP_FOLLOW_TO_START,
    STEP_CLAW_OPEN,
    STEP_COMPLETED
};


// ============================================================================
// 5. AUTONOMOUS OPMODE CLASS
// ============================================================================

class pickup_leave : public NRLOpMode {
private:
    AutoStep currentStep = WAIT_FOR_START;
    unsigned long stepStartTime = 0;

    float previousError = 0.0f;
    float lastKnownError = 0.0f;
    int lostLineFrames = 0;
    char sensorVisualizer[10] = "........"; // For live telemetry display

    void selectChannel(int ch) {
        digitalWrite(PIN_S0, (ch & 0x01) ? HIGH : LOW);
        digitalWrite(PIN_S1, (ch & 0x02) ? HIGH : LOW);
        digitalWrite(PIN_S2, (ch & 0x04) ? HIGH : LOW);
        delayMicroseconds(6); // Settling delay for 4051 mux
    }

    /**
     * @brief Smart Thin-Tape Detector for Red Tiles
     * Automatically identifies the minority 1-3 sensors that are on the thin black tape,
     * ignoring the 5-7 sensors on the red background tile.
     */
    float readLineError(int &activeCount) {
        bool raw[8];
        int highCount = 0;

        for (int i = 0; i < 8; i++) {
            selectChannel(i);
            raw[i] = (digitalRead(PIN_SIG) == HIGH);
            if (raw[i]) highCount++;
        }

        int lowCount = 8 - highCount;

        // Thin tape logic: the tape covers 1 to 3 sensors at most.
        // The red tile covers the rest (5 to 7 sensors).
        bool tapeLogic = HIGH;
        if (highCount >= 1 && highCount <= 3) {
            tapeLogic = HIGH;
            activeCount = highCount;
        } else if (lowCount >= 1 && lowCount <= 3) {
            tapeLogic = LOW;
            activeCount = lowCount;
        } else {
            // All 8 sensors see the red tile (or off track): NO TAPE DETECTED!
            activeCount = 0;
            strcpy(sensorVisualizer, "........");
            return 0.0f;
        }

        // Build visualizer string and calculate center position
        float weightedSum = 0.0f;
        for (int i = 0; i < 8; i++) {
            if (raw[i] == tapeLogic) {
                sensorVisualizer[i] = 'X';
                weightedSum += (i - 3.5f); // 0..7 mapped to -3.5 (left) .. +3.5 (right)
            } else {
                sensorVisualizer[i] = '.';
            }
        }
        sensorVisualizer[8] = '\0';

        return (weightedSum / (float)activeCount);
    }

    /**
     * @brief Checks if center sensors (error near 0) have re-acquired the line
     */
    bool isCenterOnLine() {
        int count = 0;
        float err = readLineError(count);
        return (count > 0 && abs(err) <= 1.5f);
    }

    void nextStep(AutoStep next) {
        currentStep = next;
        stepStartTime = millis();
        lostLineFrames = 0;
        previousError = 0.0f;
        lastKnownError = 0.0f;
    }

    /**
     * @brief Follows curved/sharp lines; returns true when the line ends
     */
    bool followLineWithTurns() {
        int activeSensors = 0;
        float error = readLineError(activeSensors);

        if (activeSensors > 0) {
            // Line is detected on track
            lostLineFrames = 0;
            lastKnownError = error;

            // PD Steering calculation
            float derivative = error - previousError;
            previousError = error;
            float steer = (KP * error) + (KD * derivative);
            steer = constrain(steer, -0.6f, 0.6f);

            // Adaptive speed: automatically slow down when entering bends
            float curveSeverity = abs(error) / 3.5f;
            float speed = BASE_SPEED - curveSeverity * (BASE_SPEED - MIN_TURN_SPEED);

            drive.drive(-speed, steer);
            return false;

        } else {
            // No sensor sees the thin tape!
            if (lastKnownError < -1.8f) {
                // Slipped off a sharp Left bend -> pivot left to recover
                drive.drive(-MIN_TURN_SPEED * 0.5f, -SHARP_PIVOT_SPEED);
                return false;
            } 
            else if (lastKnownError > 1.8f) {
                // Slipped off a sharp Right bend -> pivot right to recover
                drive.drive(-MIN_TURN_SPEED * 0.5f, SHARP_PIVOT_SPEED);
                return false;
            } 
            else {
                // Line ended while near center -> Stop motors immediately!
                drive.stop();
                lostLineFrames++;

                // Confirm for 6 consecutive frames (~60ms) that tape has truly ended
                if (lostLineFrames >= 6) {
                    drive.stop();
                    return true; // Line ended! Proceed to pickup
                }
                return false;
            }
        }
    }

public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        drive.stop();

        armServo.begin();
        clawServo.begin();
        armServo.setPosition(ARM_UP_ANGLE);
        clawServo.setPosition(CLAW_OPEN_ANGLE);

        pinMode(PIN_SIG, INPUT);
        pinMode(PIN_S0, OUTPUT);
        pinMode(PIN_S1, OUTPUT);
        pinMode(PIN_S2, OUTPUT);

        currentStep = WAIT_FOR_START;
        telemetry.addData("Auto", "Ready (Thin Tape on Red Tile Mode)");
    }

    void start() override {
        nextStep(STEP_FOLLOW_TO_END);
    }

    void loop() override {
        unsigned long elapsedTime = millis() - stepStartTime;

        // Continuously display live sensor bar on controller / OLED: e.g. [..XX....]
        telemetry.addData("Tape Sensor", sensorVisualizer);

        switch (currentStep) {

            // -------------------------------------------------------------
            // Step 1: Follow thin line (and all curves) until it ends
            // -------------------------------------------------------------
            case STEP_FOLLOW_TO_END:
                if (followLineWithTurns()) {
                    drive.stop();
                    nextStep(STEP_ARM_DOWN);
                }
                telemetry.addData("Step", "1: Tracking Line Forward");
                break;

            // -------------------------------------------------------------
            // Step 2: Lower arm
            // -------------------------------------------------------------
            case STEP_ARM_DOWN:
                drive.stop();
                armServo.setPosition(ARM_DOWN_ANGLE);
                if (elapsedTime >= ARM_DOWN_TIME_MS) {
                    nextStep(STEP_CLAW_CLOSE);
                }
                telemetry.addData("Step", "2: Lowering Arm");
                break;

            // -------------------------------------------------------------
            // Step 3: Close claw to grip object
            // -------------------------------------------------------------
            case STEP_CLAW_CLOSE:
                drive.stop();
                clawServo.setPosition(CLAW_CLOSED_ANGLE);
                if (elapsedTime >= CLAW_CLOSE_TIME_MS) {
                    nextStep(STEP_ARM_UP);
                }
                telemetry.addData("Step", "3: Gripping Object");
                break;

            // -------------------------------------------------------------
            // Step 4: Raise arm to carry position
            // -------------------------------------------------------------
            case STEP_ARM_UP:
                drive.stop();
                armServo.setPosition(ARM_UP_ANGLE);
                if (elapsedTime >= ARM_UP_TIME_MS) {
                    nextStep(STEP_TURN_180_FIND_LINE);
                }
                telemetry.addData("Step", "4: Lifting Arm");
                break;

            // -------------------------------------------------------------
            // Step 5: Turn 180° until center sensors lock back onto the tape
            // -------------------------------------------------------------
            case STEP_TURN_180_FIND_LINE:
                drive.drive(0.0f, -TURN_180_SPEED); // Spin in place

                // Must turn for at least 400ms to clear the old orientation
                if (elapsedTime >= TURN_BLIND_TIME_MS) {
                    if (isCenterOnLine()) {
                        drive.stop();
                        nextStep(STEP_FOLLOW_TO_START);
                    }
                }
                telemetry.addData("Step", "5: Turning 180 (Searching Line)");
                break;

            // -------------------------------------------------------------
            // Step 6: Follow the line (and turns) all the way back to start
            // -------------------------------------------------------------
            case STEP_FOLLOW_TO_START:
                if (followLineWithTurns()) {
                    drive.stop();
                    nextStep(STEP_CLAW_OPEN);
                }
                telemetry.addData("Step", "6: Tracking Line Back to Start");
                break;

            // -------------------------------------------------------------
            // Step 7: Open claw to drop the object in start zone
            // -------------------------------------------------------------
            case STEP_CLAW_OPEN:
                drive.stop();
                clawServo.setPosition(CLAW_OPEN_ANGLE);
                if (elapsedTime >= CLAW_OPEN_TIME_MS) {
                    nextStep(STEP_COMPLETED);
                }
                telemetry.addData("Step", "7: Dropping Object");
                break;

            // -------------------------------------------------------------
            // Step 8: Done!
            // -------------------------------------------------------------
            case STEP_COMPLETED:
                drive.stop();
                telemetry.addData("Step", "Mission Complete!");
                break;

            default:
                drive.stop();
                break;
        }
    }

    void stop() override {
        drive.stop();
        armServo.detach();
        clawServo.detach();
    }
};

REGISTER_OPMODE(pickup_leave, "picup_leave", AUTO);
