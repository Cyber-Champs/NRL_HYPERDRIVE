
#include "NRL.h"

// -------------------- DRIVE MOTORS --------------------

static HexaDCMotor leftMotor {
    { .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }
};

static HexaDCMotor rightMotor {
    { .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }
};

static TankDrive drive(leftMotor, rightMotor);


// -------------------- SERVOS --------------------

// Servo 1: Arm
static HexaServo armServo {
    { .signalPin = SERVO_1, .startAngle = 30.0f }
};

// Servo 2: Claw
static HexaServo clawServo {
    { .signalPin = SERVO_2, .startAngle = 70.0f }
};


// -------------------- ARM SETTINGS --------------------

float armAngle = 30.0f;

const float ARM_MIN_ANGLE = 30.0f;
const float ARM_MAX_ANGLE = 100.0f;
const float ARM_STEP = 5.0f;


// -------------------- CLAW SETTINGS --------------------

const float CLAW_CLOSED_ANGLE = 70.0f;
const float CLAW_OPEN_ANGLE = 160.0f;


// -------------------- MAIN OPMODE --------------------

class RobotDrive : public NRLOpMode {

public:

    void init() override {

        // Initialize servos first
        armServo.begin();
        clawServo.begin();

        // Set initial servo positions
        armAngle = ARM_MIN_ANGLE;

        armServo.setPosition(armAngle);
        clawServo.setPosition(CLAW_CLOSED_ANGLE);

        // Initialize drive motors
        leftMotor.begin();
        rightMotor.begin();

        telemetry.addData("Robot", "Initialized");
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Claw", "Closed");
    }


    void loop() override {

        // ---------------- DRIVE CONTROL ----------------

        // Left joystick UP = forward
        // Left joystick DOWN = backward
        // Negative sign corrects the reversed direction
        float forward = -gamepad1.leftY();

        // Right joystick LEFT/RIGHT = turning
        float turn = -gamepad1.rightX();

        // Deadzone to prevent unwanted movement
        if (forward > -0.06f && forward < 0.06f) {
            forward = 0.0f;
        }

        if (turn > -0.06f && turn < 0.06f) {
            turn = 0.0f;
        }

        drive.drive(forward, turn);


        // ---------------- ARM SERVO CONTROL ----------------


        // X button: Set arm to 40 degrees
        if (gamepad1.justPressed(BTN_X)) {
            armAngle = 40.0f;
            armServo.setPosition(armAngle);
        }

        // Y button: Set arm to 100 degrees
        if (gamepad1.justPressed(BTN_Y)) {
            armAngle = 98.0f;
            armServo.setPosition(armAngle);
        }

        // D-pad DOWN: Increase arm angle
        if (gamepad1.justPressed(BTN_DPAD_DOWN)) {

            armAngle += ARM_STEP;

            if (armAngle > ARM_MAX_ANGLE) {
                armAngle = ARM_MAX_ANGLE;
            }

            armServo.setPosition(armAngle);
        }


        // D-pad UP: Decrease arm angle
        if (gamepad1.justPressed(BTN_DPAD_UP)) {

            armAngle -= ARM_STEP;

            if (armAngle < ARM_MIN_ANGLE) {
                armAngle = ARM_MIN_ANGLE;
            }

            armServo.setPosition(armAngle);
        }


        // ---------------- CLAW SERVO CONTROL ----------------

        // D-pad RIGHT: Close claw to 70 degrees
        if (gamepad1.justPressed(BTN_DPAD_RIGHT)) {

            clawServo.setPosition(CLAW_CLOSED_ANGLE);
        }


        // D-pad LEFT: Open claw to 160 degrees
        if (gamepad1.justPressed(BTN_DPAD_LEFT)) {

            clawServo.setPosition(CLAW_OPEN_ANGLE);
        }


        // ---------------- TELEMETRY ----------------

        telemetry.addData("Forward/Backward", forward);
        telemetry.addData("Left/Right", turn);
        telemetry.addData("Arm Angle", armAngle);

    }


    void stop() override {

        // Stop drive motors
        drive.stop();

        // Detach servos
        armServo.detach();
        clawServo.detach();

    }

};


// Register OpMode in controller menu
REGISTER_OPMODE(RobotDrive, "Robot Drive + Arm + Claw", TELEOP);
