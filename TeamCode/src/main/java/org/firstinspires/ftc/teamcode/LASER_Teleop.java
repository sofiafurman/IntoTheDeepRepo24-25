/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//This is just a test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="LASER Main Teleop", group="Linear OpMode")
//@Disabled
public class LASER_Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;
    private DcMotor slideVertical   = null;
    private DcMotor slideHorizontal = null;
    private DcMotor wristMotor      = null;
    private Servo   outtakeServo;
    private Servo   intakeServo;

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        double speed = 1.0;   // used to manage half speed, defaults to full speed
        boolean invertDir = false;  // used for inverted direction prompts
        int invDir = 1;    // used to activate inverted direction
        boolean keyA = false, keyB = false;    // used for toggle keys

        double C_LATERAL, C_AXIAL, C_YAW, C_HORIZ_SLIDE;
        boolean C_HALF_SPEED, C_INV_DIR, C_OUT_SERVO, C_IN_SERVO_TRANSF, C_INTAKE, C_IN_SERVO_SPIT,
                PREV_C_INTAKE, C_VERT_SLIDE_UP = false, PREV_C_VERT_SLIDE_UP = false, C_VERT_SLIDE_DWN = false, PREV_C_VERT_SLIDE_DWN = false;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        intakeServo  = hardwareMap.get(Servo.class, "intake_servo");
        outtakeServo = hardwareMap.get(Servo.class, "outtake_servo");
        final double OUT_SERVO_DOWN_POS = 0.7;
        final double OUT_SERVO_UP_POS   = 0.12;
        final double SERVO_SPEED        = -1;
        double outtakeServoPosition     = outtakeServo.getPosition();

        slideHorizontal = hardwareMap.get(DcMotor.class, "horizontal_slide");
        slideHorizontal.setDirection(DcMotor.Direction.REVERSE);
        slideHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* slideHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideHorizontal.setTargetPosition(0);
        slideHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideHorizontal.setPower(1.0);
        int hSlidePos = 0; */

        slideVertical = hardwareMap.get(DcMotor.class, "vertical_slide");
        slideVertical.setDirection(DcMotor.Direction.FORWARD);
        slideVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertical.setTargetPosition(0);
        slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideVertical.setPower(1.0);
        int vSlideMotorState = 0;

        wristMotor = hardwareMap.get(DcMotor.class, "wrist_drive");
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(0);
        wristMotor.setPower(0.5);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double targetTime = 0.0;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // KEYBINDS
            C_AXIAL               = gamepad1.left_stick_y;
            C_LATERAL             = gamepad1.left_stick_x;
            C_YAW                 = gamepad1.right_stick_x;
            C_HALF_SPEED          = gamepad1.a;
            C_INV_DIR             = gamepad1.b;
            PREV_C_VERT_SLIDE_UP  = C_VERT_SLIDE_UP;
            C_VERT_SLIDE_UP       = gamepad2.right_bumper;
            PREV_C_VERT_SLIDE_DWN = C_VERT_SLIDE_DWN;
            C_VERT_SLIDE_DWN      = gamepad2.left_bumper;
            C_HORIZ_SLIDE         = gamepad2.left_stick_y;
            C_OUT_SERVO           = gamepad2.b;
            C_IN_SERVO_TRANSF     = gamepad2.y;
            C_IN_SERVO_SPIT       = gamepad2.x;
            C_INTAKE              = gamepad2.a;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -C_AXIAL;  // Note: pushing stick forward gives negative value
            double lateral = -C_LATERAL;
            double yaw     =  C_YAW;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = -axial + lateral + yaw;
            double rightFrontPower = -axial - lateral - yaw;
            double leftBackPower   = -axial - lateral + yaw;
            double rightBackPower  =  axial - lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // HALF SPEED CONTROLS
            if (C_HALF_SPEED) {
                if (keyA == false) {
                    keyA = !keyA;
                    switch ((int)(speed * 10)) {
                        case 10 : speed = 0.5; break;
                        case 5 : speed = 1.0; break;
                    }
                }
            } else {
                keyA = false;
            }

            // INVERTED DIRECTION CONTROLS
            if (C_INV_DIR) {
                if (keyB == false) {
                    keyB = !keyB;
                    invertDir = !invertDir;
                    if (invertDir) {
                        invDir = -1;
                    } else {
                        invDir = 1;
                    }
                }
            } else {
                keyB = false;
            }

            // VERTICAL SLIDE CONTROLS
            // 157 = 1 INCH
            switch (vSlideMotorState) {
                case 0:
                    if (C_VERT_SLIDE_UP && !PREV_C_VERT_SLIDE_UP) {vSlideMotorState = 1;}
                    slideVertical.setTargetPosition(0);
                    break;
                case 1:
                    if (C_VERT_SLIDE_UP && !PREV_C_VERT_SLIDE_UP) {vSlideMotorState = 2;}
                    else if (C_VERT_SLIDE_DWN && !PREV_C_VERT_SLIDE_DWN) {vSlideMotorState = 0;}
                    slideVertical.setTargetPosition(2669);
                    break;
                case 2:
                    if (C_VERT_SLIDE_DWN && !PREV_C_VERT_SLIDE_DWN) {vSlideMotorState = 1;}
                    slideVertical.setTargetPosition(5024);
                    break;
            }

            // HORIZONTAL SLIDE CONTROLS
            /*
            if (hSlidePos < 0) {
                hSlidePos = 0;
                slideHorizontal.setTargetPosition(hSlidePos);
            } else if (hSlidePos > 1000) {
                hSlidePos = 1000;
                slideHorizontal.setTargetPosition(hSlidePos);
            } else {
                hSlidePos += (int)(C_HORIZ_SLIDE * 100);
                slideHorizontal.setTargetPosition(hSlidePos);
            } */
            slideHorizontal.setPower(C_HORIZ_SLIDE);

            // OUTTAKE SERVO CONTROLS
            if (C_OUT_SERVO) {
                outtakeServoPosition += SERVO_SPEED;
                if (outtakeServoPosition <= OUT_SERVO_UP_POS) {
                    outtakeServoPosition = OUT_SERVO_UP_POS;
                }
            } else {
                outtakeServoPosition -= SERVO_SPEED;
                if (outtakeServoPosition >= OUT_SERVO_DOWN_POS) {
                    outtakeServoPosition = OUT_SERVO_DOWN_POS;
                }
            }
            outtakeServo.setPosition(outtakeServoPosition);

            // INTAKE SERVO / TRANSFER CONTROLS
            if (C_IN_SERVO_TRANSF && !C_INTAKE) {
                intakeServo.setPosition(1.0);
            } else if (C_IN_SERVO_SPIT) {
                intakeServo.setPosition(0.0);
            } else {
                intakeServo.setPosition(0.5);
            }

            // INTAKE WRIST CONTROLS
            if (C_INTAKE) {
                wristMotor.setPower(0.5);
                wristMotor.setTargetPosition(710);
                intakeServo.setPosition(1.0);
            } else if (slideVertical.isBusy() && slideVertical.getCurrentPosition() < 1500) {
                wristMotor.setPower(0.2);
                wristMotor.setTargetPosition(130);
            } else {
                wristMotor.setPower(0.7);
                wristMotor.setTargetPosition(25);
                if (!C_IN_SERVO_TRANSF && !C_IN_SERVO_SPIT) {
                    intakeServo.setPosition(0.5);
                }
            }
            /*
            if (C_INTAKE) {
                wristMotor.setTargetPosition(100);
                intakeServo.setPosition(1.0);
                wristMotor.setPower(0.4);
                if (wristMotor.getCurrentPosition() > wristMotor.getTargetPosition()) {
                    wristMotor.setPower(0.0);
                }
            } else {
                wristMotor.setTargetPosition(0);
                intakeServo.setPosition(0.5);
                wristMotor.setPower(0.5);
                if (wristMotor.getCurrentPosition() < wristMotor.getTargetPosition()) {
                    wristMotor.setPower(0.0);
                }
            }
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            leftBackDrive.setPower(leftBackPower * speed * invDir);
            rightBackDrive.setPower(rightBackPower * speed * invDir);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time:" + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);
            telemetry.addData("Servo Position", "%4.2f", outtakeServoPosition);
            telemetry.addData("Servo Position", "%4.2f", outtakeServo.getPosition());
            telemetry.addData("Wrist Position", "%4.2f", (double)wristMotor.getCurrentPosition()); //??
            telemetry.addData("Wrist Power", "%4.2f", wristMotor.getPower()); //??
            telemetry.addData("Slide Level", vSlideMotorState);
            telemetry.addData("vSlidePos", slideVertical.getCurrentPosition());
            telemetry.addData("hSlidePower", slideHorizontal.getPowerFloat());
            //telemetry.addData("hSlidePos", hSlidePos);
            //telemetry.addData("hSlidePos", slideHorizontal.getCurrentPosition());

            telemetry.update();

            PREV_C_INTAKE = C_INTAKE;

            sleep(CYCLE_MS);
        }
    }
}
