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

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;
    private DcMotor slideVertical   = null;
    private DcMotor slideHorizontal = null;
    Servo outtakeServo;

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        double speed = 1.0;   // used to manage half speed, defaults to full speed
        boolean invertDir = false;  // used for inverted direction prompts
        int invDir = 1;    // used to activate inverted direction
        boolean keyA = false, keyB = false;    // used for toggle keys

        double C_LATERAL, C_AXIAL, C_YAW, C_VERT_SLIDE_U, C_DWN_SLIDE_U;
        boolean C_HALF_SPEED, C_INV_DIR, C_VERT_SLIDE_D, C_DWN_SLIDE_D, C_OUT_SERVO;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        slideVertical   = hardwareMap.get(DcMotor.class, "vertical_slide");
        slideHorizontal = hardwareMap.get(DcMotor.class, "horizontal_slide");

        outtakeServo = hardwareMap.get(Servo.class, "outtake_servo");
        final double OUT_SERVO_DOWN_POS = 1.0;
        final double OUT_SERVO_UP_POS   = 0.55;
        final double SERVO_SPEED        = -0.1;
        double outtakeServoPosition     = outtakeServo.getPosition();



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
        slideVertical.setDirection(DcMotor.Direction.FORWARD);
        slideHorizontal.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // KEYBINDS
            C_AXIAL        = gamepad1.left_stick_y;
            C_LATERAL      = gamepad1.left_stick_x;
            C_YAW          = gamepad1.right_stick_x;
            C_HALF_SPEED   = gamepad1.a;
            C_INV_DIR      = gamepad1.b;
            C_VERT_SLIDE_U = gamepad2.right_trigger;
            C_DWN_SLIDE_U  = gamepad2.left_trigger;
            C_VERT_SLIDE_D = gamepad2.right_bumper;
            C_DWN_SLIDE_D  = gamepad2.left_bumper;
            C_OUT_SERVO    = gamepad2.y;

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
                        case 10 : speed = 0.7; break;
                        case 7 : speed = 0.5; break;
                        case 5 : speed = 0.3; break;
                        case 3 : speed = 1.0; break;
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
            slideVertical.setPower(0);
            if (C_VERT_SLIDE_U > 0.1 || C_VERT_SLIDE_U < -0.1) {
                slideVertical.setPower(C_VERT_SLIDE_U);
            } else {
                if (C_VERT_SLIDE_D) {
                    slideVertical.setPower(-1);
                } else {
                    slideVertical.setPower(0.05);
                }
            }

            // HORIZONTAL SLIDE CONTROLS
            slideHorizontal.setPower(0);
            if (C_DWN_SLIDE_U > 0.1 || C_DWN_SLIDE_U < -0.1) {
                slideHorizontal.setPower(C_DWN_SLIDE_U);
            } else {
                if (C_DWN_SLIDE_D) {
                    slideHorizontal.setPower(-1);
                } else {
                    slideHorizontal.setPower(0);
                }
            }

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

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            leftBackDrive.setPower(leftBackPower * speed * invDir);
            rightBackDrive.setPower(rightBackPower * speed * invDir);
            outtakeServo.setPosition(outtakeServoPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);
            telemetry.addData("Servo Position", "%4.2f", outtakeServoPosition);
            telemetry.addData("Servo Position", "%4.2f", outtakeServo.getPosition());
            telemetry.update();

            sleep(CYCLE_MS);
        }
    }
}
