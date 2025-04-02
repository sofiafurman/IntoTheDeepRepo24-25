package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Lopez Expo Teleop", group="Linear OpMode")
//@Disabled
public class LopezExpo_Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor slideVertical;
    private DcMotor wristMotor;
    private Servo   outtakeServo;
    private Servo   lights;

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        double speed = 0.3;
        boolean keyA = false;

        double C_LATERAL, C_AXIAL, C_YAW;
        boolean C_OUT;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        lights = hardwareMap.get(Servo.class, "light_strip");
        outtakeServo   = hardwareMap.get(Servo.class, "outtake_servo");
        final double OUT_SERVO_DOWN_POS = 0.7;
        final double OUT_SERVO_UP_POS   = 0.06;
        final double SERVO_SPEED        = -5;
        double outtakeServoPosition     = OUT_SERVO_DOWN_POS;

        slideVertical = hardwareMap.get(DcMotor.class, "vertical_slide");
        slideVertical.setDirection(DcMotor.Direction.FORWARD);
        slideVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertical.setTargetPosition(0);
        slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideVertical.setPower(1.0);

        wristMotor = hardwareMap.get(DcMotor.class, "wrist_drive");
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(25);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(0.5);

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
            /*
             * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
             * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
             * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
             */
            C_AXIAL   = gamepad1.left_stick_y;
            C_LATERAL = gamepad1.left_stick_x;
            C_YAW     = gamepad1.right_stick_x;
            C_OUT     = gamepad1.a;

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

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * speed);
            rightFrontDrive.setPower(rightFrontPower * speed);
            leftBackDrive.setPower(leftBackPower * speed);
            rightBackDrive.setPower(rightBackPower * speed);

            // OUTTAKE CONTROLS
            if (C_OUT && (Math.abs(C_AXIAL)<0.2 && Math.abs(C_LATERAL)<0.2 && Math.abs(C_YAW)<0.2)) {
                slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (slideVertical.getCurrentPosition() < 1500) {
                    slideVertical.setPower(0.75);
                    slideVertical.setTargetPosition(2000);
                }
                while (outtakeServoPosition > OUT_SERVO_UP_POS) {
                    outtakeServoPosition += SERVO_SPEED;
                    if (outtakeServoPosition <= OUT_SERVO_UP_POS) {
                        outtakeServoPosition = OUT_SERVO_UP_POS;
                    }
                    outtakeServo.setPosition(outtakeServoPosition);
                    sleep(15);
                }
                sleep(1000);
                while (outtakeServoPosition < OUT_SERVO_DOWN_POS) {
                    outtakeServoPosition -= SERVO_SPEED;
                    if (outtakeServoPosition >= OUT_SERVO_DOWN_POS) {
                        outtakeServoPosition = OUT_SERVO_DOWN_POS;
                    }
                    outtakeServo.setPosition(outtakeServoPosition);
                    sleep(15);
                }
                sleep(1000);
                slideVertical.setPower(0.75);
                slideVertical.setTargetPosition(78);
                if (keyA == false) {
                    keyA = true;
                }
            } else {
                keyA = false;
            }
            slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeServo.setPosition(outtakeServoPosition);

            if (true) {
                lights.setPosition(0.695); //yellow
            } else if (false) {
                lights.setPosition(0.335); //red fire
            } else {
                lights.setPosition(0.27); //blue ray
            }
            wristMotor.setPower(0.7);
            wristMotor.setTargetPosition(15);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time:" + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed, rightFrontPower * speed);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed, rightBackPower * speed);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Servo Position", "%4.2f", outtakeServoPosition);
            telemetry.addData("Servo Position", "%4.2f", outtakeServo.getPosition());
            telemetry.addData("Wrist Position", "%4.2f", (double)wristMotor.getCurrentPosition()); //??
            telemetry.addData("Wrist Power", "%4.2f", wristMotor.getPower()); //??
            telemetry.addData("vSlidePos", slideVertical.getCurrentPosition());

            telemetry.update();

            sleep(CYCLE_MS);

            if (!opModeIsActive()) {
                slideVertical.setTargetPosition(0);
                wristMotor.setPower(0.2);
                wristMotor.setTargetPosition(0);
            }
        }
    }
}
