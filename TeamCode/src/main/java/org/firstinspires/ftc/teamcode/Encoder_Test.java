package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Encoder Test", group = "Test")
//@Disabled
public class Encoder_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    boolean C_INPUT = false, PREV_C_INPUT, CONTROL = false;
    int motorState = 0;
    private DcMotor motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            C_INPUT = gamepad1.a;

            if (C_INPUT && !PREV_C_INPUT) {
                switch (motorState) {
                    case 0:
                        motorState = 1;
                        motor.setTargetPosition(0);
                        motor.setPower(0.5);
                        while (motor.isBusy()) {}
                        motor.setPower(0.0);
                        sleep(1000);
                        break;
                    case 1:
                        motorState = 2;
                        motor.setTargetPosition(100);
                        motor.setPower(0.5);
                        while (motor.isBusy()) {}
                        motor.setPower(0.0);
                        sleep(1000);
                        break;
                    case 2:
                        motorState = 0;
                        motor.setTargetPosition(200);
                        motor.setPower(0.5);
                        while (motor.isBusy()) {}
                        motor.setPower(0.0);
                        sleep(1000);
                        break;
                }
            }

            /*
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(1000);
            motor.setPower(1.0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (motor.isBusy()) {}
            motor.setPower(0.0);
            sleep(1000);

            motor.setTargetPosition(0);
            motor.setPower(1.0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (motor.isBusy()) {}
            motor.setPower(0.0);
            sleep(1000);
            */

            telemetry.addData("runtime ", runtime);
            telemetry.addData("motorPos", motor.getCurrentPosition());
            telemetry.addData("C_INPUT", C_INPUT);
            telemetry.addData("PREV_C_INPUT", PREV_C_INPUT);
            telemetry.addData("CONTROL", CONTROL);
            telemetry.update();

            PREV_C_INPUT = C_INPUT;
            CONTROL = !CONTROL;
        }
    }
}
