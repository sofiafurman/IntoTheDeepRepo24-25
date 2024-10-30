package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Encoder Test", group = "Test")
//@Disabled
public class Encoder_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(1.0);
            motor.setTargetPosition((int)(1000));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            motor.setTargetPosition((int)(0));
            sleep(1000);
            motor.setPower(0.0);

        }
    }
}
