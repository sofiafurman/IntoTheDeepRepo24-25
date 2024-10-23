package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Tester", group = "Linear OpMode")
//@Disabled
public class ServoTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Servo servo;

    @Override

    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");
        final int    CYCLE_MS = 50;
        final double SERVO_MIN_POS = 0.0;
        final double SERVO_MAX_POS = 1.0;
        final double SERVO_SPEED = 0.01;
        double servoPos = servo.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                servoPos += SERVO_SPEED;
                if (servoPos >= SERVO_MAX_POS) {
                    servoPos = SERVO_MAX_POS;
                }
            } else if (gamepad1.b) {
                servoPos -= SERVO_SPEED;
                if (servoPos <= SERVO_MIN_POS) {
                    servoPos = SERVO_MIN_POS;
                }
            }

            servo.setPosition(servoPos);

            telemetry.addData("KEYA: ", "%1b", gamepad1.a);
            telemetry.addData("KEYB: ", "%1b", gamepad1.b);
            telemetry.addData("Intended Servo Position: ", "%4.2f", servoPos);
            telemetry.addData("Actual Servo Position:   ", "%4.2f", servo.getPosition());
            telemetry.update();

            sleep(CYCLE_MS);
        }
    }
}