package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Blinker.*;

@TeleOp(name="LED_Test", group="Linear OpMode")
public class LED_Test extends LinearOpMode{

    private Servo blinkin;
    private double blinkPatt = 0.22;

    public void runOpMode() {

        blinkin = hardwareMap.get(Servo.class, "blinkuh");
        waitForStart();

        while (opModeIsActive()) {
            blinkin.setPosition(blinkPatt);
            if (gamepad1.a && blinkPatt < 0.78) {
                blinkPatt += 0.005;
                sleep(250);
            } else if (gamepad1.b && blinkPatt > 0.23) {
                blinkPatt -= 0.005;
                sleep(250);
            }
            while (gamepad1.y && opModeIsActive()) {
                blinkin.setPosition(0.335);
                sleep(3000);
                blinkin.setPosition(0.405);
                sleep(3000);
                blinkin.setPosition(0.27);
                sleep(3000);
            }
            telemetry.addData("blinkPatt:", blinkPatt);
            telemetry.update();
        }
    }
}
