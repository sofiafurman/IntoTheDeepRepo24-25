package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Blinker.*;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp(name="LED_Test", group="Linear OpMode")
public class LED_Test extends LinearOpMode{

    private Servo blinkin;
    private Step val = new Step();
    BlinkinPattern list;

    public void runOpMode() {
        blinkin = hardwareMap.get(Servo.class, "blinkuh");

        while (opModeIsActive()) {
            blinkin.setPosition(0.5);
            sleep(500);
            blinkin.setPosition(0.25);
            sleep(500);
            blinkin.setPosition(0.75);
            sleep(500);
        }
    }
}
