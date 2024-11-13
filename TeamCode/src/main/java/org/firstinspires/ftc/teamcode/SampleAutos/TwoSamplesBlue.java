package org.firstinspires.ftc.teamcode.SampleAutos;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "Scrimmage Blue", group = "Autonomous")

//Back left wheel touching edge of red net zone tape (tape fully visible)
public class TwoSamplesBlue extends LinearOpMode{
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;
    //variable


    @Override
    public void runOpMode() throws InterruptedException {
        //instantiation
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");



        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        waitForStart();

        leftFrontDrive.setPower(0.3);
        leftBackDrive.setPower(0.3);
        rightFrontDrive.setPower(0.3);
        rightBackDrive.setPower(0.3);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToX(48)
                        .setTangent(Math.PI / 2)
                        .lineToY(10)
                        .setTangent(0)
                        .build());
        //FACING BLUE: LEFT IS LESS, RIGHT IS MORE

        leftFrontDrive.setPower(0.1);
        leftBackDrive.setPower(0.1);
        rightFrontDrive.setPower(0.1);
        rightBackDrive.setPower(0.1);

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .setTangent(0)
                        .lineToX(2)
                        .setTangent(-Math.PI / 2)
                        .lineToY(-140)
                        .build());
    }
}
