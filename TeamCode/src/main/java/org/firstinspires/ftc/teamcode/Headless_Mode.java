package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.acmerobotics.roadrunner.ftc.Encoder;

import java.util.Base64;

@TeleOp(name="Headless Test", group="Linear OpMode")
public class Headless_Mode extends LinearOpMode {

    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;

    private Encoder odo1;
    private Encoder odo2;

    public static double[] rotatePoint(double x, double y, double rotateAngle) {
        double rotateAngleRadians = -rotateAngle / 57.29578;
        double newX = (x)*Math.cos(rotateAngleRadians) - (y)*Math.sin(rotateAngleRadians);
        double newY = (x)*Math.sin(rotateAngleRadians) + (y)*Math.cos(rotateAngleRadians);
        newX = Math.round(newX * 10000) / 10000.0;
        newY = Math.round(newY * 10000) / 10000.0;
        double[] ray = {newX, newY};
        return ray;
    }

    public void runOpMode() {

        double[] ray;

        double trackWidthTicks = 7093.064250763197;
        double wheelDistance = 0.0;
        double prevOdo1Pos, prevOdo2Pos, odo1Diff, odo2Diff;

        double axial, lateral;
        double max;
        double robotHeading = 90;
        double C_LATERAL, C_AXIAL, C_YAW;
        boolean C_RESET;

        odo1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "left_front_drive")));
        odo2 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "right_back_drive")));

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            C_AXIAL   = -gamepad1.left_stick_y;
            C_LATERAL = -gamepad1.left_stick_x;
            C_YAW     = gamepad1.right_stick_x;
            C_RESET   = gamepad1.a;

            prevOdo1Pos = odo1.getPositionAndVelocity().position;
            prevOdo2Pos = odo2.getPositionAndVelocity().position;
            odo1Diff = odo1.getPositionAndVelocity().position - prevOdo1Pos;
            odo2Diff = odo2.getPositionAndVelocity().position - prevOdo2Pos;
            double headingChange = (odo1Diff - odo2Diff) / trackWidthTicks;

            if (C_RESET) {
                robotHeading = 0;
            }
            robotHeading += headingChange;

            ray = rotatePoint(C_AXIAL, C_LATERAL, robotHeading);
            axial = ray[0];
            lateral = ray[1];

            double leftFrontPower  = -axial + lateral + C_YAW;
            double rightFrontPower = -axial - lateral - C_YAW;
            double leftBackPower   = -axial - lateral + C_YAW;
            double rightBackPower  =  axial - lateral + C_YAW;

            max = Math.max(Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.abs(leftBackPower)), Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max; rightFrontPower /= max; leftBackPower /= max; rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

        }
    }
}
