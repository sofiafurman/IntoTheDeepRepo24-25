package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    double quarter = 92.5;
    double tile = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0); //first tile, end zone
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

           Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(10, 10), Math.PI / 2)
                        .splineTo(new Vector2d(0, 20), Math.PI)
                        .build());
                       /* .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .strafeTo(new Vector2d(28, 0))
                        .strafeTo(new Vector2d(28 * 2, 0))
                        .turn(Math.toRadians(95))
                        .turn(Math.toRadians(95))
                        .strafeTo(new Vector2d(28, -0))
                        //.lineToY(tile)
                        .build());*/
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(10, 10), Math.PI / 2)
                            .splineTo(new Vector2d(0, 20), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
