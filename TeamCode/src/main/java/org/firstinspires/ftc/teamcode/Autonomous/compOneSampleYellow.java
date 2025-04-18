package org.firstinspires.ftc.teamcode.Autonomous;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "1 yellow sample", group = "Autonomous")
@Disabled


//Back left wheel touching edge of red net zone tape (tape fully visible)
public class compOneSampleYellow extends LinearOpMode{
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
                        .lineToX(44)
                        .setTangent(Math.PI / 2)
                        .lineToY(8)
                        .setTangent(0)
                        .lineToX(3.7)
                        .lineToX(20)
                        .setTangent(Math.PI / 2)
                        .lineToY(-80)
                        .setTangent(0)
                        .lineToX(2)
                        .build());



    }
}
