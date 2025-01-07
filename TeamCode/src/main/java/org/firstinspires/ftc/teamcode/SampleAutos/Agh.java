package org.firstinspires.ftc.teamcode.SampleAutos;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;




@Config
@Autonomous(name = "slayyyyyyyyayayayaya", group = "Autonomous")
//Next to red net zone. Once completed, should score one sample to the low basket and drive to the end zone
//Intake is on the front of the robot. Assume the low basket is at 45 degrees
//MUCH OF THIS, ESPECIALLY INTAKE AND OUTTAKE, IS THEORETICAL!!! INTAKE AND OUTTAKE HAVEN'T BEEN IMPLEMENTED AS SUBROUTINES AT TIME OF WRITINGedge
public class Agh extends LinearOpMode{

    double quarter = 92.5; //"90 degree" turn
    double tile = 20; //"24 inches;" one tile

    //mechanism instantiation



    public class slideVertical {

        private DcMotorEx lift;

        public slideVertical(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "vertical_slide"); //config?
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotor.Direction.FORWARD);
        }

        public class LowLift implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2512) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2512);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action lowLift(){
            return new LowLift();
        }


        public class HighLift implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1); //og 1
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2500) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2500);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action highLift(){
            return new HighLift();
        }



        public class LiftDown implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(0);
                    return true;
                } else {
                    //false stops action rerun

                    return false;
                }
            }

        }

        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class slideHorizontal{
        private DcMotorEx extend;

        public slideHorizontal(HardwareMap hardwareMap) {
            extend = hardwareMap.get(DcMotorEx.class, "horizontal_slide"); //config?
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public class wristDrive{
        private DcMotorEx wrist;

        public wristDrive(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(DcMotorEx.class, "wrist_drive"); //NOT SURE WHAT IT IS IN CONFIG
            wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wrist.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public class intakeServo{
        private Servo spin;

        public intakeServo(HardwareMap hardwaremap){
            spin = hardwareMap.get(Servo.class, "intake_servo"); //NOT SURE WHAT IT IS IN CONFIG
        }
    }




    //begin code
    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d initialPose = new Pose2d(0, -72, Math.toRadians(270));
        Pose2d initialPose = new Pose2d(0, -72, Math.toRadians(3 * quarter));
        Pose2d sub = new Pose2d(0, -41, Math.toRadians(3 * quarter));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        slideVertical lift = new slideVertical(hardwareMap);

        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0, -41)); //28 for 2, 1  for 14?
        TrajectoryActionBuilder toSamp1 = drive.actionBuilder(sub)
                .strafeToConstantHeading(new Vector2d(30, -45))//, new TranslationalVelConstraint(10.0))
                .strafeToConstantHeading(new Vector2d(30, -16))//, new TranslationalVelConstraint(10.0))
                .strafeToConstantHeading(new Vector2d(39, -24), new TranslationalVelConstraint(10.0)) //position in front of first
                .strafeToConstantHeading(new Vector2d(39, -55)) //push first back
                .strafeToConstantHeading(new Vector2d(39, -16))
                .strafeToConstantHeading(new Vector2d(48, -16), new TranslationalVelConstraint(10.0))
                .strafeToConstantHeading(new Vector2d(49, -55)) //push second back
                .strafeToLinearHeading(new Vector2d(45, -35), Math.toRadians(97.5)) //quarter not enough, x og 36
                .strafeToConstantHeading(new Vector2d(45, -72), new TranslationalVelConstraint(10.0));
        /*.strafeToConstantHeading(new Vector2d(49, -60)); //push second back*/
        //.splineToConstantHeading(new Vector2d(40,-41), Math.toRadians(100),new  ,new ProfileAccelConstraint(-5, 5));
        //.strafeToConstantHeading(new Vector2d(0, -72)); //28 for 2, 1  fpor 14?
        Action trajectoryActionCloseOut = toSub.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -72))
                .build();



        // actions that need to happen on init; for instance, a claw tightening.


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                //lift.highLift(),
                                //toSub.build(),
                                //toSub.build(),
                                lift.highLift(),
                                toSub.build()
                                //fwd.build()

                                //trajectoryActionCloseOut
                        ),
                        lift.liftDown(),
                        toSamp1.build()
                )
                //lift.highLift()
        );

    }}