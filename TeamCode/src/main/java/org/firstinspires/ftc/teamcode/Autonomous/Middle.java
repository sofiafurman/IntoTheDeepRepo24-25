package org.firstinspires.ftc.teamcode.Autonomous;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;




@Config @Autonomous(name = "Middle Auto 13", group = "Autonomous")
//Next to red net zone. Once completed, should score one sample to the low basket and drive to the end zone
//Intake is on the front of the robot. Assume the low basket is at 45 degrees
//MUCH OF THIS, ESPECIALLY INTAKE AND OUTTAKE, IS THEORETICAL!!! INTAKE AND OUTTAKE HAVEN'T BEEN IMPLEMENTED AS SUBROUTINES AT TIME OF WRITINGedge
public class Middle extends LinearOpMode{

    double quarter = 92.5; //"90 degree" turn
    double tile = 20; //"24 inches;" one tile

    //mechanism instantiation



    public class slideVertical {

        private DcMotorEx lift;

        public slideVertical(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "vertical_slide"); //config?
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotor.Direction.FORWARD);
            //lift.setTargetPosition(0);
            //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                    lift.setTargetPosition(2512);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                telemetry.addData("vSlidePos", lift.getCurrentPosition());
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1); //og 1
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3500) { //og 2500
                    //true causes the action to return
                    lift.setTargetPosition(3500);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                if (pos > 50) {
                    //true causes the action to return
                    lift.setTargetPosition(30);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
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
                .strafeToConstantHeading(new Vector2d(0, -41), new TranslationalVelConstraint(10));
        TrajectoryActionBuilder park = drive.actionBuilder(sub)
                .strafeToConstantHeading(new Vector2d(58, -60));




        // actions that need to happen on init; for instance, a claw tightening.

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                lift.highLift(),
                                toSub.build()
                        ),
                        lift.liftDown(),
                        park.build()
                )
        );
    }}