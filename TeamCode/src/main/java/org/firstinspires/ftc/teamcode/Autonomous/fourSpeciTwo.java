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



//this is theoretical if everything is perfectly tuned, but using 90 degrees and 24 inches
@Config
@Autonomous(name = "4speci attempt 2", group = "Autonomous")
//Next to red net zone. Once completed, should score one sample to the low basket and drive to the end zone
//Intake is on the front of the robot. Assume the low basket is at 45 degrees
//MUCH OF THIS, ESPECIALLY INTAKE AND OUTTAKE, IS THEORETICAL!!! INTAKE AND OUTTAKE HAVEN'T BEEN IMPLEMENTED AS SUBROUTINES AT TIME OF WRITINGedge
public class fourSpeciTwo extends LinearOpMode{

    // if odometry is not properly tuned or constantly being retuned:
    // you MIGHT find it useful to change these values and use multiples of them instead of direct number
    // keep in mind that this may not work well
    // i.e. if something is wrong with acceleration/deceleration, two lengths may not be equal to 2 * (one length)
    double quarter = 90; // "90 degrees" / right angle turn
    double tile = 24; // "24 inches" / one tile

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
                if (pos < 2750) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2750);
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

        public class StartHighLift implements Action{
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
                if (pos < 1000) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(1000);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action startHighLift(){
            return new StartHighLift();
        }
        public class StartLiftDown implements Action{
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
                if (pos > 1500) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(1500);
                    return true;
                } else {
                    //false stops action rerun

                    return false;
                }
            }

        }

        public Action startLiftDown(){
            return new StartLiftDown();
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
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(0);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
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

        //Pose2d initPose = new Pose2d(0, -72, Math.toRadians(270)); put this back in
        Pose2d backingPose = new Pose2d(0, -50, Math.toRadians(270));
        Pose2d initPose = new Pose2d(0, -72, Math.toRadians(270));
        Pose2d sub = new Pose2d(0, -41, Math.toRadians(270));
        Pose2d endZone = new Pose2d(38, -75, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        slideVertical lift = new slideVertical(hardwareMap);

        TrajectoryActionBuilder toSub = drive.actionBuilder(initPose)
                .strafeToConstantHeading(new Vector2d(0, -41.5)); //28 for 2, 1  for 14?
        TrajectoryActionBuilder back = drive.actionBuilder(sub)
                .strafeToConstantHeading(new Vector2d(0, -50)); //28 for 2, 1  for 14?

        TrajectoryActionBuilder push3noHeadingKISS = drive.actionBuilder(backingPose)
                // GUI: KISS - 1 spec 3 push NH

                // NOTE:
                    // this will not actually work since heading stays fixed
                    // use for major path changes only
                    // do not use for minor path changes b/c turning may cause... unwanted collisions

                // PART 1: score 1st spec & prepare
                //.splineToConstantHeading(new Vector2d(0, -41), Math.toRadians(90)) //2 // 1 spec put this back in
                //.splineToConstantHeading(new Vector2d(0, -50), Math.toRadians(90)) //3 // move back to avoid sub
                //.splineToConstantHeading(new Vector2d(30, -40), Math.toRadians(90)) //4 // move to samp area
                //.splineToConstantHeading(new Vector2d(30, -24), Math.toRadians(90)) //5 // move to be in line with samps

                .splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, -26), Math.toRadians(90))

                //PART 2a: 1st push
                .splineToConstantHeading(new Vector2d(39, -26), Math.toRadians(270)) //6 // position
                .splineToConstantHeading(new Vector2d(39, -55), Math.toRadians(90)) //7 // push // y originally -67
                .splineToConstantHeading(new Vector2d(39, -26), Math.toRadians(90)) //8 // back to samp area

                //PART 2b: 2nd push
                .splineToConstantHeading(new Vector2d(48, -26), Math.toRadians(270)) //9 // position
                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(270)) //10 // push

                //.splineToConstantHeading(new Vector2d(48, -26), Math.toRadians(90)) //11 // back to samp area

                .splineToConstantHeading(new Vector2d(43, -60), Math.toRadians(180)) //14 // quarter circle 1
                .splineToConstantHeading(new Vector2d(38, -65), Math.toRadians(270)) //15 // quarter circle 2
                .strafeToConstantHeading(new Vector2d(32, -73.5), new TranslationalVelConstraint(10.0));

                /*
                //PART 2c: 3rd push
                .splineToConstantHeading(new Vector2d(54, -24), Math.toRadians(270)) //12 // position
                .splineToConstantHeading(new Vector2d(54, -55), Math.toRadians(270)) //13 // push

                //PART 3: pick up 2nd spec
                .splineToConstantHeading(new Vector2d(48, -60), Math.toRadians(180)) //14 // quarter circle 1
                .splineToConstantHeading(new Vector2d(38, -67), Math.toRadians(270)); //15 // quarter circle 2
                */



        TrajectoryActionBuilder score3 = drive.actionBuilder(endZone)
                .strafeToConstantHeading(new Vector2d(38, -70)) // TODO: change to left. currently 38 maybe 28 instead
                .strafeToLinearHeading(new Vector2d(3, -42), Math.toRadians(270));
               /* .splineToSplineHeading(new Pose2d(48, -20, Math.toRadians(90)), Math.toRadians(90)) //position by first sample

                .splineToConstantHeading(new Vector2d(52, -24), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(52, -72)) //first sample pushed
                .strafeToConstantHeading(new Vector2d(52, -20))

                .strafeToConstantHeading(new Vector2d(56, -24))
                .strafeToConstantHeading(new Vector2d(56, -72)) //second sample pushed
                .strafeToConstantHeading(new Vector2d(56, -20))

                .splineToConstantHeading(new Vector2d(60, -24), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(60, -72)) //third sample pushed

                .splineToConstantHeading(new Vector2d(49, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(38, -72), Math.toRadians(270));*/







        // actions that need to happen on init; for instance, a claw tightening.


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //toSub.build()
                        new ParallelAction(
                                lift.highLift(),
                                toSub.build()
                        ),
                        lift.startLiftDown(),
                        new ParallelAction(
                                lift.liftDown(),
                                back.build()
                        ),
                        push3noHeadingKISS.build(),
                        lift.startHighLift(),
                        //score3.build()
                        new ParallelAction(
                                lift.highLift(),
                                score3.build()
                        )



                )

        );



    }
}