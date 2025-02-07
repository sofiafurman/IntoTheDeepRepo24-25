package org.firstinspires.ftc.teamcode.Autonomous;
import androidx.annotation.NonNull;
import java.util.concurrent.*;
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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.ScheduledExecutorService;


//this is theoretical if everything is perfectly tuned, but using 90 degrees and 24 inches

// weird ahh pseudo code
// semicircle ish spline when going to pick up the samples, just a straight line back??

@Config
@Autonomous(name = "erm what the sigma", group = "Autonomous")
//Next to red net zone. Once completed, should score one sample to the low basket and drive to the end zone
//Intake is on the front of the robot. Assume the low basket is at 45 degrees
//MUCH OF THIS, ESPECIALLY INTAKE AND OUTTAKE, IS THEORETICAL!!! INTAKE AND OUTTAKE HAVEN'T BEEN IMPLEMENTED AS SUBROUTINES AT TIME OF WRITINGedge
public class anotheronethankyou extends LinearOpMode {

    //mechanism instantiation

    public class outtakeServo{
        private Servo rotateOut;

        public outtakeServo(HardwareMap hardwaremap){
            rotateOut = hardwareMap.get(Servo.class, "outtake_servo"); //NOT SURE WHAT IT IS IN CONFIG
            rotateOut.setPosition(0.70);
            //initpos 0.70
            //endpos 0.06
        }

        public class OuttakeIt implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (opModeIsActive()){
                    rotateOut.setPosition(1.0);
                    sleep(3000);
                }
                //powers on motor if not on
                if (!initialized){
                    //rotateOut.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = rotateOut.getPosition();
                packet.put("liftPos", pos);
                if (pos != 0.06) {
                    //true causes the action to return
                    //rotateOut.se;
                    rotateOut.setPosition(0.06);
                    return true;
                } else {
                    //false stops action rerun
                    //rotateOut.setPower(0);
                    //rotateOut.setPosition(0.7);
                    rotateOut.setPosition(0.06);
                    return false;
                }
            }
        }

        public Action outtakeIt() {
            return new outtakeServo.OuttakeIt();
        }


        public class OuttakeHome implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    //rotateOut.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = rotateOut.getPosition();
                packet.put("liftPos", pos);
                if (pos < 0.7) {
                    //true causes the action to return
                    //rotateOut.se;
                    rotateOut.setPosition(0.7);
                    return true;
                } else {
                    //false stops action rerun
                    //rotateOut.setPower(0);
                    //rotateOut.setPosition(0.06);
                    return false;
                }
            }
        }

        public Action outtakeHome() {
            return new outtakeServo.OuttakeHome();
        }


    }

    public class intakeServo {
        private Servo spinny;

        public intakeServo(HardwareMap hardwareMap) {
            spinny = hardwareMap.get(Servo.class, "intake_servo"); // Make sure the configuration matches
            //spinny.setPosition(0.5); // Initialize servo position
        }

        public class IntakeServoPickUp implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            //private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Initialize the servo movement and record the start time
                /*if (!initialized) {
                    spinny.setPosition(1);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }*/
                if (opModeIsActive()){
                    spinny.setPosition(1.0);
                    sleep(3000);
                }
                //spinny.setDirection(Servo.Direction.FORWARD);
                //spinny.setPosition(0.3);
                //spinny.setPosition(1);
                // Check if 3 seconds have elapsed
                /*long elapsedTime = System.currentTimeMillis() - startTime;
                while (elapsedTime <= 3000) {
                    try{
                    spinny.setPosition(1);}
                    catch(NumberFormatException e) {
                        // Stop the servo
                        spinny.setPosition(0);
                    }
                    return true; // Action is complete
                }*/

                // Update telemetry
                packet.put("servoPos", spinny.getPosition());
                return false; // Action is still running
            }

        }
        public Action intakeServoPickUp() {
            return new IntakeServoPickUp();
        }


    }













    public class slideVertical {

        private DcMotorEx lift;

        public slideVertical(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "vertical_slide"); //config?
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class HighLift implements Action{
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
                if (pos < 5024) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(5024);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action highLift(){
            return new slideVertical.HighLift();
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
                if (pos > 0) { //50
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
            return new slideVertical.LiftDown();
        }
    }



    public class slideHorizontal{
        private DcMotorEx extend;

        public slideHorizontal(HardwareMap hardwareMap) {
            extend = hardwareMap.get(DcMotorEx.class, "horizontal_slide"); //config?
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setDirection(DcMotor.Direction.FORWARD);
            extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class HSlideOut implements Action {
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    extend.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = extend.getCurrentPosition();
                packet.put("extendPos", pos);
                if (pos < 300) { //50
                    //true causes the action to return
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setTargetPosition(300);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    extend.setPower(0);
                    return false;
                }
            }
        }
        public Action hSlideOut(){
            return new slideHorizontal.HSlideOut();
        }

        public class HSlideFinish implements Action {
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    extend.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = extend.getCurrentPosition();
                packet.put("extendPos", pos);
                if (pos < 775) { //50
                    //true causes the action to return
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setTargetPosition(775);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    extend.setPower(0);
                    return false;
                }
            }
        }
        public Action hSlideFinish(){
            return new slideHorizontal.HSlideFinish();
        }


        public class HSlideIn implements Action {
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    extend.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = extend.getCurrentPosition();
                packet.put("extendPos", pos);
                if (pos > 0) { //50
                    //true causes the action to return
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setTargetPosition(0);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    extend.setPower(0);
                    return false;
                }
            }


        }

        public Action hSlideIn() {
            return new slideHorizontal.HSlideIn();
        }


    }

    public class wristDrive {


        private DcMotorEx intakeW;

        public wristDrive(HardwareMap hardwareMap) {
            intakeW = hardwareMap.get(DcMotorEx.class, "wrist_drive"); //config?
            intakeW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeW.setDirection(DcMotor.Direction.REVERSE);
            intakeW.setTargetPosition(15);
            intakeW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        public class IntakeAvoid implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    intakeW.setPower(1); //og 1
                    initialized = true;
                }
                //checks lift's current position
                double pos = intakeW.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 130) {
                    //true causes the action to return
                    intakeW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeW.setTargetPosition(130);
                    return true;
                } else {
                    //false stops action rerun
                    //intakeW.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeAvoid(){
            return new wristDrive.IntakeAvoid();
        }


        public class IntakeTransfer implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    intakeW.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = intakeW.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos > 15) { //50
                    //true causes the action to return
                    intakeW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeW.setTargetPosition(15);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    //intakeW.setPower(0);
                    return false;
                }
            }

        }
        public Action intakeTransfer() {
            return new wristDrive.IntakeTransfer();
        }

        public class IntakePickUp implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    intakeW.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = intakeW.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos < 700) { //50
                    //true causes the action to return
                    intakeW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeW.setTargetPosition(700);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    //intakeW.setPower(0);
                    return false;
                }
            }

        }
        public Action intakePickUp() {
            return new wristDrive.IntakePickUp();
        }

    }








    //begin code
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initPose = new Pose2d(-48, -72, Math.toRadians(0));

        Pose2d scoring = new Pose2d(-65, -65, Math.toRadians(45));

        Pose2d first = new Pose2d(-48, -47, Math.toRadians(140));

        Pose2d second = new Pose2d(-58, -46, Math.toRadians(135));

        Pose2d third = new Pose2d(-68, -46, Math.toRadians(135));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        slideVertical lift = new slideVertical(hardwareMap);
        slideHorizontal extend = new slideHorizontal(hardwareMap);
        wristDrive intakeW = new wristDrive(hardwareMap);
        intakeServo spinny = new intakeServo(hardwareMap);
        outtakeServo rotateOut = new outtakeServo(hardwareMap);

        TrajectoryActionBuilder score1 = drive.actionBuilder(initPose)
                .strafeToLinearHeading(new Vector2d(-65, -65), Math.toRadians(45));

        TrajectoryActionBuilder grab2 = drive.actionBuilder(scoring)
                .strafeToLinearHeading(new Vector2d(-46, -46), Math.toRadians(135));

        TrajectoryActionBuilder score2 = drive.actionBuilder(first)
                .strafeToLinearHeading(new Vector2d(-65, -65), Math.toRadians(45));

        TrajectoryActionBuilder grab3 = drive.actionBuilder(scoring)
                .strafeToLinearHeading(new Vector2d(-58, -46), Math.toRadians(135));

        TrajectoryActionBuilder score3 = drive.actionBuilder(second)
                .strafeToLinearHeading(new Vector2d(-65, -65), Math.toRadians(45));

        TrajectoryActionBuilder grab4 = drive.actionBuilder(scoring)
                .strafeToLinearHeading(new Vector2d(-68, -46), Math.toRadians(135), new TranslationalVelConstraint(15.0));

        TrajectoryActionBuilder score4 = drive.actionBuilder(third)
                .strafeToLinearHeading(new Vector2d(-65, -65), Math.toRadians(45));

        //TrajectoryActionBuilder Spline1 = drive.actionBuilder(initPose)
        //.splineToLinearHeading(new Pose2d(-52, -42, Math.toRadians(135)), Math.toRadians(100));
        //.strafeToConstantHeading(new Vector2d(-65, -65));







        // actions that need to happen on init; for instance, a claw tightening.




        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //START WITH INTAKE PUSHED ALL THE WAY BACK!!!!


                        //intakeW.intakePickUp(),
                        //intakeW.intakeAvoid(),

                        //extend.hSlideIn()

                        //intakeW.intakeTransfer()

                        //intakeW.intakePickUp()
                        //intakeServo.IntakeServoStop


                        /*new ParallelAction(
                                score1.build(),
                                lift.highLift()
                        ),
                        new ParallelAction(
                                lift.liftDown(),
                                extend.hSlideOut(),
                                //intakeW.intakePickUp(),
                                //spinny.intakeServoPickUp(),
                                grab2.build()
                        ),
                        new ParallelAction(
                                intakeW.intakePickUp(),
                                extend.hSlideFinish(),
                                spinny.intakeServoPickUp()
                        ),
                        intakeW.intakeTransfer()*/
                        rotateOut.outtakeIt()

                        //new ParallelAction(
                        //spinny.intakeServoPickUp(),
                        //score2.build()
                        )






                        /*score1.build(),
                        grab2.build(),
                        score2.build(),
                        grab3.build(),
                        score3.build(),
                        grab4.build(),
                        score4.build()*/
                //);
        );



    }
}