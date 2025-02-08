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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "dontuse", group = "Autonomous")
@Disabled
//Next to red net zone. Once completed, should score one sample to the low basket and drive to the end zone
//Intake is on the front of the robot. Assume the low basket is at 45 degrees
//MUCH OF THIS, ESPECIALLY INTAKE AND OUTTAKE, IS THEORETICAL!!! INTAKE AND OUTTAKE HAVEN'T BEEN IMPLEMENTED AS SUBROUTINES AT TIME OF WRITINGedge
public class threeSampleShoddy extends LinearOpMode{

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
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        public class FinalLift implements Action{
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
                if (pos < 2850) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2850);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action finalLift(){
            return new FinalLift();
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
                if (pos < 2800) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2800);
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
                if (pos > 1200) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(1200);
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
            return new LiftDown();
        }
    }



    public class slideHorizontal{
        private DcMotorEx extend;

        public slideHorizontal(HardwareMap hardwareMap) {
            extend = hardwareMap.get(DcMotorEx.class, "horizontal_slide"); //config?
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setDirection(DcMotor.Direction.REVERSE);
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
                if (pos < 500) { //50
                    //true causes the action to return
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setTargetPosition(500);
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
            return new HSlideOut();
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

            public Action hSlideIn() {
                return new HSlideIn();
            }
        }


    }

    public class wristDrive{

        private DcMotorEx wrist;

        public wristDrive(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(DcMotorEx.class, "wrist_drive"); //NOT SURE WHAT IT IS IN CONFIG
            wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wrist.setDirection(DcMotor.Direction.REVERSE);
            wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public class IntakeAvoid implements Action {
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    wrist.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = wrist.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos != 130) { //50
                    //true causes the action to return
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setTargetPosition(130);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    wrist.setPower(0);
                    return false;
                }
            }

            public Action intakeAvoid() {
                return new IntakeAvoid();
            }
        }

        public class IntakeTransfer implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    wrist.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = wrist.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos != 15) { //50
                    //true causes the action to return
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setTargetPosition(15);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    wrist.setPower(0);
                    return false;
                }
            }

            public Action intakeTransfer() {
                return new IntakeTransfer();
            }
        }

        public class IntakeIdle implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    wrist.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = wrist.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos != 400) { //50
                    //true causes the action to return
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setTargetPosition(400);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    wrist.setPower(0);
                    return false;
                }
            }

            public Action intakeIdle() {
                return new IntakeIdle();
            }
        }


    }

    /*public class intakeServo{
        private Servo spin;

        public intakeServo(HardwareMap hardwaremap){
            spin = hardwareMap.get(Servo.class, "intake_servo"); //NOT SURE WHAT IT IS IN CONFIG
            spin.setPosition(0.5);
        }
        public class IntakeServoStop implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    spin.setPosition(0.5);
                    initialized = true;
                }
                //checks lift's current position
                double pos = spin.getPosition();
                packet.put("liftPos", pos);
                spin.setPosition(1);
                try {
                    threeSampleShoddy.intakeServo.this.wait(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return true;
            }

        }
        public Action intakeServoStop() {
            return new IntakeServoStop();
        }


        public class IntakeServoPickUp implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    spin.setPosition(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = spin.getPosition();
                packet.put("liftPos", pos);
                if (pos != 1) { //50
                    //true causes the action to return
                    spin.setPosition(1);

                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    spin.setPosition(0.5);
                    return false;
                }
            }

        }
        public Action intakeServoPickUp() {
            return new IntakeServoPickUp();
        }

    }*/
    public class intakeServo {
        private Servo spin;

        public intakeServo(HardwareMap hardwareMap) {
            spin = hardwareMap.get(Servo.class, "intake_servo"); // Make sure the configuration matches
            spin.setPosition(0.5); // Initialize servo position
        }

        // Action to stop the intake servo (set to position 0.5)
        public class IntakeServoStop implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Initialize the servo movement and record the start time
                if (!initialized) {
                    spin.setPosition(0.5);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                // Check if 3 seconds have elapsed
                long elapsedTime = System.currentTimeMillis() - startTime;
                if (elapsedTime >= 3000) {
                    spin.setPosition(0); // Stop the servo
                    return true; // Action is complete
                }

                // Update telemetry
                packet.put("servoPos", spin.getPosition());
                return false; // Action is still running
            }
        }

        public Action intakeServoStop() {
            return new IntakeServoStop();
        }

        // Action to pick up using the intake servo (set to position 1)
        public class IntakeServoPickUp implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Initialize the servo movement and record the start time
                if (!initialized) {
                    spin.setPosition(1);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                // Check if 3 seconds have elapsed
                long elapsedTime = System.currentTimeMillis() - startTime;
                if (elapsedTime >= 3000) {
                    spin.setPosition(0.5); // Stop the servo
                    return true; // Action is complete
                }

                // Update telemetry
                packet.put("servoPos", spin.getPosition());
                return false; // Action is still running
            }
        }

        public Action intakeServoPickUp() {
            return new IntakeServoPickUp();
        }
    }

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
                //powers on motor if not on
                if (!initialized){
                    //rotateOut.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = rotateOut.getPosition();
                packet.put("liftPos", pos);
                if (pos > 0.06) {
                    //true causes the action to return
                    //rotateOut.se;
                    rotateOut.setPosition(0.06);
                    return true;
                } else {
                    //false stops action rerun
                    //rotateOut.setPower(0);
                    rotateOut.setPosition(0.7);
                    return false;
                }
            }
        }

        public Action outtakeIt() {
            return new OuttakeIt();
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
            return new OuttakeHome();
        }


    }




    //begin code
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initPose = new Pose2d(-48, -72, Math.toRadians(0));

        Pose2d scoring = new Pose2d(-65, -65, Math.toRadians(45));

        Pose2d first = new Pose2d(-48, -46, Math.toRadians(135));

        Pose2d second = new Pose2d(-58, -46, Math.toRadians(135));

        Pose2d third = new Pose2d(-68, -46, Math.toRadians(135));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        slideVertical lift = new slideVertical(hardwareMap);
        slideHorizontal extend = new slideHorizontal(hardwareMap);
        wristDrive wrist = new wristDrive(hardwareMap);
        intakeServo spin = new intakeServo(hardwareMap);
        outtakeServo rotateOut = new outtakeServo(hardwareMap);

        TrajectoryActionBuilder score1 = drive.actionBuilder(initPose)
                .strafeToLinearHeading(new Vector2d(-65, -65), Math.toRadians(45));

        TrajectoryActionBuilder grab2 = drive.actionBuilder(scoring)
                .strafeToLinearHeading(new Vector2d(-48, -46), Math.toRadians(135));

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
                 //lift.highLift(),
                 //extend.hSlideOut()
                 //wrist.IntakeTransfer()
                 //intakeServo.IntakeServoPickUp ,
                 score1.build(),
                 grab2.build(),
                 score2.build(),
                 grab3.build(),
                 score3.build(),
                 grab4.build(),
                 score4.build()
            )
        );



    }
}