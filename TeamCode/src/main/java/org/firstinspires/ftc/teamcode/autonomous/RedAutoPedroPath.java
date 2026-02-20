package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="RedAutoPedroPath", group="Six Seven")
public class RedAutoPedroPath extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Servo roulette; //Servos
    private DcMotor RubberBandIntake, BallTransfer; //DcMotors for Intake/Outtake
    private DcMotorEx flywheel;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private int pathState; // Current autonomous path state (state machine)
    public enum ShootingState {
        IDLE,
        POSITION_1,
        FIRE_1,
        POSITION_2,
        FIRE_2,
        POSITION_3,
        FIRE_3,
        DONE
    }

    private ShootingState shootingState = ShootingState.IDLE;
    private boolean shooting = false;
    private int tag;

    //This should be velocity
    private final double Shooting_Power = 0.575;
    private final double transfer_power = -0.7;
    private final double intake_power = 1;
    private int lastSeenTag = -1;


    @Override
    public void init(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        initAprilTag();

        roulette = hardwareMap.get(Servo.class, "roulette");

        flywheel = hardwareMap.get(DcMotorEx.class, "spinny");
        BallTransfer = hardwareMap.get(DcMotor.class, "Ball Transfer");
        RubberBandIntake = hardwareMap.get(DcMotor.class, "Rubber Band Intake");

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        BallTransfer.setDirection(DcMotor.Direction.FORWARD);
        RubberBandIntake.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0, 0, 0, 0);
        //flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower); //Build paths
        follower.setStartingPose(new Pose(126.16829745596868, 121.0019569471624, Math.toRadians(125)));
        panelsTelemetry.addData("Status", "Initialized");
        panelsTelemetry.addLine("Scanning AprilTags...");
        panelsTelemetry.update(telemetry);

    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {

            for (AprilTagDetection tag : detections) {
                if (tag.id == 21 || tag.id == 22 || tag.id == 23) {
                    lastSeenTag = tag.id;
                }
            }

            telemetry.addData("Last Seen Tag", lastSeenTag);
            telemetry.addData("Auto Choice", "too lazy to write method");
        } else {
            telemetry.addLine("No tag visible");
            telemetry.addData("Last Seen Tag", lastSeenTag);
        }

        telemetry.update();
    }
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system**/
    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
        tag = lastSeenTag;
        if (tag == -1) {
            tag = 22; // default → PPG
        }
        visionPortal.stopStreaming();
        visionPortal.close();
        panelsTelemetry.addData("FINAL TAG", tag);
        panelsTelemetry.addData("RUNNING", "too lazy to write method");
        panelsTelemetry.update();
    }

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();

        if (shooting) {
            if (tag == 22) shootPPG();
            else if (tag == 21) shootPGP();
            else if (tag == 23) shootGPP();
            //shootPPG();
        }
        else {
            BallTransfer.setPower(0);
            flywheel.setPower(0);
        }

        panelsTelemetry.addData("path state", pathState);
        panelsTelemetry.addData("x", follower.getPose().getX());
        panelsTelemetry.addData("y", follower.getPose().getY());
        panelsTelemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()) );
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        panelsTelemetry.addLine("Stop");
        panelsTelemetry.addData("Elapsed Time", opmodeTimer.getElapsedTime());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.168, 121.002),
                                    new Pose(100.524, 100.532)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(50))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.524, 100.532),
                                    new Pose(99.630, 83.090)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(99.630, 83.090),
                                    new Pose(110.158, 83.339)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.158, 83.339),
                                    new Pose(124.192, 82.897)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124.192, 82.897),
                                    new Pose(100.924, 100.456)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(100.924, 100.456),
                                    new Pose(87.659, 28.283),
                                    new Pose(107.536, 35.123)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(107.536, 35.123),
                                    new Pose(121.697, 35.100)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(121.697, 35.100),
                                    new Pose(79.281, 40.381),
                                    new Pose(100.751, 100.331)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(100.751, 100.331),
                                    new Pose(85.048, 51.330),
                                    new Pose(112.378, 58.916)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch(pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && shootingState == ShootingState.IDLE) {
                    panelsTelemetry.addLine("Path 1 Done");
                    panelsTelemetry.update();
                    patternShooting();  // start shooting
                }
                if (!follower.isBusy() && shootingState == ShootingState.DONE) {
                    panelsTelemetry.addLine("First Set of 3 Balls Shot");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path2);
                    setPathState(2);
                    shootingState = ShootingState.IDLE;  // reset for next use

                    //RubberBandIntake.setPower(intake_power);
                }

                break;
            case 2: //Gaybo tdawg put path 2 here for some reason
                if(!follower.isBusy()) {
                    panelsTelemetry.addLine("Path 2 Done");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }

                break;
            case 3: //This should intake 2
                if(!follower.isBusy()) {
                    panelsTelemetry.addLine("Path 3 Done");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path4);
                    setPathState(4);

                    //roulette.setPosition(200.0 / 280.0);
                    //wait like 500 ms
                }
                break;
            case 4: //This should intake the last one
                if(!follower.isBusy()) {
                    panelsTelemetry.addLine("Path 4 Done");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path5);
                    setPathState(5);

                    //RubberBandIntake.setPower(0);
                    //roulette.setPosition(1.0 / 280.0);
                }

                break;
            case 5: //Should shoot 3 balls
                if (!follower.isBusy() && shootingState == ShootingState.IDLE) {
                    panelsTelemetry.addLine("Path 5 Done");
                    panelsTelemetry.update();
                    patternShooting();  // start shooting
                }
                if (!follower.isBusy() && shootingState == ShootingState.DONE) {
                    panelsTelemetry.addLine("Second Set of 3 Balls Shot");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path6);
                    setPathState(6);
                    shootingState = ShootingState.IDLE;  // reset for next use

                    //RubberBandIntake.setPower(intake_power);
                    //roulette.setPosition(200.0 / 280.0);
                    //wait like 500ms
                }

                break;
            case 6: //This should intake 1
                if(!follower.isBusy()) {
                    panelsTelemetry.addLine("Path 6 Done");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path7);
                    setPathState(7);

                    //roulette.setPosition(1.0 / 280.0);
                    //wait like 500ms
                }

                break;
            case 7: // This should intake the last 2
                if(!follower.isBusy()) {
                    panelsTelemetry.addLine("Path 7 Done");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path8);
                    setPathState(8);

                    //RubberBandIntake.setPower(0);
                    //roulette.setPosition(1.0 / 280.0);
                    //wait like 500ms
                }
                break;
            case 8: //Should shoot 3 balls
                if (!follower.isBusy() && shootingState == ShootingState.IDLE) {
                    panelsTelemetry.addLine("Path 8 Done");
                    panelsTelemetry.update();
                    patternShooting();  // start shooting
                }
                if (!follower.isBusy() && shootingState == ShootingState.DONE) {
                    panelsTelemetry.addLine("Third Set of 3 Balls Shot");
                    panelsTelemetry.update();
                    follower.followPath(paths.Path9);
                    setPathState(9);
                    shootingState = ShootingState.IDLE;  // reset for next use

                    //roulette.setPosition(1.0 / 280.0);
                }
                break;
            case 9: //Final position to move onto teleop
                if(!follower.isBusy()) {
                    panelsTelemetry.addLine("Path 9 Done");
                    panelsTelemetry.update();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();

    }

    public void patternShooting(){
            shooting = true;
            shootingState = ShootingState.POSITION_1;
            actionTimer.resetTimer();
    }

    public void setShootingState(ShootingState pState) {
        shootingState = pState;
        flywheel.setPower(Shooting_Power);
        actionTimer.resetTimer();
    }

    public void shootPPG() {
        panelsTelemetry.addData("Auto Pattern:" ,"Running PPG Auto");

        if(!shooting) {return;}

        switch (shootingState) {
            case POSITION_1:
                roulette.setPosition(10.0 / 280.0);
                setShootingState(ShootingState.FIRE_1);
                break;
            case FIRE_1:
                if(actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.POSITION_2);
                }
                break;
            case POSITION_2:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(140.0 / 280.0);
                    setShootingState(ShootingState.FIRE_2);
                }
                break;
            case FIRE_2:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.POSITION_3);
                }
                break;
            case POSITION_3:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(266.0 / 280.0);
                    setShootingState(ShootingState.FIRE_3);
                }
                break;

            case FIRE_3:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                }
                if(actionTimer.getElapsedTime() > 1500) {
                    setShootingState(ShootingState.DONE);
                    shooting = false;
                }
                break;

            case DONE:
                break;
        }

    }

    //Tell Tyler to edit this too lazy myself
    public void shootPGP() {

        panelsTelemetry.addData("Auto Pattern:" ,"Running PGP Auto");
        if(!shooting) {return;}

        switch (shootingState) {
            case POSITION_1:
                roulette.setPosition(10.0 / 280.0);
                setShootingState(ShootingState.FIRE_1);
                break;
            case FIRE_1:
                if(actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.POSITION_2);
                }
                break;
            case POSITION_2:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(140.0 / 280.0);
                    setShootingState(ShootingState.FIRE_2);
                }
                break;
            case FIRE_2:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.POSITION_3);
                }
                break;
            case POSITION_3:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(266.0 / 280.0);
                    setShootingState(ShootingState.FIRE_3);
                }
                break;

            case FIRE_3:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.DONE);
                    shooting = false;
                }
                break;
        }

    }
    //Tell tyler to edit this too lazy myself
    public void shootGPP() {

        panelsTelemetry.addData("Auto Pattern:" ,"Running GPP Auto");
        if(!shooting) {return;}

        switch (shootingState) {
            case POSITION_1:
                roulette.setPosition(10.0 / 280.0);
                setShootingState(ShootingState.FIRE_1);
                break;
            case FIRE_1:
                if(actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.POSITION_2);
                }
                break;
            case POSITION_2:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(140.0 / 280.0);
                    setShootingState(ShootingState.FIRE_2);
                }
                break;
            case FIRE_2:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.POSITION_3);
                }
                break;
            case POSITION_3:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(266.0 / 280.0);
                    setShootingState(ShootingState.FIRE_3);
                }
                break;

            case FIRE_3:
                if (actionTimer.getElapsedTime() > 1000) {
                    BallTransfer.setPower(transfer_power);
                    setShootingState(ShootingState.DONE);
                    shooting = false;
                }
                break;
        }

    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);


    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        panelsTelemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                panelsTelemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                panelsTelemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                panelsTelemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                panelsTelemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                panelsTelemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                panelsTelemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        panelsTelemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        panelsTelemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        panelsTelemetry.addLine("RBE = Range, Bearing & Elevation");

        panelsTelemetry.update();
    }

}


