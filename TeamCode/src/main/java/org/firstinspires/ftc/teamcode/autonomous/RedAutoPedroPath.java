package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helper.AutoMethods;
import org.firstinspires.ftc.teamcode.helper.Drawing;
import org.firstinspires.ftc.teamcode.helper.PoseStorage;
import org.firstinspires.ftc.teamcode.helper.RedAutoPath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "RedAutoPedroPath", group = "Six Seven")
public class RedAutoPedroPath extends OpMode {

    // Shooting state enum (shared with AutoMethods)
    public enum ShootingState {
        IDLE,
        POSITION_1, FIRE_1,
        POSITION_2, FIRE_2,
        POSITION_3, FIRE_3,
        DONE
    }

    //Hardware constants
    private static final double SHOOTING_POWER  =  0.575;
    private static final double TRANSFER_POWER = 0.7;
    private static final double INTAKE_POWER =  1.0;

    //Core objects
    private TelemetryManager panelsTelemetry;
    public  Follower follower;
    private RedAutoPath paths;
    private AutoMethods autoMethods;       // helper class
    private Timer pathTimer, actionTimer, opmodeTimer, intakeTimer, rouletteTimer;

    // Hardware
    private Servo roulette;
    private DcMotor RubberBandIntake, BallTransfer;
    private DcMotorEx flywheel;

    // State
    private int pathState;
    private int tag;
    private int lastSeenTag = -1;
    private final int  flywheelVelo = 1200;

    private GoBildaPinpointDriver goBildaPinpointDriver;
    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer   = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        intakeTimer = new Timer();
        rouletteTimer = new Timer();

        opmodeTimer.resetTimer();

        // Hardware map
        roulette         = hardwareMap.get(Servo.class,      "roulette");
        flywheel         = hardwareMap.get(DcMotorEx.class,  "spinny");
        BallTransfer     = hardwareMap.get(DcMotor.class,    "Ball Transfer");
        RubberBandIntake = hardwareMap.get(DcMotor.class,    "Rubber Band Intake");

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        BallTransfer.setDirection(DcMotor.Direction.FORWARD);
        RubberBandIntake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(14, 0, 0, 70);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        paths    = new RedAutoPath(follower);
        follower.setStartingPose(new Pose(125.16364629317799, 121.16939880762749, Math.toRadians(125)));
        Drawing.init();

        // AutoMethods helper (inject all dependencies)
        autoMethods = new AutoMethods(
                panelsTelemetry,
                roulette,
                BallTransfer,
                flywheel,
                actionTimer,
                TRANSFER_POWER,
                flywheelVelo
        );
        autoMethods.initAprilTag(hardwareMap);

        panelsTelemetry.addData("Status", "Initialized");
        panelsTelemetry.addLine("Scanning AprilTags...");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        List<AprilTagDetection> detections = autoMethods.getAprilTagDetections();

        if (!detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                    lastSeenTag = detection.id;
                }
            }
            panelsTelemetry.addData("Last Seen Tag", lastSeenTag);
        } else {
            panelsTelemetry.addLine("No tag visible");
            panelsTelemetry.addData("Last Seen Tag", lastSeenTag);
        }

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        tag = (lastSeenTag != -1) ? lastSeenTag : 22; // default → PPG

        autoMethods.stopVisionPortal();

        panelsTelemetry.addData("FINAL TAG", tag);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        Drawing.drawDebug(follower);

        // Delegate shooting to AutoMethods
        if (autoMethods.shooting) {
            if      (tag == 22) autoMethods.shootPPG();
            else if (tag == 21) autoMethods.shootPGP();
            else if (tag == 23) autoMethods.shootGPP();
            else autoMethods.shootGPP();
        }

        panelsTelemetry.addData("Path State", pathState);
        panelsTelemetry.addData("Shooting State", autoMethods.shootingState);
        panelsTelemetry.addData("X",       follower.getPose().getX());
        panelsTelemetry.addData("Y",       follower.getPose().getY());
        panelsTelemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        autoMethods.stopShooting();
        RubberBandIntake.setPower(0);

        // Save final pose
        PoseStorage.currentPose = follower.getPose();

        panelsTelemetry.addLine("Stopped");
        panelsTelemetry.addData("Elapsed Time", opmodeTimer.getElapsedTime());
        panelsTelemetry.update(telemetry);
    }
    private boolean roulettePrimed = false;
    public void autonomousPathUpdate() {
        flywheel.setVelocity(flywheelVelo);
        RubberBandIntake.setPower(INTAKE_POWER);

        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1: // Move to first shooting position → shoot 3 balls → go to intake
                if (!follower.isBusy() && autoMethods.shootingState == ShootingState.IDLE) {
                    panelsTelemetry.addLine("Path 1 Done > Starting Shooting");
                    autoMethods.patternShooting();
                }
                if (!follower.isBusy() && autoMethods.shootingState == ShootingState.DONE) {
                    panelsTelemetry.addLine("Shooting 3 Balls Done > Starting Intake");
                    autoMethods.shootingState = ShootingState.IDLE;
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;

            //-------------------------------------------------------------------------------- >
            case 2: // lolz
                if (!follower.isBusy() && !roulettePrimed) {
                    panelsTelemetry.addLine("Path 2 Done > Starting 2 Intake");
                    roulette.setPosition(10.0 / 280.0);
                    rouletteTimer.resetTimer();
                    roulettePrimed = true;

                }
                //ADJUST BASED ON HOW MUCH OVERLAP IS ALLOWED BETWEEN MOVING AND THE ROULETTE SPINNING
                if(!follower.isBusy() && rouletteTimer.getElapsedTime() > 1000 && roulettePrimed) {
                    follower.followPath(paths.Path3);
                    setPathState(3);
                    roulettePrimed = false;
                }
                break;

            case 3: // Intake 2
                if (!follower.isBusy() && !roulettePrimed) {
                    panelsTelemetry.addLine("2 Intake Done > Starting 1 Intake");
                    roulette.setPosition(200.0 / 280.0);
                    rouletteTimer.resetTimer();
                    roulettePrimed = true;

                }
                //ADJUST BASED ON HOW MUCH OVERLAP IS ALLOWED BETWEEN MOVING AND THE ROULETTE SPINNING
                if(!follower.isBusy() && rouletteTimer.getElapsedTime() > 1000 && roulettePrimed) {
                    follower.followPath(paths.Path4);
                    setPathState(4);
                    roulettePrimed = false;
                }

                break;
            //-------------------------------------------------------------------------------- <

            case 4: // Intake 1
                if (!follower.isBusy()) {
                    panelsTelemetry.addLine("Intake Done > Start Path 5");
                    follower.followPath(paths.Path5);
                    setPathState(5);
                    if(tag == 22 || tag == 21) roulette.setPosition(266.0 / 280.0);
                    else if (tag == 23) roulette.setPosition(10.0 / 280.0);
                }
                break;

            //-------------------------------------------------------------------------------- >
            case 5: // Move to second shooting position → shoot 3 balls → go to intake
                if (!follower.isBusy() && autoMethods.shootingState == ShootingState.IDLE) {
                    panelsTelemetry.addLine("Path 5 Done > Starting Shooting");
                    autoMethods.patternShooting();
                }
                if (!follower.isBusy() && autoMethods.shootingState == ShootingState.DONE && !roulettePrimed) {
                    panelsTelemetry.addLine("Starting Shooting >  Starting Intake");
                    autoMethods.shootingState = ShootingState.IDLE;
                    roulette.setPosition(200.0 / 280.0);
                    rouletteTimer.resetTimer();
                    roulettePrimed = true;
                }
                //ADJUST BASED ON HOW MUCH OVERLAP IS ALLOWED BETWEEN MOVING AND THE ROULETTE SPINNING
                if(!follower.isBusy() && rouletteTimer.getElapsedTime() > 1000  && roulettePrimed) {
                    follower.followPath(paths.Path6);
                    setPathState(6);
                    roulettePrimed = false;
                }
                break;

            case 6: // Intake 1
                if (!follower.isBusy() && !roulettePrimed) {
                    panelsTelemetry.addLine("1 Intake Done > Starting 2 Intake");
                    roulette.setPosition(200.0 / 280.0);
                    rouletteTimer.resetTimer();
                    roulettePrimed = true;
                }
                //ADJUST BASED ON HOW MUCH OVERLAP IS ALLOWED BETWEEN MOVING AND THE ROULETTE SPINNING
                if(!follower.isBusy() && rouletteTimer.getElapsedTime() > 1000 && roulettePrimed) {
                    follower.followPath(paths.Path7);
                    setPathState(7);
                    roulettePrimed = false;
                }
                break;
            //-------------------------------------------------------------------------------- <

            case 7: // Intake  2
                if (!follower.isBusy()) {
                    panelsTelemetry.addLine("2 Intake Done > Starting Path 8");
                    follower.followPath(paths.Path8);
                    setPathState(8);
                    if(tag == 22 || tag == 21) roulette.setPosition(266.0 / 280.0);
                    else if (tag == 23) roulette.setPosition(10.0 / 280.0);
                }
                break;

            case 8: // Move to third shooting position → shoot 3 balls → park
                if (!follower.isBusy() && autoMethods.shootingState == ShootingState.IDLE) {
                    panelsTelemetry.addLine("Path 8 Done > Starting Shooting");
                    autoMethods.patternShooting();
                }
                if (!follower.isBusy() && autoMethods.shootingState == ShootingState.DONE) {
                    panelsTelemetry.addLine("Shooting Done > Moving to final park Position");
                    autoMethods.shootingState = ShootingState.IDLE;
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }

                break;

            case 9: // Final park position
                if (!follower.isBusy()) {
                    panelsTelemetry.addLine("In final Position > Autonomous Complete");
                }
                break;
        }
    } // Path state machine

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        roulettePrimed = false;
    }
}