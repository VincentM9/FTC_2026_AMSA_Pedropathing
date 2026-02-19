package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
@Autonomous(name="Red Auto Pedro Path", group="Six Seven")
public class RedAutoPedroPath extends OpMode {

    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer, opmodeTimer;

/*
    public enum PathState {
        //Start POSITION_END  POSITION
        //Drive > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE ARTIFACT

        //DRIVE_STARTPOST_SHOOT_POS

        //SHOOT_PRELOAD
    }
    PathState pathState;
*/
    private int pathState; // Current autonomous path state (state machine)


    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower); //Build paths
        follower.setStartingPose(new Pose(126.16829745596868, 121.0019569471624, Math.toRadians(125)));
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
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
                                    new Pose(112.078, 83.339)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(112.078, 83.339),
                                    new Pose(128.272, 83.137)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.272, 83.137),
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
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 1 Done");
                }
                follower.followPath(paths.Path2);
                setPathState(2);
                break;
            case 2:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 2 Done");
                }
                follower.followPath(paths.Path3);
                setPathState(3);
                break;
            case 3:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 3 Done");
                }
                follower.followPath(paths.Path4);
                setPathState(4);
                break;
            case 4:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 4 Done");
                }
                follower.followPath(paths.Path5);
                setPathState(5);
                break;
            case 5:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 5 Done");
                }
                follower.followPath(paths.Path6);
                setPathState(6);
                break;
            case 6:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 6 Done");
                }
                follower.followPath(paths.Path7);
                setPathState(7);
                break;
            case 7:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 7 Done");
                }
                follower.followPath(paths.Path8);
                setPathState(8);
                break;
            case 8:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 8 Done");
                }
                follower.followPath(paths.Path9);
                setPathState(9);
                break;
            case 9:
                if(!follower.isBusy()) {
                    telemetry.addLine("Path 9 Done");
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}


