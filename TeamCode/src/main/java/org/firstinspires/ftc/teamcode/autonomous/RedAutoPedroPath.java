package org.firstinspires.ftc.teamcode.autonomous;

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
@TeleOp(name="Red Auto Pedro Path", group="Linear Opmode")
public class RedAutoPedroPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //Start POSITION_END  POSITION
        //Drive > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE ARTIFACT

        //DRIVE_STARTPOST_SHOOT_POS

        //SHOOT_PRELOAD
    }

    PathState pathstate;

    private final Pose startPose = new Pose(0, 0, 0);






    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop(){

    }
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
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.168, 121.002),

                                new Pose(100.524, 100.532)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(50))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.524, 100.532),

                                new Pose(99.630, 83.090)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.630, 83.090),

                                new Pose(112.078, 83.339)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(112.078, 83.339),

                                new Pose(128.272, 83.137)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.272, 83.137),

                                new Pose(100.924, 100.456)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(100.924, 100.456),
                                new Pose(87.659, 28.283),
                                new Pose(107.536, 35.123)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(107.536, 35.123),

                                new Pose(121.697, 35.100)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(121.697, 35.100),
                                new Pose(79.281, 40.381),
                                new Pose(100.751, 100.331)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(100.751, 100.331),
                                new Pose(85.048, 51.330),
                                new Pose(112.378, 58.916)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                .build();
    }
}
