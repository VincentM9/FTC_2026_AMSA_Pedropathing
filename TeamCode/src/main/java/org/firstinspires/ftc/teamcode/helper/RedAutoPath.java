package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedAutoPath {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;

    public RedAutoPath(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.164, 121.169),

                                new Pose(93.492, 98.523)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(43))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(93.492, 98.523),

                                new Pose(100.635, 83.257)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.635, 83.257),

                                new Pose(112.078, 83.339)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(112.078, 83.339),

                                new Pose(125.426, 82.970)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.426, 82.970),

                                new Pose(93.556, 98.614)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(93.556, 98.614),
                                new Pose(87.659, 28.283),
                                new Pose(106.364, 35.123)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(106.364, 35.123),

                                new Pose(121.697, 35.100)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(121.697, 35.100),
                                new Pose(79.281, 40.381),
                                new Pose(93.719, 98.489)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(93.719, 98.489),
                                new Pose(87.572, 50.233),
                                new Pose(111.540, 59.083)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                .build();
    }
}
