package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose currentPose = new Pose(0, 0, 0);
    public void resetPose(){
        currentPose = new Pose(0, 0, 0);
    }
}