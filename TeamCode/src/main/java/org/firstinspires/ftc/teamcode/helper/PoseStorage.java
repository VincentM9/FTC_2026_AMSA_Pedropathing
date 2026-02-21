package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose currentPose;
    public void resetPose(){
        currentPose = null;
    }
}