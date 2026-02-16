package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "AprilTag Init Select Auto")
public class AprilTagInitSelectAuto extends LinearOpMode {

    // ---- Vision ----
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ---- Selection State ----
    private int detectedTagId = -1;
    private int selectedPath = 0;

    @Override
    public void runOpMode() {

        // =========================
        // Vision Setup
        // =========================
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change if needed
                .build();

        telemetry.addLine("Vision initialized");
        telemetry.update();

        // =========================
        // INIT LOOP — Scan Tags
        // =========================
        while (!isStarted() && !isStopRequested()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {

                AprilTagDetection tag = detections.get(0);
                detectedTagId = tag.id;

                // ---- Map tag → path ----
                if (tag.id == 1) selectedPath = 1;
                else if (tag.id == 2) selectedPath = 2;
                else if (tag.id == 3) selectedPath = 3;

                telemetry.addLine("TAG DETECTED");
                telemetry.addData("ID", tag.id);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Selected Path", selectedPath);

            } else {
                telemetry.addLine("No tag visible");
                telemetry.addData("Last Selected Path", selectedPath);
            }

            telemetry.update();
            sleep(50);
        }

        // =========================
        // START PRESSED — Freeze Choice
        // =========================
        waitForStart();

        if (selectedPath == 0) {
            selectedPath = 2; // safe default
        }

        telemetry.addData("FINAL PATH", selectedPath);
        telemetry.update();

        // =========================
        // Run Selected Auto Path
        // =========================
        if (selectedPath == 1) {
            runLeftAuto();
        }
        else if (selectedPath == 2) {
            runCenterAuto();
        }
        else {
            runRightAuto();
        }

        // =========================
        // Cleanup
        // =========================
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // =========================
    // Example Path Methods
    // =========================

    private void runLeftAuto() {
        telemetry.addLine("Running LEFT auto");
        telemetry.update();
        sleep(1000);
        // your pedro pathing call here
    }

    private void runCenterAuto() {
        telemetry.addLine("Running CENTER auto");
        telemetry.update();
        sleep(1000);
    }

    private void runRightAuto() {
        telemetry.addLine("Running RIGHT auto");
        telemetry.update();
        sleep(1000);
    }
}


