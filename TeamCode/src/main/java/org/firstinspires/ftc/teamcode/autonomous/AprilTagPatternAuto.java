package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
@Disabled
@Autonomous(name = "AprilTag Pattern Auto")
public class AprilTagPatternAuto extends LinearOpMode {

    // -------- Vision --------
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // -------- State --------
    private int lastSeenTag = -1;

    @Override
    public void runOpMode() {

        // =============================
        // Vision Setup
        // =============================
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change if needed
                .build();

        telemetry.addLine("Scanning AprilTags...");
        telemetry.update();

        // =============================
        // INIT LOOP — keep scanning
        // =============================
        while (!isStarted() && !isStopRequested()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {

                for (AprilTagDetection tag : detections) {
                    if (tag.id == 21 || tag.id == 22 || tag.id == 23) {
                        lastSeenTag = tag.id;
                    }
                }

                telemetry.addData("Last Seen Tag", lastSeenTag);
                telemetry.addData("Auto Choice", tagToAuto(lastSeenTag));
            } else {
                telemetry.addLine("No tag visible");
                telemetry.addData("Last Seen Tag", lastSeenTag);
            }

            telemetry.update();
            sleep(50);
        }

        // =============================
        // START — lock decision
        // =============================
        waitForStart();

        int chosenTag = lastSeenTag;

        // fallback if none seen
        if (chosenTag == -1) {
            chosenTag = 21; // default → PGP
        }

        telemetry.addData("FINAL TAG", chosenTag);
        telemetry.addData("RUNNING", tagToAuto(chosenTag));
        telemetry.update();

        // =============================
        // Run Selected Auto
        // =============================
        if (chosenTag == 22) {
            runPPG();
        }
        else if (chosenTag == 21) {
            runPGP();
        }
        else if (chosenTag == 23) {
            runGPP();
        }

        // =============================
        // Cleanup
        // =============================
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // =============================
    // Your Auto Routines
    // Replace with Pedro paths + shooter logic
    // =============================

    private void runPPG() {
        telemetry.addLine("Running PPG Auto");
        telemetry.update();
        sleep(1000);
    }

    private void runPGP() {
        telemetry.addLine("Running PGP Auto");
        telemetry.update();
        sleep(1000);
    }

    private void runGPP() {
        telemetry.addLine("Running GPP Auto");
        telemetry.update();
        sleep(1000);
    }

    // =============================
    // Helper for telemetry
    // =============================

    private String tagToAuto(int id) {
        if (id == 22) return "PPG";
        if (id == 21) return "PGP";
        if (id == 23) return "GPP";
        return "UNKNOWN";
    }
}
