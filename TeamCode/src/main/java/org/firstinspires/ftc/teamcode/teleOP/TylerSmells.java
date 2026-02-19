package org.firstinspires.ftc.teamcode.teleOP;

//import com.bylazar.gamepad.*;
import android.util.Size;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.GamepadPluginConfig;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "TylerSmells", group= "OpMode")
public class TylerSmells extends OpMode {
    private final GamepadManager g1Manager = PanelsGamepad.INSTANCE.getFirstManager();
    private final GamepadManager g2Manager = PanelsGamepad.INSTANCE.getSecondManager();

    private Timer actionTimer, opmodeTimer;
    private TelemetryManager panelsTelemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf;
    private DcMotor lr;
    private DcMotor rf;
    private DcMotor rr;
    private DcMotor spinny;
    private Servo roulette;
    private DcMotor BallTransfer;
    private DcMotor RubberBandIntake;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean prevRightBumper = false;
    private boolean shootingSequenceActive = false;
    private int shootingStep = 0;
    private boolean transferBurstActive = false;
    private boolean transferReverseActive = false;
    private boolean spinReverseActive = false;


    @Override
    /** This method is called once at the init of the OpMode. **/
    public void init(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        float y;
        double x;
        float rx;

        actionTimer = new Timer();

        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        spinny = hardwareMap.get(DcMotor.class, "spinny");
        roulette = hardwareMap.get(Servo.class, "roulette");
        BallTransfer = hardwareMap.get(DcMotor.class, "Ball Transfer");
        RubberBandIntake = hardwareMap.get(DcMotor.class, "Rubber Band Intake");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);

        panelsTelemetry.addData("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    /** This method is called continuously after Init while waiting for "play". **/
    public void init_loop(){}

    @Override
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions**/
    public void start(){
        runtime.reset();
        actionTimer.resetTimer();
    }

    @Override
    public void loop(){

        /** Panels Gamepad Plugin **/
        Gamepad g1 = g1Manager.asCombinedFTCGamepad(gamepad1);
        Gamepad g2 = g2Manager.asCombinedFTCGamepad(gamepad2);
        gamePadTelemetry(g1, g2);
        /** End of Panels Gamepad Plugin **/

        /** Driving Code **/
        double max;
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if(max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;

        }

        lf.setPower(frontLeftPower);
        rf.setPower(frontRightPower);
        lr.setPower(backLeftPower);
        rr.setPower(backRightPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.update();

        /** End Of Driving Code **/

        /** Six Seven Code  **/
        if (gamepad2.rightBumperWasPressed()) { //If right bumper is pressed
            spinny.setPower(0.6); //Set spinny power to 0.6
        }
        if (gamepad2.rightBumperWasReleased()) { //If right bumper is released
            spinny.setPower(0); //Set spinny power to 0
        }

        if (gamepad2.left_bumper) {
            spinny.setPower(-0.35);
            //sleep(500);
            spinny.setPower(0);
        }
        if (spinny.getPower() <= -0.15) {
            gamepad2.rumble(1, 0, 50);
        }
        if (spinny.getPower() >= 0.5) {
            gamepad2.rumble(0, 1, 50);
        }

        // End Ball Launcher (Motor)
        // Start Artifact Spinner (Servo)
        if (gamepad2.a) {
            roulette.setPosition(1.0 / 280.0);
        }
        if (gamepad2.b) {
            roulette.setPosition(129.0 / 280.0);
        }
        if (gamepad2.x) {
            roulette.setPosition(261.0 / 280.0);
        }

        if (gamepad2.y && !shootingSequenceActive) {
            shootingSequenceActive = true;
            shootingStep = 0;
            actionTimer.resetTimer();
        }
        if (shootingSequenceActive) {
            shootingSequence();
        }

        // End Artifact Spinner (Servo)

        // Start Artifact Transfer (Motor)
        if (gamepad2.dpad_up) {
            BallTransfer.setPower(-0.65);
            //sleep(670);
            BallTransfer.setPower(0);
        }
        if (gamepad2.dpad_down) {
            BallTransfer.setPower(0.4);
            RubberBandIntake.setPower(0);
            //sleep(500);
            BallTransfer.setPower(0);
        }
        // End Artifact Transfer (Motor)

        // Start Artifact Intake (Motor)
        if (gamepad2.dpad_right) {
            gamepad2.rumble(1, 0, 2);
            RubberBandIntake.setPower(-1);
        }
        if (gamepad2.dpad_left) {
            RubberBandIntake.setPower(1);
        }
        // End Artifact Intake (Motor)

        /** End of Six Seven Code  **/

    }

    @Override
    /** We do not use this because everything should automatically disable **/
    public void stop(){
        panelsTelemetry.addLine("Stop");
        panelsTelemetry.addData("Elapsed Time", opmodeTimer.getElapsedTime());
        panelsTelemetry.update(telemetry);
    }

    public void gamePadTelemetry(Gamepad g1, Gamepad g2){
        panelsTelemetry.debug("==== Buttons ====");
        panelsTelemetry.debug("A: " + g1.a);
        panelsTelemetry.debug("B: " + g1.b);
        panelsTelemetry.debug("X: " + g1.x);
        panelsTelemetry.debug("Y: " + g1.y);

        panelsTelemetry.debug("DPAD Up: " + g1.dpad_up);
        panelsTelemetry.debug("DPAD Down: " + g1.dpad_down);
        panelsTelemetry.debug("DPAD Left: " + g1.dpad_left);
        panelsTelemetry.debug("DPAD Right: " + g1.dpad_right);

        panelsTelemetry.debug("Left Bumper: " + g1.left_bumper);
        panelsTelemetry.debug("Right Bumper: " + g1.right_bumper);

        panelsTelemetry.debug("Left Trigger: " + g1.left_trigger);
        panelsTelemetry.debug("Right Trigger: " + g1.right_trigger);

        panelsTelemetry.debug("Start / Options: " + g1.options);
        panelsTelemetry.debug("Back / Share: " + g1.back);
        panelsTelemetry.debug("Guide / PS: " + g1.guide);
        panelsTelemetry.debug("Touchpad: " + g1.touchpad);

        panelsTelemetry.debug("Left Stick Button: " + g1.left_stick_button);
        panelsTelemetry.debug("Right Stick Button: " + g1.right_stick_button);

        panelsTelemetry.debug("==== Sticks ====");
        panelsTelemetry.debug("Left Stick X: " + g1.left_stick_x);
        panelsTelemetry.debug("Left Stick Y: " + g1.left_stick_y);
        panelsTelemetry.debug("Right Stick X: " + g1.right_stick_x);
        panelsTelemetry.debug("Right Stick Y: " + g1.right_stick_y);

        panelsTelemetry.update(telemetry);
    }
    public void shootingSequence() {
        switch (shootingStep) {

            case 0:
                roulette.setPosition(1.0 / 280.0);
                actionTimer.resetTimer();
                shootingStep++;
                break;

            case 1:
                if (actionTimer.getElapsedTime() > 1500) {
                    BallTransfer.setPower(-0.65);
                    actionTimer.resetTimer();
                    shootingStep++;
                }
                break;

            case 2:
                if (actionTimer.getElapsedTime() > 750) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(129.0 / 280.0);
                    actionTimer.resetTimer();
                    shootingStep++;
                }
                break;

            case 3:
                if (actionTimer.getElapsedTime() > 750) {
                    BallTransfer.setPower(-0.65);
                    actionTimer.resetTimer();
                    shootingStep++;
                }
                break;

            case 4:
                if (actionTimer.getElapsedTime() > 750) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(261.0 / 280.0);
                    actionTimer.resetTimer();
                    shootingStep++;
                }
                break;

            case 5:
                if (actionTimer.getElapsedTime() > 750) {
                    BallTransfer.setPower(-0.65);
                    actionTimer.resetTimer();
                    shootingStep = 6;
                }
                break;

            case 6:
                if (actionTimer.getElapsedTime() > 750) {
                    BallTransfer.setPower(0);
                    shootingSequenceActive = false; // DONE
                }
                break;
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagID(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()
}
