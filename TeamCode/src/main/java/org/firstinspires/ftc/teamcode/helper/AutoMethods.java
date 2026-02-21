package org.firstinspires.ftc.teamcode.helper;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.RedAutoPedroPath;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AutoMethods is a dependency-injected helper class for autonomous shooting logic
 * and AprilTag detection. Pass all required hardware references via the constructor.
 *
 * Roulette positions per pattern
 *     Ball 1 (G) →  10 / 280
 *     Ball 2 (P) → 140 / 280
 *     Ball 3 (P) → 266 / 280
 */
public class AutoMethods {

    // Roulette servo positions (raw fraction of 280)
    // PPG
    private static final double PPG_POS_1 = 266.0 / 280.0;  // Purple
    private static final double PPG_POS_2 = 140.0 / 280.0;  // Purple
    private static final double PPG_POS_3 =  10.0 / 280.0;  // Green

    // PGP
    private static final double PGP_POS_1 = 266.0 / 280.0;  // Purple
    private static final double PGP_POS_2 =  10.0 / 280.0;  // Green
    private static final double PGP_POS_3 = 140.0 / 280.0;  // Purple

    // GPP
    private static final double GPP_POS_1 =  10.0 / 280.0;  // Green
    private static final double GPP_POS_2 = 140.0 / 280.0;  // Purple
    private static final double GPP_POS_3 = 266.0 / 280.0;  // Purple

    //Injected dependencies
    private final TelemetryManager panelsTelemetry;
    private final Servo roulette;
    private final DcMotor BallTransfer;
    private final DcMotorEx flywheel;
    private final Timer actionTimer;
    private final double transfer_power;

    private final int flywheelVelocity;

    //AprilTag state
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //Shooting state (owned by AutoMethods, read back by OpMode)
    public RedAutoPedroPath.ShootingState shootingState = RedAutoPedroPath.ShootingState.IDLE;
    public boolean shooting = false;

    /**
     * @param panelsTelemetry  Telemetry manager from the OpMode
     * @param roulette         Roulette servo
     * @param BallTransfer     Ball transfer motor
     * @param flywheel         Flywheel motor
     * @param actionTimer      Shared action timer from the OpMode
     * @param transfer_power   Power value for BallTransfer (negative = forward)
     * @param flywheelVelocity   Velocity/ for flywheel
     */

    public AutoMethods(
            TelemetryManager panelsTelemetry,
            Servo roulette,
            DcMotor BallTransfer,
            DcMotorEx flywheel,
            Timer actionTimer,
            double transfer_power,
            int flywheelVelocity
    ) {
        this.panelsTelemetry  = panelsTelemetry;
        this.roulette         = roulette;
        this.BallTransfer     = BallTransfer;
        this.flywheel         = flywheel;
        this.actionTimer      = actionTimer;
        this.transfer_power   = transfer_power;
        this.flywheelVelocity = flywheelVelocity;
    }

    // Public API

    /** Call this once to kick off a shooting sequence. */
    public void patternShooting() {
        shooting      = true;
        shootingState = RedAutoPedroPath.ShootingState.POSITION_1;
        actionTimer.resetTimer();
    }

    /** Stops all shooting hardware and resets state. */
    public void stopShooting() {
        shooting      = false;
        shootingState = RedAutoPedroPath.ShootingState.IDLE;
        BallTransfer.setPower(0);
    }

    // Shooting patterns  (call the right one each loop() tick)

    /** Purple → Purple → Green */
    public void shootPPG() {
        panelsTelemetry.addData("Auto Pattern", "PPG");
        if (!shooting) return;
        runShootSequence(PPG_POS_1, PPG_POS_2, PPG_POS_3);
    }

    /** Purple → Green → Purple */
    public void shootPGP() {
        panelsTelemetry.addData("Auto Pattern", "PGP");
        if (!shooting) return;
        runShootSequence(PGP_POS_1, PGP_POS_2, PGP_POS_3);
    }

    /** Green → Purple → Purple */
    public void shootGPP() {
        panelsTelemetry.addData("Auto Pattern", "GPP");
        if (!shooting) return;
        runShootSequence(GPP_POS_1, GPP_POS_2, GPP_POS_3);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // AprilTag helpers
    // ─────────────────────────────────────────────────────────────────────────

    public void initAprilTag(HardwareMap hardwareMap) {
        aprilTag    = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        return aprilTag.getDetections();
    }

    public void stopVisionPortal() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
            visionPortal.close();
            visionPortal = null;
        }
    }

    // Private helpers
    /**
     * Generic 3-ball shooting sequence driven by roulette positions.
     * The same state machine runs for all three patterns — only the
     * servo positions differ, making it easy to tune each pattern independently.
     */
    //Timing constants (ms)
    private static final long ROULETTE_MS = 1500; // wait after positioning roulette before firing
    private static final long TRANSFER_MS = 800; // how long to run BallTransfer per ball
    private static final long FIRE_MS = 100;

    private void runShootSequence(double pos1, double pos2, double pos3) {
        switch (shootingState) {
            case POSITION_1:
                roulette.setPosition(pos1);
                setShootingState(RedAutoPedroPath.ShootingState.FIRE_1);
                break;

            case FIRE_1:
                //ADJUST BASED ON HOW MUCH OVERLAP IS ALLOWED BETWEEN MOVING AND THE ROULETTE SPINNING
                if (actionTimer.getElapsedTime() > (ROULETTE_MS)) {
                    BallTransfer.setPower(transfer_power);
                }
                if (actionTimer.getElapsedTime() > (ROULETTE_MS + TRANSFER_MS) ) {
                    BallTransfer.setPower(0);
                    setShootingState(RedAutoPedroPath.ShootingState.POSITION_2);
                }
                break;

            case POSITION_2:
                if (actionTimer.getElapsedTime() > FIRE_MS) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(pos2);
                    setShootingState(RedAutoPedroPath.ShootingState.FIRE_2);
                }
                break;

            case FIRE_2:
                if (actionTimer.getElapsedTime() > ROULETTE_MS) {
                    BallTransfer.setPower(transfer_power);
                }
                if (actionTimer.getElapsedTime() > (ROULETTE_MS + TRANSFER_MS)){
                    BallTransfer.setPower(0);
                    setShootingState(RedAutoPedroPath.ShootingState.POSITION_3);
                }
                break;

            case POSITION_3:
                if (actionTimer.getElapsedTime() > FIRE_MS) {
                    BallTransfer.setPower(0);
                    roulette.setPosition(pos3);
                    setShootingState(RedAutoPedroPath.ShootingState.FIRE_3);
                }
                break;

            case FIRE_3:
                if (actionTimer.getElapsedTime() > (ROULETTE_MS + 500)) {
                    BallTransfer.setPower(transfer_power);
                }
                if (actionTimer.getElapsedTime() > TRANSFER_MS + ROULETTE_MS + 500) {
                    BallTransfer.setPower(0);
                    shootingState = RedAutoPedroPath.ShootingState.DONE;
                    shooting = false;
                }
                break;

            case DONE:
                break;

            case IDLE:
                break;

            default:
                break;
        }
    }

    /**
     * Transitions to a new shooting state and resets the action timer.
     * Also spins up the flywheel on each transition so it stays at speed.
     */
    private void setShootingState(RedAutoPedroPath.ShootingState newState) {
        shootingState = newState;
        flywheel.setVelocity(flywheelVelocity);
        actionTimer.resetTimer();
    }
}
