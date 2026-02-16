package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TylerSmells", group="Linear OpMode")
public class TylerSmells extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf;
    private DcMotor lr;
    private DcMotor rf;
    private DcMotor rr;
    private DcMotor spinny;
    private Servo roulette;
    private DcMotor BallTransfer;
    private DcMotor RubberBandIntake;
    private boolean prevRightBumper = false;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float y;
        double x;
        float rx;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Put initialization blocks here.
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                /* OLD DRIVING CODE NO WORKIE
                telemetry.update();
                // Start driving section (Motors)
                y = -gamepad1.left_stick_y;
                x = 1.1 * gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;
                // left_front_drive = Port 3
                lf.setPower(-1 * (y + x + rx));
                // left_rear_type = Port 2
                lr.setPower(-1 * ((y - x) + rx));
                // right_front_drive = Port 1
                rf.setPower((y - x) - rx);
                // right_rear_drive = Port 0
                rr.setPower((y + x) - rx);
                */

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

                // End driving section
                // Start Ball Launcher (Motor)
                /*
                boolean rightBumper = gamepad2.right_bumper;
                if (rightBumper && !prevRightBumper) { //If right bumper is pressed
                    spinny.setPower(0.6); //Set spinny power to 0.6
                }
                if (!rightBumper && prevRightBumper) { //If right bumper is released
                    spinny.setPower(0); //Set spinny power to 0
                }
                if (gamepad2.left_bumper) {
                    spinny.setPower(-0.35);
                    sleep(500);
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
                if (gamepad2.y) {
                    roulette.setPosition(1.0 / 280.0);
                    sleep(1500);
                    BallTransfer.setPower(-0.65);
                    sleep(750);
                    BallTransfer.setPower(0);
                    roulette.setPosition(129.0 / 280.0);
                    sleep(750);
                    BallTransfer.setPower(-0.65);
                    sleep(750);
                    BallTransfer.setPower(0);
                    roulette.setPosition(261.0 / 280.0);
                    sleep(750);
                    BallTransfer.setPower(-0.65);
                    sleep(750);
                    BallTransfer.setPower(0);
                }
                // End Artifact Spinner (Servo)
                // Start Artifact Transfer (Motor)
                if (gamepad2.dpad_up) {
                    BallTransfer.setPower(-0.65);
                    sleep(670);
                    BallTransfer.setPower(0);
                }
                if (gamepad2.dpad_down) {
                    BallTransfer.setPower(0.4);
                    RubberBandIntake.setPower(0);
                    sleep(500);
                    BallTransfer.setPower(0);
                }
                // End Artifact Spinner (Servo)
                // Start Artifact Intake (Motor)
                if (gamepad2.dpad_right) {
                    RubberBandIntake.setPower(-1);
                }
                if (gamepad2.dpad_left) {
                    RubberBandIntake.setPower(1);
                }
                // End Artifact Intake (Motor)

                 */
            }
        }
    }
}
