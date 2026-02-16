package org.firstinspires.ftc.teamcode.teleOP;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Debug Test", group = "Linear OpMode")
public class DebugTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Gamepad", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}