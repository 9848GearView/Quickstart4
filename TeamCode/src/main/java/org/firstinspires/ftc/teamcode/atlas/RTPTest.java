package org.firstinspires.ftc.teamcode.atlas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.mech.RTPAxon;
import org.firstinspires.ftc.teamcode.mech.GamepadPair;

import org.firstinspires.ftc.teamcode.mech.IntakeV3;


// TeleOp test class for manual tuning and testing
@TeleOp(name = "yay test yayyy", group = "test")
public class RTPTest extends LinearOpMode {
    IntakeV3 cannon = null;
    RTPAxon axon = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        cannon = new IntakeV3(hardwareMap);
        axon = cannon.getRTPAxon();

        RTPAxon servo = new RTPAxon(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            gamepads.copyStates();
            servo.update();

            // Manual controls for target and PID tuning
            if (gamepads.isPressed(-1, "dpad_up")) {
                servo.changeTargetRotation(15);
            }
            if (gamepads.isPressed(-1, "dpad_down")) {
                servo.changeTargetRotation(-15);
            }
            if (gamepads.isPressed(-1, "cross")) {
                servo.setTargetRotation(0);
            }

            if (gamepads.isPressed(-1, "triangle")) {
                servo.setKP(servo.getKP() + 0.001);
            }
            if (gamepads.isPressed(-1, "square")) {
                servo.setKP(Math.max(0, servo.getKP() - 0.001));
            }

            if (gamepads.isPressed(-1, "right_bumper")) {
                servo.setKI(servo.getKI() + 0.0001);
            }
            if (gamepads.isPressed(-1, "left_bumper")) {
                servo.setKI(Math.max(0, servo.getKI() - 0.0001));
            }

            if (gamepads.isPressed(-1, "touchpad")) {
                servo.setKP(0.015);
                servo.setKI(0.0005);
                servo.setKD(0.0025);
                servo.resetPID();
            }

            telemetry.addData("Starting angle", servo.STARTPOS);
            telemetry.addLine(servo.log());
            telemetry.addData("NTRY", servo.ntry);
            telemetry.update();
        }
    }
}
