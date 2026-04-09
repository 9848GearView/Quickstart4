package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp
public class JoeyFlywheel extends OpMode {
    private DcMotorEx outtakeT;
    private DcMotorEx outtakeB;

    public double highVelocity = 1150;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        outtakeT = hardwareMap.get(DcMotorEx.class, "outtakeT");
        outtakeB = hardwareMap.get(DcMotorEx.class, "outtakeB");
        outtakeT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeT.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeB.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outtakeT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }


    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.leftBumperWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        //set new PIDF coeffcients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outtakeT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //set velocity
        outtakeT.setVelocity(curTargetVelocity);
        outtakeB.setVelocity(curTargetVelocity);


        double curVelocityT = outtakeT.getVelocity();

        double error = curTargetVelocity - curVelocityT;
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%, 2f", curVelocityT);
        telemetry.addData("Error", "%, 2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad U/D)", F);
        telemetry.addData("Step Size", "%.4f (Right Bumper)", stepSizes[stepIndex]);

    }
}