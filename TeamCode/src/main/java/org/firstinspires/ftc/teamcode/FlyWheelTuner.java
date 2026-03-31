package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.mech.IntakeV3;

@TeleOp(name="FlyWheelTuner", group="Iterative OpMode")
public class FlyWheelTuner extends OpMode {

    private VoltageSensor batteryVoltageSensor;

    private DcMotorEx outtakeT;
    private DcMotorEx outtakeB;

    private boolean aPressed;
    private boolean oldAPressed;
    private boolean intakeOn = true;

    private double highVel = 1600;
    private double lowVel = 1270;

    IntakeV3 cannon = null;

    double f = 0;
    double p = 0;

    double curTargetVel = highVel;

    double[] stepSizes = {10,1,.1,.001,.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        outtakeT = hardwareMap.get(DcMotorEx.class, "outtakeT");
        outtakeB = hardwareMap.get(DcMotorEx.class, "outtakeB");

        cannon = new IntakeV3(hardwareMap);


        outtakeT.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeB.setDirection(DcMotorEx.Direction.FORWARD);

        outtakeT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, 0, 0, f);

        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        outtakeT.setZeroPowerBehavior(FLOAT);
        outtakeB.setZeroPowerBehavior(FLOAT);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        aPressed = gamepad2.a;
        if(gamepad1.yWasPressed()){
            if(curTargetVel == highVel) {
                curTargetVel = lowVel;
            } else{
                curTargetVel = highVel;
            }
        }
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadLeftWasPressed()){
            f -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            f += stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            p += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            p -= stepSizes[stepIndex];
        }

        if (gamepad2.a && !oldAPressed){
            intakeOn = !intakeOn;
            if(intakeOn){
                cannon.intake(0);
            }else {
                cannon.intake(1);
            }
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, 0, 0, f);
        outtakeT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        outtakeT.setVelocity(curTargetVel);
        outtakeB.setVelocity(curTargetVel);

        double curVel = outtakeT.getVelocity();
        double error = curTargetVel - curVel;

        oldAPressed = gamepad2.a;

        telemetry.addData("Target Velocity: ", curTargetVel);
        telemetry.addData("Current Velocity: ", "%.2f", curVel);
        telemetry.addData("Error: ", "%.2f", error);
        telemetry.addData("Tuning P: ", "%.4f (D-Pad U/D)", p);
        telemetry.addData("Tuning F: ", "%.4f (D-Pad L/R)", f);
        telemetry.addData("Step Size: ", "%.4f (B Button)", stepSizes[stepIndex]);

    }

}
