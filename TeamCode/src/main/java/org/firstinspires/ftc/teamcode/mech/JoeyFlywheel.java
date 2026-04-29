package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.mech.IntakeV3;


@TeleOp
public class JoeyFlywheel extends OpMode {
    private DcMotorEx outtakeT;
    private DcMotorEx outtakeB;

    IntakeV3 cannon = null;

    public double highVelocity = 1320;
    public double lowVelocity = 1040;

    double curTargetVelocity = highVelocity;

    double F = 21.25;
    double P = 212.3;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        outtakeT = hardwareMap.get(DcMotorEx.class, "outtakeT");
        outtakeB = hardwareMap.get(DcMotorEx.class, "outtakeB");
        outtakeT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeT.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeB.setDirection(DcMotorSimple.Direction.FORWARD);
        cannon = new IntakeV3(hardwareMap);
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

        if (gamepad2.rightBumperWasPressed()) {
            stepIndex = (stepIndex - 1) % stepSizes.length;
        }
        if (gamepad2.leftBumperWasPressed()) {
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

        if(gamepad1.a){
            cannon.intake(1);
        }

        if(gamepad1.b){
            cannon.intake(0);
        }

        if(gamepad1.right_trigger > 0.1){
            cannon.setActuatorPos(1);
        }

        if(gamepad1.x){
            cannon.setGatePosition(0.15);
        }

        if(gamepad1.y){
            cannon.setGatePosition(0);
        }

        if(gamepad1.left_stick_button){
            F = 12.1;
            P = 120;
        }

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