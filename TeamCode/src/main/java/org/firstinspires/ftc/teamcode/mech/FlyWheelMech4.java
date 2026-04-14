package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlyWheelMech4 {
    private DcMotorEx outtakeT;
    private DcMotorEx outtakeB;
    private double highVelocity = 1150;
    private double lowVelocity = 900;
    private double feederTimeSeconds = 5;
    double curTargetVelocity = highVelocity;

    // Move these into the constructor or methods
    double F = 16;
    double P = 36;


    ElapsedTime feederTimer = new ElapsedTime();
    private LaunchState launchState = LaunchState.IDLE;

    public void FlywheelMotorOn(int i) {
        outtakeT.setVelocity(i);
        outtakeB.setVelocity(i);
    }

    public void on(boolean b) {

    }

    public void off() {
    }

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    public FlyWheelMech4(HardwareMap hardwareMap) {
        outtakeT = hardwareMap.get(DcMotorEx.class, "outtakeT");
        outtakeB = hardwareMap.get(DcMotorEx.class, "outtakeB");


        // Configuration
        outtakeT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outtakeT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void launch(boolean shotRequested) {
        // Calculate velocity and error INSIDE the method
        double curVelocity = outtakeT.getVelocity();
        double error = curTargetVelocity - curVelocity;

        switch (launchState) {
            case IDLE:
                if (shotRequested) launchState = LaunchState.SPIN_UP;
                break;
            case SPIN_UP:
                outtakeT.setVelocity(highVelocity);
                outtakeB.setVelocity(highVelocity);
                if (curVelocity > lowVelocity) launchState = LaunchState.LAUNCH;
                break;
            case LAUNCH:
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > feederTimeSeconds) {
                    FlywheelMotorOff(); // Stop motors after launch
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public void FlywheelMotorOff() {
        outtakeT.setVelocity(0);
        outtakeB.setVelocity(0);
    }

    public double FlywheelMotorVelocity(){
        return outtakeT.getVelocity();
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Actual Velocity", outtakeT.getVelocity());

    }
}

