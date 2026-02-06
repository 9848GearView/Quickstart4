package org.firstinspires.ftc.teamcode.mech;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;


public class IntakeV2 {

    //velocity
    private VoltageSensor batteryVoltageSensor;

    // intake
    private DcMotorEx intake;

    //outtake
    private DcMotorEx outtakeT;
    private DcMotorEx outtakeB;

    //angle
    private Servo angle;

    //gate
    private Servo gate;

    //half park
//    private Servo halfParkL;
//    private Servo halfParkR;

    //turret
    private Servo turretL;
    private Servo turretR;

    ElapsedTime feederTimer = new ElapsedTime();

    private Timer timer;
    private final int DBM = 1000;

    //n Steven and Alex code :) - Mrs. B moved here from their auto code
    final double FEED_TIME_SECONDS = 3;

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    //likely change
    final double LAUNCHER_TARGET_VELOCITY= 850;//started at 1125//was725//Mrs.b changed to 850
    final double LAUNCHER_MIN_VELOCITY = 800;//started at 1075//was 675//mrs B changed to 800





    ElapsedTime feedTimer = new ElapsedTime();


    private LaunchState launchState;


    public enum  LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    public IntakeV2(HardwareMap hwMap) {
        launchState = IntakeV2.LaunchState.IDLE;

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        //intake
        intake = hwMap.get(DcMotorEx.class, "intake");

        //outtake
        outtakeT = hwMap.get(DcMotorEx.class, "outtakeT");
        outtakeB = hwMap.get(DcMotorEx.class, "outtakeB");

        
        //angle
        angle = hwMap.get(Servo.class, "angle");
        
        //gate
        gate = hwMap.get(Servo.class, "gate");

        //half park
//        halfParkL = hwMap.get(Servo.class, "halfParkL");
//        halfParkR = hwMap.get(Servo.class, "halfParkR");

        //turret
        turretL = hwMap.get(Servo.class, "turretL");
        turretR = hwMap.get(Servo.class, "turretR");

        outtakeT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        intake.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeT.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeB.setDirection(DcMotorEx.Direction.REVERSE);




        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting
         */
        outtakeT.setZeroPowerBehavior(FLOAT);
        outtakeB.setZeroPowerBehavior(FLOAT);



        //something.setDirection(CRServo.Direction.REVERSE);


        /*Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
         * This control method reads the current speed as reported by the motor's encoder and applies a
         * varying amount of power to reach, and then hold a target velocity. The FTC SDK calls this
         * control method "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where
         * you control the power applied to the motor directly.
         * Since the dynamics of a OWMotor wheel system varies greatly from those of most other FTC
         * mechanisms, we will also need to adjust the "PIDF" coefficients with some that are a better fit * for our application.
         */
        double voltage = batteryVoltageSensor.getVoltage();
        double adjustedF = 12 * (12.0 / voltage);

        outtakeT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20, 0, 0, adjustedF));
        outtakeB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20, 0, 0, adjustedF));


        /*
         * We set the left feeder servo to reverse so that they both work to feed the ball into the robot.
         */
    }

    public void intake(double i){
        intake.setPower(i);
    }

    public void setGatePosition(double i) {
        gate.setPosition(i);
    }

    public void stopLaunch(){
        launchState = IntakeV2.LaunchState.IDLE;
        outtakeT.setPower(0);
        outtakeB.setPower(0);
        // change set positions to whatever
    }

    public void testLaunch() {
        outtakeT.setPower(0.3);
        outtakeB.setPower(0.3);
    }

    public void setTurret(double i) {
        turretL.setPosition(i);
        turretR.setPosition(i);
    }
    public double getTurretPos(){
        return turretL.getPosition();
    }

    public void setActuatorPos(double i){ angle.setPosition(i); }

    public double getActuatorPos(){return angle.getPosition();}



    //launch method using state machine concept
    public void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = IntakeV2.LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                outtakeT.setVelocity(LAUNCHER_TARGET_VELOCITY);
                outtakeB.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (getLauncherVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = IntakeV2.LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                setGatePosition(1);  //set position, change later
                feederTimer.reset();
                launchState = IntakeV2.LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    stopLaunch();
                    launchState = IntakeV2.LaunchState.IDLE;
                }
                break;
        }//closes switch
    }//closes method

    public LaunchState getLaunchState(){
        return launchState;
    }


    //accessor method for OWMotor’s velocity
    public double getLauncherVelocity(){
        return (Math.abs(outtakeT.getVelocity())
        );
    }

}//closes class




