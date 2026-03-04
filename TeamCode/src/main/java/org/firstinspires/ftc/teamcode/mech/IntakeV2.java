package org.firstinspires.ftc.teamcode.mech;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
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
    private DcMotorEx transfer;

    //outtake
    private DcMotorEx outtakeT;
    private DcMotorEx outtakeB;

    //angle
    private Servo angle;

    //gate
    private Servo gate;
    private Servo blinky;
    private boolean shot;



    //turret
    private Servo turretL;
    private Servo turretR;
    private double tInc = .001;
    private double leftLim = .65;
    private boolean leftLimReached = false;
    private double rightLim = .35;
    private boolean rightLimReached = false;
    private boolean movingRight = false;

    private double goalBX = 12;
    private double goalRX = 132;
    private double goalY = 136;

    ElapsedTime feederTimer = new ElapsedTime();

    private Timer timer;
    private final int DBM = 1000;

    //n Steven and Alex code :) - Mrs. B moved here from their auto code
    final double FEED_TIME_SECONDS = 3;

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    /*private double voltage = batteryVoltageSensor.getVoltage();

    double adjustedFFar = 16 * (12/voltage);
    double adjustedFClose = 15.5 * (12/voltage);

     */


    //likely change
    final double LAUNCHER_TARGET_VELOCITY= 1600;//started at 1125//was725//Mrs.b changed to 850
    final double LAUNCHER_MIN_VELOCITY = 1550;//started at 1075//was 675//mrs B changed to 800

    private String launchStatus;

    PIDFCoefficients farCoeffs = new PIDFCoefficients(30, 0, 0, 11.5);
    PIDFCoefficients closeCoeffs = new PIDFCoefficients(5, 0, 0, 6);


    ElapsedTime feedTimer = new ElapsedTime();


    private LaunchState launchState;


    public enum  LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    public IntakeV2(HardwareMap hwMap) {
        launchState = LaunchState.IDLE;

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        //intake
        intake = hwMap.get(DcMotorEx.class, "intake");
        transfer = hwMap.get(DcMotorEx.class, "transfer");

        //outtake
        outtakeT = hwMap.get(DcMotorEx.class, "outtakeT");
        outtakeB = hwMap.get(DcMotorEx.class, "outtakeB");

        
        //angle
        angle = hwMap.get(Servo.class, "angle");
        
        //gate
        gate = hwMap.get(Servo.class, "gate");
        blinky = hwMap.get(Servo.class, "blinky");



        //turret
        turretL = hwMap.get(Servo.class, "turretL");
        turretR = hwMap.get(Servo.class, "turretR");

        shot = false;

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

        /*Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
         * This control method reads the current speed as reported by the motor's encoder and applies a
         * varying amount of power to reach, and then hold a target velocity. The FTC SDK calls this
         * control method "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where
         * you control the power applied to the motor directly.
         * Since the dynamics of a OWMotor wheel system varies greatly from those of most other FTC
         * mechanisms, we will also need to adjust the "PI++++DF" coefficients with some that are a better fit * for our application.
         */

        outtakeT.setVelocity(0);
        outtakeB.setVelocity(0);

        /*
         * We set the left feeder servo to reverse so that they both work to feed the ball into the robot.
         */
    }

    //intake
    public void intake(double i){
        intake.setPower(i);
        transfer.setPower(-i);
    }

    //set gate position
    public void setGatePosition(double i) {
        gate.setPosition(i);
    }

    //set light color
    public void setLightColor(){
        if (gate.getPosition() ==.25){
            blinky.setPosition(.500); //open, green
        }
        if (gate.getPosition() == .38){
            blinky.setPosition(.28); //closed, red
        }
    }

    //set actuator position
    public void setActuatorPos(double i){ angle.setPosition(i); }

    //set turret position
    public void setTurret(double i) {
        turretL.setPosition(i);
        turretR.setPosition(i);
    }

    public void setLeftTurret(double i){
        turretL.setPosition(i);
    }

    public void setRightTurret(double i){
        turretR.setPosition(i);
    }

    //limelight scanning
    public void scanTurret(){
        //update if I've reached right or left limit
        leftLimReached = getTurretPos() > leftLim;
        rightLimReached = getTurretPos() < rightLim;

        //scan right (towards 0.35 right limit)
        if(movingRight){
            if(rightLimReached){
                setTurret(getTurretPos() + tInc);
                movingRight = false;
            }else {
                setTurret(getTurretPos() - tInc);
                movingRight = true;
            }
        }
            //scan left (towards 0.65 right limit)
        if(!movingRight){
            if(leftLimReached){
                setTurret(getTurretPos() - tInc);
                movingRight = true;
            }else{
                setTurret(getTurretPos() + tInc);
                movingRight = false;
            }
        }
    }

    //stop launch motors
    public void stopLaunch(){
        launchState = LaunchState.IDLE;
        outtakeT.setVelocity(0);
        outtakeB.setVelocity(0);
        // change set positions to whatever
    }

    //start spinning up outtake motors
    public void toggleSpinUp() {
        if (launchState == LaunchState.IDLE) {
            launchState = LaunchState.SPIN_UP;
        } else {
            stopLaunch();
        }
    }

    //request gate open
    //not used, outtake is on all match and only thing that's necessary is to use right bumper to close and open gate
    public void requestLaunch() {
        if (launchState == LaunchState.SPIN_UP &&
                getLauncherVelocity() > LAUNCHER_MIN_VELOCITY) {
            launchState = LaunchState.LAUNCH;
        }
    }

    //pidf coefficients and velocity for big triangle
    public void launchClose() {
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, closeCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, closeCoeffs);
        outtakeT.setVelocity(1450);
        outtakeB.setVelocity(1450);
        launchStatus = "close";
    }//closes method

    //pidf coefficients and velocity for small triangle
    public void launchFar() {
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, farCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, farCoeffs);
        outtakeT.setVelocity(1930);
        outtakeB.setVelocity(1930);
        launchStatus = "far";
    }//closes method

    //big triangle shooting during auto
    public void launchAutoClose(boolean b) {
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, closeCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, closeCoeffs);
        switch (launchState) {
            case IDLE:
                setGatePosition(.5);
                outtakeT.setVelocity(0);
                outtakeB.setVelocity(0);
                if(b) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                outtakeT.setVelocity(1450);
                outtakeB.setVelocity(1450);
                if (getLauncherVelocity() > 1400) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feederTimer.reset();
                launchState = IntakeV2.LaunchState.LAUNCHING;
                break;
            case LAUNCHING: // not used, just use stopLaunch method manually
                intake(1);
                setGatePosition(0);
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    intake(0);                    stopLaunch();
                    launchState = LaunchState.IDLE;
                }
                break;
        }//closes switch
    }//closes method

    //small triangle shooting during auto
    public void launchAutoFar(boolean b) {
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, farCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, farCoeffs);
        switch (launchState) {
            case IDLE:
                setGatePosition(.5);
                outtakeT.setVelocity(0);
                outtakeB.setVelocity(0);
                if(b) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                outtakeT.setVelocity(1930);
                outtakeB.setVelocity(1930);
                if (getLauncherVelocity() > 1880) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feederTimer.reset();
                launchState = IntakeV2.LaunchState.LAUNCHING;
                break;
            case LAUNCHING: // not used, just use stopLaunch method manually
                intake(1);
                setGatePosition(0);
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    intake(0);
                    stopLaunch();
                    launchState = LaunchState.IDLE;
                }
                break;
        }//closes switch
    }//closes method

    //GET AND SET METHODS

    //get turret position
    public double getTurretPos(){
        return turretL.getPosition();
    }

    //checks if the turret has reached the rightmost limit
    public boolean getRightLimitReached(){
        return rightLimReached;
    }

    //checks if the turret has reached the rightmost limit
    public boolean getLeftLimitReached(){
        return leftLimReached;
    }

    //get actuator position
    public double getActuatorPos(){return angle.getPosition();}

    //find what LaunchState the robot's in
    public LaunchState getLaunchState(){
        return launchState;
    }

    public double getMinVelocity() { return LAUNCHER_MIN_VELOCITY;}

    //accessor method for OWMotor’s velocity
    public double getLauncherVelocity(){
        return (Math.abs(outtakeT.getVelocity())
        );
    }

    public double getActuatorPosition() {
        return angle.getPosition();
    }

    public String getLaunchStatus() {return launchStatus;}

    public boolean hasFinishedShot() {
        return launchState == LaunchState.IDLE && feederTimer.seconds() > 0.1;
    }

}//closes class




