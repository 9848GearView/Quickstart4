package org.firstinspires.ftc.teamcode.mech;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;


public class IntakeV2 {

    // intake
    private DcMotorEx intake;

    //outtake
    private DcMotorEx outtakeL;
    private DcMotorEx outtakeR;

    //sorter
    private Servo sort;

    //turret
    private Servo t1;
    private Servo t2;

    //transfer
    private Servo up1;
    private Servo up2;


    private Timer timer;
    private final int DBM = 1000;

    //n Steven and Alex code :) - Mrs. B moved here from their auto code
    final double FEED_TIME_SECONDS = 3;

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    //likely change
    final double LAUNCHER_MAX_VELOCITY= 325;//started at 1125//was725
    final double LAUNCHER_MIN_VELOCITY = 275;//started at 1075//was 675


    ElapsedTime feedTimer = new ElapsedTime();


    private LaunchState launchState;


    private enum  LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    public IntakeV2(HardwareMap hwMap){
        //intake
        intake = hwMap.get(DcMotorEx.class, "intake");

        //outtake
        outtakeL = hwMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hwMap.get(DcMotorEx.class, "outtakeR");

        //sorter
        sort = hwMap.get(Servo.class, "sort");

        //turret
        t1 = hwMap.get(Servo.class, "t1");
        t2 = hwMap.get(Servo.class, "t2");

        //transfer
        up1 = hwMap.get(Servo.class, "up1");
        up2 = hwMap.get(Servo.class, "up1");



        //whatever.setDirection(CRServo.Direction.FORWARD);

        /* i dont know what this means (connor)
        //Mrs. B brought in as it was missing from the initialization
        launchState = LaunchState.IDLE;

        OWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OWMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //OWMotor.setZeroPowerBehavior(BRAKE);

        ICLServo.setPower(STOP_SPEED);
        ICRServo.setPower(STOP_SPEED);

        OWMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,0,0,10));

         */


    }

    /*
    class intake extends TimerTask {
        double power;
        public intake(double power) {
            this.power = power;
        }
        @Override
        public void run() {
            intake.setPower(power);
        }
    }
    class spin extends TimerTask {
        double position;
        public spin(double position) {
            this.position = position;
        }
        @Override
        public void run() {
            spinny.setPosition(position);
        }
    }
    class transfer extends TimerTask {
        double position;
        public transfer(double position) {
            this.position = position;
        }
        public void run() {
            up1.setPosition(position);
            up2.setPosition(-position);
        }
    }
    class launch extends TimerTask {
        double power;
        public launch(double power) {
            this.power = power;

        }


        public void run() {
            outtakeL.setPower(power);
            outtakeR.setPower(-power);
        }
    }

     */


    public void intake(double i) {
        intake.setPower(i);
    }

    public void launch(double i) {
        outtakeL.setPower(i);
        outtakeR.setPower(-i);
    }

    public void stopLaunch(){
        outtakeL.setPower(0);
        outtakeR.setPower(0);    }

    //idk if this is how u do it but wtv
    public void sort(double i) {
        sort.setPosition(i);
    }

    //replace this with whatever the limelight code is
    public void turret(double pos1, double pos2) {
        t1.setPosition(pos1);
        t2.setPosition(pos2);
    }

    public void transfer(double i) {
        up1.setPosition(i);
        up2.setPosition(-i);
    }


    //partialy failed 1.5 attempt with timer tasks, will revisit in future - Mitch
    /*
    public void intakeTimer(){
        //timer.schedule(new moveChain(.5),0);

    }
    public void stopIntake(){
        //timer.schedule(new moveChain(0),0);
    }

    public void launchTimer (){
        timer.schedule(new moveChain(.5),2 * DBM);
        timer.schedule(new moveLaunch(.6), 0);
        timer.schedule(new moveChain(0), 3 * DBM);
        timer.schedule(new moveLaunch(0), 4 * DBM);
    }
     */

    /*
    //Mrs. B brought in code from Alex & Steven coding in their auto
    public void launchSmarter(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                OWMotor.setVelocity(LAUNCHER_MAX_VELOCITY);
                if ((OWMotor.getVelocity() > LAUNCHER_MIN_VELOCITY)) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                ICLServo.setPower(FULL_SPEED);
                ICRServo.setPower(FULL_SPEED);
                feedTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feedTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    ICLServo.setPower(STOP_SPEED);
                    ICRServo.setPower(STOP_SPEED);
                }
                break;
        }//closes switch
    }//closes launchSmarter Method


     */
    public double getLauncherVelocity(){
        return ((outtakeL.getVelocity() + outtakeR.getVelocity()) / 2);
    }



}//closes class




