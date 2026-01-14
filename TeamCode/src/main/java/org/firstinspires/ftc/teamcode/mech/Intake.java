package org.firstinspires.ftc.teamcode.mech;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;


public class Intake{

    //instance variables/class members
    private CRServo ICLServo;
    private CRServo ICRServo;

    private DcMotorEx OWLMotor;
    private DcMotorEx OWRMotor;
    private Timer timer;
    private final int DBM = 1000;

    //n Steven and Alex code :) - Mrs. B moved here from their auto code
    final double FEED_TIME_SECONDS = 3;

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

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

    public Intake(HardwareMap hwMap){

        ICLServo = hwMap.get(CRServo.class, "ICLServo");
        ICRServo = hwMap.get(CRServo.class, "ICRServo");

        OWLMotor = hwMap.get(DcMotorEx.class, "OWLMotor");
        OWRMotor = hwMap.get(DcMotorEx.class, "OWRMotor");

        ICLServo.setDirection(CRServo.Direction.FORWARD);
        ICRServo.setDirection(CRServo.Direction.REVERSE);

        //Mrs. B brought in as it was missing fromt he initialization
        launchState = LaunchState.IDLE;

        OWLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OWRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        OWLMotor.setDirection(DcMotorEx.Direction.FORWARD);
        OWRMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //OWLMotor.setZeroPowerBehavior(BRAKE);
        //
        // OWRMotor.setZeroPowerBehavior(BRAKE);

        ICLServo.setPower(STOP_SPEED);
        ICLServo.setPower(STOP_SPEED);

        OWLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,0,0,10));
        OWRMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,0,0,10));

    }

    class moveChain extends TimerTask {
        double power;
        public moveChain(double power) {
            this.power = power;

        }
        public void run() {
            ICLServo.setPower(power);
            ICRServo.setPower(power);
        }
    }
    class moveLaunch extends TimerTask {
        double power;
        public moveLaunch(double power) {
            this.power = power;

        }
        public void run() {
            OWLMotor.setPower(power);
            OWRMotor.setPower(power);
        }
    }





    /*public String getPosition(){
        return "CAL: " + IWLMotor.getCurrentPosition() + "CAR: " + IWRMotor.getCurrentPosition();
    }*/

    public void outtake(){

    }


    //NOT SET YET

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
    public void launch(double i){
        //separated to add boost to left
        OWLMotor.setPower(i +.4 );//+.4
        OWRMotor.setPower(i);
    }
    public void stopLaunch(){
        //separated
        OWLMotor.setPower(0);
        OWRMotor.setPower(0);
    }
    public void intake(double i){
        ICLServo.setPower(i);
        ICRServo.setPower(i);
    }

    //Mrs. B brought in code from Alex & Steven coding in their auto
    public void launchSmarter(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                OWLMotor.setVelocity(LAUNCHER_MAX_VELOCITY);
                OWRMotor.setVelocity(LAUNCHER_MAX_VELOCITY);
                if ((OWLMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) && (OWRMotor.getVelocity() > LAUNCHER_MIN_VELOCITY)) {//changed && to ||
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

    public double getLeftLauncherVelocity(){
        return OWLMotor.getVelocity();
    }

    public double getRightLauncherVelocity(){
        return OWRMotor.getVelocity();
    }

}//closes class




