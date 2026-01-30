package org.firstinspires.ftc.teamcode.mech;
//

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;


public class IntakeV175 {

    //instance variables/class members
    private CRServo ICLServo;
    private CRServo ICRServo;

    //added for 1.75
    private CRServo OGLServo;
    private CRServo OGRServo;

    private CRServo IHLServo;
    private CRServo IHRServo;

    private DcMotorEx OWMotor;

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

    public IntakeV175(HardwareMap hwMap){
        // IC = Intake Chain
        ICLServo = hwMap.get(CRServo.class, "ICLServo");
        ICRServo = hwMap.get(CRServo.class, "ICRServo");
        //added for 1.75, OG = Outtake Gate
        OGLServo = hwMap.get(CRServo.class, "OGLServo");
        OGRServo = hwMap.get(CRServo.class, "OGRServo");

        //IH = Intake Horizontal
        IHLServo = hwMap.get(CRServo.class, "IHLServo");
        IHRServo = hwMap.get(CRServo.class, "IHRServo");


        // OW = Outtake Wheel
        OWMotor = hwMap.get(DcMotorEx.class, "OWMotor");

        //sets direction of outtake chain
        ICLServo.setDirection(CRServo.Direction.REVERSE);
        ICRServo.setDirection(CRServo.Direction.FORWARD);

        OGLServo.setDirection(CRServo.Direction.FORWARD);
        OGRServo.setDirection(CRServo.Direction.REVERSE);

        IHLServo.setDirection(CRServo.Direction.REVERSE);
        IHRServo.setDirection(CRServo.Direction.FORWARD);


        //Mrs. B brought in as it was missing from the initialization
        launchState = LaunchState.IDLE;

        OWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OWMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //OWMotor.setZeroPowerBehavior(BRAKE);

        ICLServo.setPower(STOP_SPEED);
        ICRServo.setPower(STOP_SPEED);

        //OWMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,0,0,10));


    }
    //failed attempt at timer tasks on 1.5, will revisit in future - Mitch
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
            OWMotor.setPower(power);

        }
    }


    //spit out whatever is in holding
    public void outtake(){

    }
    //launches
    //no reason for the number to be a neg value
    public void launch(double i){
        OWMotor.setPower(i);
    }

    public void gate(double i){
        OGLServo.setPower(i);
        OGRServo.setPower(i);
    }

    public void stopLaunch(){
        OWMotor.setPower(0);
    }

    public void intake(double horizontal, double ramp, double gate){
        ICLServo.setPower(ramp);
        ICRServo.setPower(ramp);
        IHLServo.setPower(horizontal);
        IHRServo.setPower(horizontal);
        OGLServo.setPower(gate);
        OGRServo.setPower(gate);
    }


    public void upToSpeed(double i){
        OWMotor.setPower(i);
    }

    public class IntakeAuto extends TimerTask {
        double power;

        public IntakeAuto(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            ICLServo.setPower(power);
            ICRServo.setPower(power);
            IHLServo.setPower(power);
            IHRServo.setPower(power);
        }
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

    public double getLauncherVelocity(){
        return OWMotor.getVelocity();
    }



}//closes class




