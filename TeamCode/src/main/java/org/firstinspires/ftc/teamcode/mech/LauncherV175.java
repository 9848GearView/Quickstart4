package org.firstinspires.ftc.teamcode.mech;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; //extended DcMotor class for extra controls
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherV175 {

    final double FEED_TIME_SECONDS = 4; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our OWMotor motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the OWMotor should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 325;
    final double LAUNCHER_MIN_VELOCITY = 275;

     //Declare OpMode members.
    private DcMotorEx OWMotor = null;

    private CRServo ICLServo;
    private CRServo ICRServo;

    //added for 1.75
    private CRServo OGLServo;
    private CRServo OGRServo;

    private CRServo IHLServo;
    private CRServo IHRServo;


    private LaunchState launchState; //LaunchState enums are written below for launching switch

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our OWMotor motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }



    public LauncherV175(HardwareMap hwMap) {
        launchState = LaunchState.IDLE;
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        ICLServo = hwMap.get(CRServo.class, "ICLServo");
        ICRServo = hwMap.get(CRServo.class, "ICRServo");
        //added for 1.75, OG = Outtake Gate
        OGLServo = hwMap.get(CRServo.class, "OGLServo");
        OGRServo = hwMap.get(CRServo.class, "OGRServo");

        //IH = Intake Horizontal
        IHLServo = hwMap.get(CRServo.class, "IHLServo");
        IHRServo = hwMap.get(CRServo.class, "IHRServo");


        OWMotor = hwMap.get(DcMotorEx.class, "OWMotor");

        /*
         * Here we set our OWMotor to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        OWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting
         */
        OWMotor.setZeroPowerBehavior(BRAKE);

        ICLServo.setDirection(CRServo.Direction.REVERSE);
        ICRServo.setDirection(CRServo.Direction.FORWARD);

        OGLServo.setDirection(CRServo.Direction.FORWARD);
        OGRServo.setDirection(CRServo.Direction.REVERSE);

        IHLServo.setDirection(CRServo.Direction.REVERSE);
        IHRServo.setDirection(CRServo.Direction.FORWARD);

        OWMotor.setDirection(DcMotorEx.Direction.FORWARD);


        ICLServo.setPower(STOP_SPEED);
        ICRServo.setPower(STOP_SPEED);


        /*Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
         * This control method reads the current speed as reported by the motor's encoder and applies a
         * varying amount of power to reach, and then hold a target velocity. The FTC SDK calls this
         * control method "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where
         * you control the power applied to the motor directly.
         * Since the dynamics of a OWMotor wheel system varies greatly from those of most other FTC
         * mechanisms, we will also need to adjust the "PIDF" coefficients with some that are a better fit * for our application.
         */

        OWMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(2.5, .1, .2, .5));

        /*
         * We set the left feeder servo to reverse so that they both work to feed the ball into the robot.
         */
    }

    public void intake(double horizontal, double ramp, double gate){
        ICLServo.setPower(ramp);
        ICRServo.setPower(ramp);
        IHLServo.setPower(horizontal);
        IHRServo.setPower(horizontal);
        OGLServo.setPower(gate);
        OGRServo.setPower(gate);
    }

    private void launch(){
        ICLServo.setPower(FULL_SPEED);
        ICRServo.setPower(FULL_SPEED);
        IHLServo.setPower(FULL_SPEED);
        IHRServo.setPower(FULL_SPEED);
        OGLServo.setPower(FULL_SPEED);
        OGRServo.setPower(FULL_SPEED);

    }

    public void stopLaunch(){
        launchState = LaunchState.IDLE;
        ICLServo.setPower(STOP_SPEED);
        ICRServo.setPower(STOP_SPEED);
        IHLServo.setPower(STOP_SPEED);
        IHRServo.setPower(STOP_SPEED);
        OGLServo.setPower(STOP_SPEED);
        OGRServo.setPower(STOP_SPEED);
        OWMotor.setVelocity(STOP_SPEED);
    }

    //launch method using state machine concept
    public void launch(boolean shotRequested) {
            switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                OWMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (OWMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                launch();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    stopLaunch();
                    launchState = LaunchState.IDLE;
                }
                break;
        }//closes switch
    }//closes method

   // accessor method for launchState
    public LaunchState getLaunchState(){
        return launchState;
    }

    //accessor method for OWMotor’s velocity
    public double getLauncherVelocity(){
        return OWMotor.getVelocity();
    }

}//closes class








