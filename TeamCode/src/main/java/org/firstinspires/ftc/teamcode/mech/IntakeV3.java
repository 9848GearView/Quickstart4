package org.firstinspires.ftc.teamcode.mech;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.Subsystem;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;


public class IntakeV3 implements Subsystem{

    //velocity
    //private VoltageSensor batteryVoltageSensor;

    // intake
    private DcMotorEx intake;

    //outtake
    private DcMotorEx outtakeT;
    private DcMotorEx outtakeB;

    //angle
    private Servo angle;

    //gate
    private Servo gate;
    private Servo blinky;

    //distance sensor
    private DistanceSensor distanceSensorGate;
    private DistanceSensor distanceSensorIntake;

    //light
    //I2cDeviceSynch prism;

    //turret
    //private CRServo turretL;
    //private CRServo turretR;
    //private AnalogInput encoderL;
    //private AnalogInput encoderR;
    private RTPAxon turretRotation;
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

    //private double voltage = batteryVoltageSensor.getVoltage();

    //double adjustedFFar = 16 * (12/voltage);
    //double adjustedFClose = 15.5 * (12/voltage);


    //likely change
    final double LAUNCHER_TARGET_VELOCITY = 1365;//started at 1125//was725//Mrs.b changed to 850
    final double LAUNCHER_MIN_VELOCITY = 1100;//started at 1075//was 675//mrs B changed to 800
    final double farVelocity = 1480;

    private String launchStatus;
    
    
    PIDFCoefficients flywheelCoeffs = new PIDFCoefficients(36, 0, 0, 18);//possible values: 100p 12.917 f good?, 15.917 little overshoot//60.917


    ElapsedTime feedTimer = new ElapsedTime();


    private LaunchState launchState;


    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    public IntakeV3(HardwareMap hwMap) {
        launchState = LaunchState.IDLE;

        //batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        //intake
        intake = hwMap.get(DcMotorEx.class, "intake");

        //outtake
        outtakeT = hwMap.get(DcMotorEx.class, "outtakeT");
        outtakeB = hwMap.get(DcMotorEx.class, "outtakeB");

        //angle
        angle = hwMap.get(Servo.class, "actuator");

        //gate
        gate = hwMap.get(Servo.class, "gate");
        //blinky = hwMap.get(Servo.class, "blinky");

        //distance sensor
        distanceSensorGate = hwMap.get(DistanceSensor.class, "distanceGate");
        distanceSensorIntake = hwMap.get(DistanceSensor.class, "distanceIntake");

        //light
        //prism = hwMap.get(I2cDeviceSynch.class, "prism");
        //prism.setI2cAddress(I2cAddr.create7bit(0x38)); // Default Prism I2C address is usually 0x24
        //prism.engage();

        //turret


        turretRotation = new RTPAxon(hwMap);
        turretRotation.setMaxPower(.7);
        turretRotation.setPidCoeffs(0.015, 0.001, 0.0005);  // change


        outtakeT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeT.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeB.setDirection(DcMotorEx.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting
         */
        outtakeT.setZeroPowerBehavior(FLOAT); //these were float if brake makes it cry
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

    // Wrapper to update the PID and encoder logic
    public void updateTurret() {
        turretRotation.update();
    }

    public double getTurretAngle() {
        return turretRotation.getTotalRotation();
    }

    public RTPAxon getRTPAxon(){
        return turretRotation;
    }

    //intake
    public void intake(double i) {
        intake.setPower(i);
    }

    //set gate position
    public void setGatePosition(double i) {
        gate.setPosition(i);
    }

    public double getGatePosition() {
        return gate.getPosition();
    }

    public void setColor(int r, int g, int b) {
        // Replace 0x01, 0x02, 0x03 with the actual register addresses for R, G, B
        //prism.write8(0x01, r); // Red
        //prism.write8(0x02, g); // Green
        //prism.write8(0x03, b); // Blue
    }

//set light color
    public void setLightColor() {
        if (gate.getPosition() == .25) {
            //setColor(255,0,0);
        }
        if (gate.getPosition() == .38) {
            //blinky.setPosition(.28); //closed, red
        }
        if (getDistanceGate() < 15){
            //setColor()
        }
    }

    //set actuator position
    public void setActuatorPos(double i) {
        angle.setPosition(i);
    }

    public double getVoltage(){
        return turretRotation.getVoltage();
    }

    //set turret position
    /*public void setTurret(double i) {
        turretL.setPosition(i);
        turretR.setPosition(i);
    }

    //limelight scanning
    public void scanTurret() {
        //update if I've reached right or left limit
        leftLimReached = getTurretPos() > leftLim;
        rightLimReached = getTurretPos() < rightLim;

        //scan right (towards 0.35 right limit)
        if (movingRight) {
            if (rightLimReached) {
                setTurret(getTurretPos() + tInc);
                movingRight = false;
            } else {
                setTurret(getTurretPos() - tInc);
                movingRight = true;
            }
        }
        //scan left (towards 0.65 right limit)
        if (!movingRight) {
            if (leftLimReached) {
                setTurret(getTurretPos() - tInc);
                movingRight = true;
            } else {
                setTurret(getTurretPos() + tInc);
                movingRight = false;
            }
        }

    }
    
     */

    //stop launch motors
    public void stopLaunch() {
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
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeT.setVelocity(1270);
        outtakeB.setVelocity(1270);
        launchStatus = "close";
    }//closes method

    //pidf coefficients and velocity for small triangle
    public void launchFar() {
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeT.setVelocity(farVelocity);// before adjustments velocity was 1620
        outtakeB.setVelocity(farVelocity);//
        launchStatus = "far";
    }//closes method

    //big triangle shooting during auto
    public void launchAutoClose() {
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeT.setVelocity(1200);
        outtakeB.setVelocity(1200);
        launchStatus = "close";

    }//closes method

    //small triangle shooting during auto
    public void launchAutoFar() {
        outtakeT.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeB.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelCoeffs);
        outtakeT.setVelocity(1460);
        outtakeB.setVelocity(1460);
        launchStatus = "far";
    }//closes method

    //GET AND SET METHODS
    public void setVelocity(double p) {
        outtakeT.setVelocity(p);
        outtakeB.setVelocity(p);
    }

    /*public void setMaxPowerAxon(double p) {
        axon.setMaxPower(p);
    }
     */

    //get turret position
    public double getTurretPos() {
        //return turretL.getPosition();
        return 0;
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

    public double getDistanceGate() {return distanceSensorGate.getDistance(DistanceUnit.CM);}
    public double getDistanceIntake() {return distanceSensorIntake.getDistance(DistanceUnit.CM);}

    public boolean hasFinishedShot() {
        return launchState == LaunchState.IDLE && feederTimer.seconds() > 0.1;
    }

}//closes class




