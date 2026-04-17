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


import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Servo blinky2;


    //distance sensor
    private DistanceSensor distanceSensorGate;
    private DistanceSensor distanceSensorIntake;
    private Servo distanceGateLight;
    private Servo distanceIntakeLight;

    //light
    //I2cDeviceSynch prism;

    //turret
    private Servo turretL;
    private Servo turretR;
    //private AnalogInput encoderL;
    //private AnalogInput encoderR;
    private RTPAxon turretRotation;
    private double leftLim = .65;
    private boolean leftLimReached = false;
    private double rightLim = .35;
    private boolean rightLimReached = false;
    private boolean movingRight = false;

    private double goalBX = 12;
    private double goalRX = 132;
    private double goalY = 136;

    ElapsedTime feederTimer = new ElapsedTime();

    //likely change
    final double LAUNCHER_TARGET_VELOCITY = 1365;//started at 1125//was725//Mrs.b changed to 850
    final double LAUNCHER_MIN_VELOCITY = 1100;//started at 1075//was 675//mrs B changed to 800
    final double farVelocity = 1480;

    private String launchStatus;


    PIDFCoefficients flywheelCoeffs = new PIDFCoefficients(36, 0, 0, 18);//possible values: 100p 12.917 f good?, 15.917 little overshoot//60.917


    ElapsedTime feedTimer = new ElapsedTime();


    public IntakeV3(HardwareMap hwMap) {

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
        blinky = hwMap.get(Servo.class, "blinky");
        blinky2 = hwMap.get(Servo.class, "blinky2");

        distanceGateLight = hwMap.get(Servo.class, "gateLight");
        distanceIntakeLight = hwMap.get(Servo.class, "intakeLight");


        //distance sensor
        distanceSensorGate = hwMap.get(DistanceSensor.class, "distanceGate");
        distanceSensorIntake = hwMap.get(DistanceSensor.class, "distanceIntake");

        //light
        //prism = hwMap.get(I2cDeviceSynch.class, "prism");
        //prism.setI2cAddress(I2cAddr.create7bit(0x38)); // Default Prism I2C address is usually 0x24
        //prism.engage();

        //turret

        turretL = hwMap.get(Servo.class, "turretL");
        turretR = hwMap.get(Servo.class, "turretR");

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
//
//    public void setColor(int r, int g, int b) {
//        // Replace 0x01, 0x02, 0x03 with the actual register addresses for R, G, B
//        //prism.write8(0x01, r); // Red
//        //prism.write8(0x02, g); // Green
//        //prism.write8(0x03, b); // Blue
//    }

//set light color
    public void setLightColor() {
        if (gate.getPosition() == 0) {
            //setColor(255,0,0);
            blinky.setPosition(.500); //open, green
            blinky2.setPosition(.500);
        }
        if (gate.getPosition() == .15) {
            blinky.setPosition(.28);
            blinky2.setPosition(.28);//closed, red
        }
        if (getDistanceGate() < 10){
            distanceGateLight.setPosition(.388);
        } else {
            distanceGateLight.setPosition(0);
        }

        if (getDistanceIntake() < 10){
            distanceIntakeLight.setPosition(.388);
        } else {
            distanceIntakeLight.setPosition(0);
        }
    }

    public void setGateLightColor(boolean isBall){
        if(isBall){
            distanceGateLight.setPosition(0.722);
        } else {
            distanceGateLight.setPosition(0);
        }
    }

    public void setIntakeLightColor(boolean isBall){
        if(isBall){
            distanceIntakeLight.setPosition(0.722);
        } else {
            distanceIntakeLight.setPosition(0);
        }
    }

    //set actuator position
    public void setActuatorPos(double i) {
        angle.setPosition(i);
    }

    //set turret position
    public void setTurret(double i) {
        if(!limsReached(i)) {
            turretL.setPosition(i);
            turretR.setPosition(i);
        }
    }

    public boolean limsReached(double i){
        if(i >=.75| i <= .25){
            return true;
        }
        return false;
    }

    //stop launch motors
    public void stopLaunch() {

        outtakeT.setVelocity(0);
        outtakeB.setVelocity(0);
        // change set positions to whatever
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
        outtakeT.setVelocity(1150);
        outtakeB.setVelocity(1150);
        launchStatus = "far";
    }//closes method

    //GET AND SET METHODS
    public void setVelocity(double p) {
        outtakeT.setVelocity(p);
        outtakeB.setVelocity(p);
    }

    //get turret position
    public double getTurretPos() {
        return turretL.getPosition();
    }

    //get actuator position
    public double getActuatorPos(){return angle.getPosition();}

    //accessor method for OWMotor’s velocity
    public double getLauncherVelocity(){
        return (Math.abs(outtakeT.getVelocity())
        );
    }

    public double getActuatorPosition() {
        return angle.getPosition();
    }

    public double getDistanceGate() {return distanceSensorGate.getDistance(DistanceUnit.CM);}
    public double getDistanceIntake() {return distanceSensorIntake.getDistance(DistanceUnit.CM);}

    public void feed(Telemetry telemetry){
        telemetry.addData("Launcher Pos", getTurretPos());
        telemetry.addData("Elevator Actuation",getActuatorPos());
        telemetry.addData("Actuator Position", getActuatorPosition());
        telemetry.addLine();
        telemetry.addData("Gate Distance (cm)", getDistanceGate());
        telemetry.addData("Intake Distance (cm)", getDistanceIntake());
    }

}//closes class




