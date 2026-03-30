package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class MecanumDrive {

    //declare private instance variables for each motor of drivetrain
    private DcMotor flMotor = null;
    private DcMotor frMotor = null;
    private DcMotor blMotor = null;
    private DcMotor brMotor = null;

    //half park
    private DcMotor tilt;

    private IMU imu;

    //private Servo blinky;

    private double absoluteValueLA = -164;// 153 red || -153 blue || 164 red/blue
    private double LA = absoluteValueLA;

    //initialize motors, set directions and mode
    public MecanumDrive(HardwareMap hwMap){
        flMotor = hwMap.get(DcMotor.class, "FL");
        frMotor = hwMap.get(DcMotor.class, "FR");
        blMotor = hwMap.get(DcMotor.class, "BL");
        brMotor = hwMap.get(DcMotor.class, "BR");

        //tilt park
        tilt = hwMap.get(DcMotor.class, "tilt");

        imu = hwMap.get(IMU.class,"imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
        //imu.resetYaw();

        //blinky = hwMap.get(Servo.class, "blinky");


        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.FORWARD);

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    //helper method private to this class to ensure motor power is never set higher than 1.0
    private void setPowers(double fl, double fr, double bl, double br){
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(fl));
        maxSpeed = Math.max(maxSpeed, Math.abs(fr));
        maxSpeed = Math.max(maxSpeed, Math.abs(bl));
        maxSpeed = Math.max(maxSpeed, Math.abs(br));

        fl /= maxSpeed;
        fr /= maxSpeed;
        bl /= maxSpeed;
        br /= maxSpeed;

        flMotor.setPower(fl);
        frMotor.setPower(fr);
        blMotor.setPower(bl);
        brMotor.setPower(br);
    }

    //public method to be called from opMode
    public void drive(double forward, double right, double rotate){
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        //adjusted for bad rotation
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        //calling helper method
        setPowers(fl, fr, bl, br);
    }

    public void slowTurn(double rotate) {
        setPowers(rotate, -rotate, rotate, -rotate);
    }

    public double getRobotAngle(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setHalfPark(double i){
        //tilt.setPosition(i);
    }

    public double getHalfPark() {
        //return tilt.getPosition();
        return 0;
    }

    public void resetRobotAngle(){
        imu.resetYaw();
    }

//    public void setLightColor(){
//        //launch angle is positive, so blue
//        if(LA > 0) {
//            if (getRobotAngle() > LA + 2) {
//                blinky.setPosition(.28);//show red
//            } else if (getRobotAngle() < LA - 2) {
//                blinky.setPosition(.611);//show blue
//            } else {
//                blinky.setPosition(.500);//show green
//            }
//        }else{//launch angle is negative so red
//            if (getRobotAngle() > LA + 2) {
//                blinky.setPosition(.611);//show blue
//            } else if (getRobotAngle() < LA - 2) {
//                blinky.setPosition(.28);//show red
//            } else {
//                blinky.setPosition(.500);//show green
//            }
//        }
//    }


    public void redLaunchAngle(){
        LA = absoluteValueLA*-1;
    }

    public void blueLaunchAngle(){
        LA = absoluteValueLA;
    }
     public double getLaunchAngle(){
        return LA;
     }




}


