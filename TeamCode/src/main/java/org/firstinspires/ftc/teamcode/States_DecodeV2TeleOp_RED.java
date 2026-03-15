
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;
import org.firstinspires.ftc.teamcode.mech.RedLimelightAutoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//guarantee this wont work whatsoever

@TeleOp(name="States_RED-DecodeV2TeleOp", group="Iterative OpMode")
public class States_DecodeV2TeleOp_RED extends OpMode {
    private Limelight3A camera;
    MecanumDrive chassis = null;
    IntakeV2 cannon = null;
    RedLimelightAutoAim visionAid = null;

    //ColorSensor colSens = null;



    ColorSensor.detectedColor detectedColor;


    private boolean dPadUpPressed;


    private boolean rStickPressed;
    private boolean oldrStickPressed;

    //booleans for a button
    private boolean aPressed;
    private boolean oldAPressed;
    private boolean intakeOn = true;

    //booleans for down
    private boolean dPadDownPressed;
    private boolean oldDPadDownPressed;

    private boolean rBumperPressed;
    private boolean oldRBumperPressed;

    //booleans for turret
    private boolean lBumperPressed;
    private boolean oldLBumperPressed;
    private String LState;
    private boolean tLock;
    private boolean pushDown = true;

    //booleans for b button
    private boolean bPressed;
    private boolean oldBPressed;
    private boolean gateOn = true;

    //booleans for x button
    private boolean xPressed;
    private boolean oldXPressed;
    private boolean launchTrigger = false;

    //booleans for y
    private boolean yPressed;
    private boolean oldYPressed;
    private boolean actuatorIsDown;

    private double leftX;
    private double leftY;
    private double rightX;

    public double goalDistance;

    private boolean far;
    private boolean close;

    private double tInc;

    private double velMPS;


    @Override
    public void init(){
        chassis = new MecanumDrive(hardwareMap);
        cannon = new IntakeV2(hardwareMap);
        visionAid = new RedLimelightAutoAim(hardwareMap);
        camera = hardwareMap.get(Limelight3A.class,"limabean");

        camera.pipelineSwitch(1);
        camera.setPollRateHz(90);
        chassis.setHalfPark(0.45);
        cannon.setGatePosition(.38);
        cannon.setLightColor();
        cannon.setTurret(.5);
        cannon.setActuatorPos(.53);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(36,20,Math.toRadians(90)));

        tLock = false;
        velMPS = 0;
        //colSens = new ColorSensor(hardwareMap);
        //chassis.resetRobotAngle();//should be commented out to run teleOp after Auto & keep angle
    }
    @Override
    public void start(){
        camera.start();
    }
    @Override
    public void loop(){

        visionAid.update();

        dPadUpPressed = gamepad2.dpad_up;

        //controlled turning
        rStickPressed = gamepad1.right_stick_button;

        dPadDownPressed = gamepad2.dpad_down;


        aPressed = gamepad2.a;
        bPressed = gamepad2.b;
        
        xPressed = gamepad2.x;

        lBumperPressed = gamepad2.left_bumper;
        rBumperPressed = gamepad2.right_bumper;


        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        tInc = 0.01;

        if(cannon.getGatePosition() == .38){
            gateOn = true;
        }

        if(cannon.getGatePosition() == .25){
            gateOn = false;
        }

        if(gamepad1.x){
            chassis.setHalfPark(0.10);
        }

        //intake
        if (gamepad2.a && !oldAPressed){
            intakeOn = !intakeOn;
            if(intakeOn){
                cannon.intake(0);
                cannon.transfer(0);
            }else {
                cannon.intake(1);
                if(gateOn){
                    cannon.transfer(0.5);
                } else{
                    cannon.transfer(1);
                }
            }
        }


        //spits out balls
        if(gamepad2.dpad_down){
            pushDown = !pushDown;
            if(pushDown) {
                cannon.intake(0);
            } else {
                cannon.intake(-.4);
            }
        }

        //intake wheel end
        // (FI) color sensor begin
        //detectedColor = colSens.getDetectedColor(telemetry);
        // color sensor end
        //manual aim

        if(gamepad1.y) {
            follower.setX(9.713344316095563);
            follower.setY(9.186161449752879);
            follower.setHeading(180);
        }
        goalDistance = Math.sqrt(Math.pow(follower.getPose().getX() - 4,2) + Math.pow(140 - follower.getPose().getY(),2));

        if(gamepad2.dpad_left) {
            //if(cannon.getTurretPos() < 1 && cannon.getTurretPos() > 0) {
            cannon.setTurret(cannon.getTurretPos() + tInc);
            //}
        }
        if(gamepad2.dpad_right) {
            // if (cannon.getTurretPos() < 1 && cannon.getTurretPos() > 0) {
            cannon.setTurret(cannon.getTurretPos() - tInc);
            // }
        }
        if(gamepad2.dpad_up) {
            cannon.setTurret(.5);
        }


//        //auto-aim
        // why is ts the same button as gate bro
//        if(cannon.getLaunchStatus() == null){
//            LState = "mid";
//        } else if (cannon.getLaunchStatus().equals("close")){
//            LState = "close";
//        } else if(cannon.getLaunchStatus().equals("far")){
//            LState = "far";
//        }

        if( bPressed && !oldBPressed) {
            tLock = !tLock; // reminder to find a way to turn this off
        }
        if (tLock) {
            if (visionAid.hasTarget()){

                float Kp = -0.0004f; //proportional control constant
                double feedForward = ((rightX + leftX)/2.0) * .005;
                double tx = visionAid.getTx() -.5;
                double botCorr = (Kp * tx) - feedForward;
                if(Math.abs(tx) > .5) {
                    cannon.setTurret(cannon.getTurretPos() + botCorr);
                }
            }
        }
        //end auto aim

        //altitude actuator
        if (gamepad2.left_trigger > 0.1) {
            //shoot close
            cannon.launchClose();
            cannon.setActuatorPos(.8); //.53
        }
        if (gamepad2.right_trigger > 0.1) {
            // shoot far
            cannon.launchFar();
            cannon.setActuatorPos(1);
        }

        if(gamepad2.x) {
            cannon.stopLaunch();
        }

        //test launch in case break
        if(rBumperPressed && !oldRBumperPressed){
            cannon.setGatePosition(.38);
            cannon.setLightColor();
            oldRBumperPressed = true;
        }

        if(lBumperPressed && oldRBumperPressed){
            cannon.setGatePosition(0.25);
            cannon.setLightColor();
            oldRBumperPressed = false;
        }


        //Slow turn code :D
        if (gamepad1.right_bumper){
            chassis.slowTurn(0.1);
        }
        else if(gamepad1.left_bumper){
            chassis.slowTurn(-0.1);
        }
        else {
            chassis.drive(-leftY, leftX, rightX);
        }

//        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
//            if (cannon.getActuatorPos() < .5) {
//                cannon.setActuatorPos(cannon.getActuatorPos() + 0.05);
//            }
//        }
            //cannon.setActuatorPos(0.25);


//        if (Math.abs(gamepad2.right_stick_y) > 0.1){
//            if (cannon.getActuatorPos() < .25) {
//                cannon.setActuatorPos(cannon.getActuatorPos() - 0.05);
//            }
//        }

        if (gamepad2.y && !oldYPressed) {
            if (actuatorIsDown) {
                cannon.setActuatorPos(0.8); // "Up" position
                actuatorIsDown = false;
            } else {
                cannon.setActuatorPos(0.0); // "Down" position
                actuatorIsDown = true;
            }
        }


        velMPS = (cannon.getLauncherVelocity() / 4) / (28 / (0.096 * Math.PI));
        // old button presses at the bottom of loop
        oldAPressed = gamepad2.a;
        oldBPressed = gamepad2.b;
        oldXPressed = gamepad2.x;
        oldYPressed = gamepad2.y;
        oldrStickPressed = rStickPressed;
        oldDPadDownPressed = gamepad2.dpad_down;

        //chassis.setLightColor();

        //cannon.launchSmarter(gamepad2.right_bumper);

        //colSens.getDetectedColor(telemetry);
//
        telemetry.addData("Tlock on", tLock);
        telemetry.addData("Tag found", visionAid.hasTarget());
        telemetry.addData("Tx", visionAid.getTx());

        telemetry.addData("Velocity in Meters per Second", velMPS);


        
        telemetry.addData("Turret Position", cannon.getTurretPos());

        telemetry.addLine();
        telemetry.addData("Gate Closed", gateOn);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addLine();

        telemetry.addData("Launcher Pos", cannon.getTurretPos());
        telemetry.addData("Elevator Actuation",cannon.getActuatorPos());
        telemetry.addData("Launch State", cannon.getLaunchState());
        telemetry.addData("Launcher Velocity", cannon.getLauncherVelocity());
        telemetry.addData("Launch Status", cannon.getLaunchStatus());


        telemetry.addData("launchTrigger", launchTrigger);
        telemetry.addData("Actuator Position", cannon.getActuatorPosition());

        telemetry.addData("X:",follower.getPose().getX());
        telemetry.addData("Y:",follower.getPose().getY());
        telemetry.addData("Heading:",follower.getPose().getHeading());
        telemetry.addData("goalDistance", goalDistance);
        //telemetry.addData("Robot Location", ("Coords: " + follower.getPose().getX() + ", " + follower.getPose().getY() + ", Heading: " + follower.getPose().getHeading()));

        //telemetry.addData();
        telemetry.update();
        follower.update();
    }


}
