package org.firstinspires.ftc.teamcode.artemis;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;
import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.mech.RTPAxon;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV2;


import java.util.function.Supplier;

//guarantee this wont work whatsoever

@TeleOp(name="States_BLUE-DecodeV2TeleOp", group="Iterative OpMode")
public class States_DecodeV2TeleOp_BLUE extends OpMode {
    private Limelight3A camera;
    MecanumDrive chassis = null;
    IntakeV2 cannon = null;
    private Paths paths;
    private Supplier<PathChain> shootPos;
    public Follower follower;
    BlueLimelightAutoAim visionAid = null;
    RTPAxon axon = null;

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



    //booleans for turret
    private boolean rBumperPressed;
    private boolean oldRBumperPressed;
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
    private boolean oldGP1Y;
    private boolean actuatorIsDown;

    private double leftX;
    private double leftY;
    private double rightX;

    public double shootX;
    public double shootY;
    public double shootH;
    public double headingDegrees = 0;
    public double correctionX = 0;
    public double correctionY = 0;
    public double correctionH = 0;
    public boolean posLock = false;
    public boolean automatedDrive = false;
    public boolean needsCorrection = false;
    public double goalDistance;
    public double goalAngle;
    public double turretAngle = 0;
    public double launchAngleL = 60;
    public double launchAngleS = 45;

    public static double GRAVITY = -9.81;

    private boolean far;
    private boolean close;

    private double tInc;

    private double velMPS;


    @Override
    public void init(){
        chassis = new MecanumDrive(hardwareMap);
        cannon = new IntakeV2(hardwareMap);
        visionAid = new BlueLimelightAutoAim(hardwareMap);
        camera = hardwareMap.get(Limelight3A.class,"limabean");

        camera.pipelineSwitch(0);
        camera.setPollRateHz(90);
        chassis.setHalfPark(0.45);
        cannon.setGatePosition(.38);
        cannon.setLightColor();
        cannon.setTurret(.5);
        cannon.setActuatorPos(.53);



        follower = ConstantsV2.createFollower(hardwareMap);
        //follower.setStartingPose(new Pose(36,20,Math.toRadians(90)));
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(180)));
        follower.update();
        shootX = 56;
        shootY = 8;
        shootH = 90;
        paths = new Paths(follower);
        shootPos = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(shootX, shootY))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(shootH), 0.8))
                .build();

        tLock = false;
        velMPS = 0;
        //colSens = new ColorSensor(hardwareMap);
        //chassis.resetRobotAngle();//should be commented out to run teleOp after Auto & keep angle
    }
    @Override
    public void start(){
        //shootPos = new Path(new BezierLine(new Pose(follower.getPose().getX(),follower.getPose().getY()), new Pose(shootX, shootY)));
//        shootPos = follower.pathBuilder()
//                .setGlobalDeceleration()
//                .addPath(new BezierLine(new Pose(72,72), new Pose( 72,72)))
//                .setConstantHeadingInterpolation(0)
//                .build();

        camera.start();
    }
    @Override
    public void loop(){
        follower.update();
        visionAid.update();
        headingDegrees = Math.abs((360 + (follower.getPose().getHeading() * 180 / Math.PI))) % 360;

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

        //paths = new Paths(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading(),);
        //shootpos = new Paths(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading(),);

        goalDistance = Math.sqrt(Math.pow(follower.getPose().getX() - 4,2) + Math.pow(140 - follower.getPose().getY(),2));
        goalAngle = Math.atan((follower.getPose().getX() - 4 ) / (140 - follower.getPose().getY()));

        if(follower.getPose().getHeading() - goalAngle < 0){
            turretAngle = Math.abs(follower.getPose().getHeading() - goalAngle);
        } else if (follower.getPose().getHeading() - goalAngle > 0 && follower.getPose().getHeading() - goalAngle <= 180){
            turretAngle = follower.getPose().getHeading() - goalAngle;
        } else if(follower.getPose().getHeading() - goalAngle > 180){
            turretAngle = (360 - follower.getPose().getHeading()) + goalAngle;
        } else {
            turretAngle = 0;
        }
        if(turretAngle <= -90) cannon.setTurret(1);
        else if (turretAngle >= 90) cannon.setTurret(0);
        else if (turretAngle > 0 && turretAngle < 90){
            cannon.setTurret(0); // wrong right now
        }

        launchAngleL = Math.max(Math.atan((Math.pow(velMPS,2) + Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance ),
                Math.atan((Math.pow(velMPS,2) - Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance
                ));
        launchAngleS = Math.min(Math.atan((Math.pow(velMPS,2) + Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance ),
                Math.atan((Math.pow(velMPS,2) - Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance
                ));



//        //auto-aim

//        if(cannon.getLaunchStatus() == null){
//            LState = "mid";
//        } else if (cannon.getLaunchStatus().equals("close")){
//            LState = "close";
//        } else if(cannon.getLaunchStatus().equals("far")){
//            LState = "far";
//        }

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

        if( bPressed && !oldBPressed) {
            tLock = !tLock; // reminder to find a way to turn this off
        }
        visionAid.update();
        if (tLock) {
            if (visionAid.hasTarget()){
                float Kp = -0.000405f; //proportional control constant
                double feedForward = ((rightX + leftX)/2.0) * .005;
                double tx = visionAid.getTx() - 3;
                double ta = visionAid.getTa();
                double deadband = visionAid.getDeadband();
                double botCorr = (Kp * tx) - feedForward;
//
                if(Math.abs(tx) > deadband) {
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
        //chassis
        if(gamepad1.x){
            chassis.setHalfPark(0.10);
        }

        if(gamepad1.a){
            if(!posLock) {
                shootX = follower.getPose().getX();
                shootY = follower.getPose().getY();
                //shootH = headingDegrees; // sets heading to 0-360 degree range
                shootH = (follower.getPose().getHeading() * 180 / Math.PI) % 360; // sets heading to -180-180 degree range, default for pedro pathing
                posLock = true;
            } else {
                posLock = false;
            }
        }

        if(posLock && !follower.isBusy() && (Math.abs(follower.getPose().getX() - shootX) > 0.5 || Math.abs(follower.getPose().getY() - shootY) > 0.5 || Math.abs(follower.getPose().getHeading() - shootH) > 1)){
            follower.followPath(shootPos.get());
            automatedDrive = true;
        }

        if(!posLock && follower.isBusy()){
            follower.breakFollowing();
            follower.update();
            automatedDrive = false;
        }
        if(follower.isBusy())automatedDrive = false;

//        if(posLock){
//            if(follower.getPose().getX() - shootX > 0.5){
//                correctionX = -0.4;
//                needsCorrection = true;
//            } else if(follower.getPose().getX() - shootX < -0.5){
//                correctionX = 0.4;
//                needsCorrection = true;
//            } else {
//                correctionX = 0;
//            }
//
//            if(follower.getPose().getY() - shootY > 0.5){
//                correctionY = -0.6;
//                needsCorrection = true;
//            } else if(follower.getPose().getY() - shootY < -0.5){
//                correctionY = 0.6;
//                needsCorrection = true;
//            } else {
//                correctionY = 0;
//            }
//
//            if(Math.abs(headingDegrees - shootH) > 0.5) {
//                if (shootH - headingDegrees < 0 && shootH - headingDegrees > -180) {
//                    correctionH = -0.4;
//                    needsCorrection = true;
//                } else if (shootH - headingDegrees > 180 || shootH - headingDegrees < -180) {
//                    correctionH = 0.4;
//                    needsCorrection = true;
//                }
//            }  else {
//                correctionH = 0;
//            }
//
//            if(needsCorrection && !gamepad1.left_bumper && !gamepad1.right_bumper && leftX == 0 && leftY == 0 && rightX == 0){
//                chassis.drive(correctionY,correctionX,/*correctionH*/ 0);
//            } else  {
//                chassis.drive(-leftY, leftX, rightX);
//            }
//            needsCorrection = false;
//        }

//        if(posLock){
//            if(!follower.isBusy() && ((Math.abs(shootX - follower.getPose().getX()) > 2) || (Math.abs(shootY - follower.getPose().getY()) > 2) || (Math.abs(headingDegrees - shootH) > 0.5))) {
//                shootPos = follower.pathBuilder()
//                        .setGlobalDeceleration()
//                        .addPath(new BezierLine(new Pose(follower.getPose().getX(), follower.getPose().getY()), new Pose(shootX, shootY)))
//                        .setConstantHeadingInterpolation(shootH)
//                        .build();
//                follower.followPath(shootPos, true);
//            } else {
//                follower.breakFollowing();
//            }
//        } else {
//            follower.breakFollowing();
//        }


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
        telemetry.addData("Deadband", visionAid.getDeadband());
        telemetry.addData("Ta", visionAid.getTa());
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

        /*telemetry.addData("Gate Distance", cannon.getDistanceGate());
        telemetry.addData("Intake Distance", cannon.getDistanceIntake());
         */

        telemetry.addData("X:",follower.getPose().getX());
        telemetry.addData("Y:",follower.getPose().getY());
        telemetry.addData("Total Heading:",follower.getPose().getHeading());
        telemetry.addData("Heading:",(follower.getPose().getHeading() * 180 / Math.PI) % 360);
        telemetry.addData("test Heading", headingDegrees);
        telemetry.addData("shoot X:",shootX);
        telemetry.addData("shoot Y:",shootY);
        telemetry.addData("shoot Heading:",shootH);
        telemetry.addData("locked in pos:",posLock);
        telemetry.addData("goalDistance", goalDistance);
        telemetry.addData("left joystick x:",leftX);
        telemetry.addData("left joystick y:",leftY);
        telemetry.addData("right joystick x", rightX);

        //telemetry.addData();
        telemetry.update();
    }

    public static class Paths {
        public PathChain shootPos;

        public Paths(Follower follower) {
            shootPos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();}

        public Paths(Follower follower, double startX, double startY, double startH, double endX, double endY, double endH){
            {
                shootPos = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(startX, startY),
                                        new Pose(endX, endY)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(startH), Math.toRadians(endH))
                        .build();}
        }
    }


}
