package org.firstinspires.ftc.teamcode.atlas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.IntakeV3;
import org.firstinspires.ftc.teamcode.mech.FlyWheelMech4;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;
import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.mech.RTPAxon;
import org.firstinspires.ftc.teamcode.mech.RobotStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV3;

import static org.firstinspires.ftc.teamcode.atlas.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.atlas.Prism.Color;
import org.firstinspires.ftc.teamcode.atlas.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.atlas.Prism.PrismAnimations;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


import java.util.function.Supplier;

//guarantee this wont work whatsoever

@Config
@TeleOp(name="Atlas Blue TeleOp", group="Iterative OpMode")
public class AtlasTeleOpBlue extends OpMode {
    //private Limelight3A camera;
    MecanumDrive chassis = null;
    IntakeV3 cannon = null;
    FlyWheelMech4 wheel = null;
    //RTPAxon axon = null;
    private Paths paths;
    private Supplier<PathChain> shootPos;
    private Supplier<PathChain> parkPos;
    public Follower follower;
    BlueLimelightAutoAim vision = null;

    //Mrs. B moved instantiation of hubs ArrayList to init method
    ArrayList<LynxModule> hubs;
    GoBildaPrismDriver prism;
    PrismAnimations.Solid intakeBall = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.Solid noIntakeBall = new PrismAnimations.Solid(Color.TRANSPARENT);
    PrismAnimations.Solid gateStatusOpen = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid gateStatusClose = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid gateBall = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.Solid noGateBall = new PrismAnimations.Solid(Color.TRANSPARENT);

    //ColorSensor colSens = null;

    //pidfs for limelight
    private PIDFController turretController;
    public static double kP = 0.001075, kI = 0.000015, kD = 0.0003, ff = 0.0, kS = 0.00002;

    ColorSensor.detectedColor detectedColor;


    private boolean rStickPressed;
    private boolean oldrStickPressed;

    //booleans for a button
    private boolean aPressed;
    private boolean oldAPressed;
    private boolean intakeOn = true;

    //booleans for down
    private boolean dPadDownPressed;
    private boolean oldDPadDownPressed;

    //booleans for up
    private boolean dPadUpPressed;
    private boolean oldDPadUpPressed;

    //booleans for left
    private boolean dPadLeftPressed;
    private boolean oldDPadLeftPressed;

    //booleans for right
    private boolean dPadRightPressed;
    private boolean oldDPadRightPressed;


    //booleans for gate
    private boolean rBumperPressed;
    private boolean oldRBumperPressed;
    private boolean lBumperPressed;
    private boolean oldLBumperPressed;
    private boolean gateOn = true;
    private String LState;
    
    
    private boolean pushDown = true;
   

    //booleans for turrect alignment
    private boolean bPressed;
    private boolean oldBPressed;
    private boolean tLock;
    private boolean limReached = false;
    

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

    public double startingX = 36;
    public double startingY = 20;
    public double startingH = 90;
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
    public double launchAngleL = 60;
    public double launchAngleS = 45;

    public static double GRAVITY = -9.81;

    private boolean far;
    private boolean close;

    private double tInc;
    private double ffMult;
    private double ffMultInc;
    private double velMPS;
    int halfParkPos = 0;
    private double forwardBrakingGain = 0.0;

    @Override
    public void init(){
        hubs = new ArrayList<>(hardwareMap.getAll(LynxModule.class));
        chassis = new MecanumDrive(hardwareMap);
        cannon = new IntakeV3(hardwareMap);
        wheel = new FlyWheelMech4(hardwareMap);
        //axon = cannon.getRTPAxon();
        vision = new BlueLimelightAutoAim(hardwareMap);
        //camera = hardwareMap.get(Limelight3A.class,"limabean");

        //camera.pipelineSwitch(0);
        //camera.setPollRateHz(90);
        //chassis.setHalfPark(0.45);
        //cannon.setGatePosition(0);
        cannon.setLightColor();
        cannon.setActuatorPos(.53);

        turretController = new PIDFController(kP, kI, kD, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //axon.setTargetRotation(0);
        //chassis.setHalfPark(1000, 1);

        follower = ConstantsV3.createFollower(hardwareMap);
        startingX = RobotStorage.PoseX;
        startingY = RobotStorage.PoseY;
        startingH = RobotStorage.PoseH;
        follower.setStartingPose(new Pose(startingX,startingY,Math.toRadians(startingH)));
        follower.update();
        shootX = 56;
        shootY = 8;
        shootH = 90;
        paths = new Paths(follower);
        shootPos = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(shootX, shootY))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(shootH), 0.8))
                .build();
        parkPos = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(105.3, 33.3))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();


        tLock = false;
        velMPS = 0;
        

        //Mrs. B Added
        tInc = 0.05;
        ffMult = .006;
        ffMultInc = .0005;

        //prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");

        intakeBall.setBrightness(1);
        noIntakeBall.setBrightness(1);
        gateStatusOpen.setBrightness(1);
        gateStatusClose.setBrightness(1);
        gateBall.setBrightness(1);
        noGateBall.setBrightness(1);

        intakeBall.setStartIndex(0);
        intakeBall.setStopIndex(4);
        noIntakeBall.setStartIndex(0);
        noIntakeBall.setStopIndex(4);
        gateStatusOpen.setStartIndex(4);
        gateStatusOpen.setStopIndex(7);
        gateStatusClose.setStartIndex(4);
        gateStatusClose.setStopIndex(7);
        gateBall.setStartIndex(8);
        gateBall.setStopIndex(12);
        noGateBall.setStartIndex(8);
        noGateBall.setStopIndex(12);
        //colSens = new ColorSensor(hardwareMap);
        //chassis.resetRobotAngle();//should be commented out to run teleOp after Auto & keep angle

        //set bulk read mode to manual for hubs
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

//    @Override
//    public void init_loop(){
//        axon.update();
//        //telemetry.addData("Limit Reached", limReached);
//        telemetry.addLine(axon.log());
//
//    }

    @Override
    public void start(){
        //shootPos = new Path(new BezierLine(new Pose(follower.getPose().getX(),follower.getPose().getY()), new Pose(shootX, shootY)));
//        shootPos = follower.pathBuilder()
//                .setGlobalDeceleration()
//                .addPath(new BezierLine(new Pose(72,72), new Pose( 72,72)))
//                .setConstantHeadingInterpolation(0)
//                .build();


        //camera.start();
        cannon.setTurret(0.7);
        cannon.setGatePosition(.15);
        cannon.setLightColor();
        follower.startTeleOpDrive(true);
    }
    @Override
    public void loop(){
        // bulk reading
        for(LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // pedro
        follower.update();
        // limelight
        vision.update();
        // indicator lights
        cannon.setLightColor();
        //prism.updateAllAnimations();
        double drive = leftY;
        double strafe = leftX;
        if (Math.abs(drive) <= 0.01 && Math.abs(strafe) <= 0.01) {
            drive = follower.getVelocity().getXComponent() * -forwardBrakingGain;
        }

        turretController.setPIDF(kP, kI, kD, 0);

        // heading (pedro)
        headingDegrees = Math.abs((360 + (follower.getPose().getHeading() * 180 / Math.PI))) % 360;

        rStickPressed = gamepad1.right_stick_button;

        dPadDownPressed = gamepad2.dpad_down;

        dPadUpPressed = gamepad2.dpad_up;

        dPadRightPressed = gamepad2.dpad_right;

        dPadLeftPressed = gamepad2.dpad_left;

        aPressed = gamepad2.a;
        bPressed = gamepad2.b;

        xPressed = gamepad2.x;

        lBumperPressed = gamepad2.left_bumper;
        rBumperPressed = gamepad2.right_bumper;


        if (Math.abs(gamepad1.left_stick_y) > 0.05) {
            leftY = -gamepad1.left_stick_y;
        } else {
            leftY = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) > 0.05) {
            leftX = -gamepad1.left_stick_x;
        } else {
            leftX = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) > 0.05) {
            rightX = -gamepad1.right_stick_x;
        } else {
            rightX = 0;
        }

        if(cannon.getGatePosition() == .15){
            gateOn = true;
            //prism.insertAnimation(LayerHeight.LAYER_1,gateStatusClose);
        }

        if(cannon.getGatePosition() == 0){
            gateOn = false;
            //prism.insertAnimation(LayerHeight.LAYER_1,gateStatusOpen);
        }

        if(cannon.getDistanceIntake() < 4.5){
            //prism.insertAnimation(LayerHeight.LAYER_0,intakeBall);
        } else {
            //prism.insertAnimation(LayerHeight.LAYER_0,noIntakeBall);
        }

        if(cannon.getDistanceGate() < 4.5){
            //prism.insertAnimation(LayerHeight.LAYER_2,gateBall);
        } else {
           // prism.insertAnimation(LayerHeight.LAYER_2,noGateBall);
        }

        //intake
        if (gamepad2.a && !oldAPressed){
            intakeOn = !intakeOn;
            if(intakeOn){
                cannon.intake(0);
            }else {
                cannon.intake(1);
            }
        }

        //spits out balls
        if(gamepad2.dpad_down && !oldDPadDownPressed){
            pushDown = !pushDown;
            if(pushDown) {
                cannon.intake(0);
            }else {
                cannon.intake(-1.0);
            }
        }

        if(gamepad1.dpad_up){
            ffMult = ffMult+ ffMultInc;
        }
        if(gamepad1.dpad_down){
            ffMult = ffMult - ffMultInc;
        }

        //intake wheel end

        /*if(gamepad1.y) {
            follower.setX(9.713344316095563);
            follower.setY(9.186161449752879);
            follower.setHeading(180);
        }
        
         */

        //paths = new Paths(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading(),);
        //shootpos = new Paths(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading(),);

        goalDistance = Math.sqrt(Math.pow(follower.getPose().getX() - 4,2) + Math.pow(140 - follower.getPose().getY(),2));
        goalAngle = Math.atan((140 - follower.getPose().getY()) / (follower.getPose().getX() - 4)) + 90;
        launchAngleL = Math.max(Math.atan((Math.pow(velMPS,2) + Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance ),
                Math.atan((Math.pow(velMPS,2) - Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance
                ));
        launchAngleS = Math.min(Math.atan((Math.pow(velMPS,2) + Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance ),
                Math.atan((Math.pow(velMPS,2) - Math.sqrt(Math.pow(velMPS,4) - (GRAVITY*(GRAVITY*Math.pow(goalDistance,2) + 2*Math.pow(velMPS,2)*.7)))) / GRAVITY*goalDistance
                ));


//        //auto-aim

        // Manual controls for turret
        if (gamepad2.dpad_right && !oldDPadRightPressed) {
            cannon.setTurret(cannon.getTurretPos() - tInc);
        }
        if (gamepad2.dpad_left && !oldDPadLeftPressed) {
            cannon.setTurret(cannon.getTurretPos() + tInc);
        }
        if (gamepad2.dpad_up && !oldDPadUpPressed) {
            cannon.setTurret(.5);
        }

        if( bPressed && !oldBPressed) {
            tLock = !tLock; // reminder to find a way to turn this off
        }

        if (tLock) {
            if (vision.hasTarget()){
                double tx = vision.getTx();
                double deadband = vision.getDeadband();
                double turretIncrement = turretController.calculate(vision.getTx() , 0.4);
                if(turretIncrement > 0){
                    turretIncrement += kS;
                } else if (turretIncrement < 0){
                    turretIncrement -= kS;
                }

                telemetry.addData("Turret Increment", turretIncrement);
                double botCorr = turretIncrement;

                if(Math.abs(tx) > deadband) {
                    cannon.setTurret(cannon.getTurretPos() + botCorr);
                }
            }
        }
        //end auto aim


        //altitude actuator
        if (gamepad2.left_trigger > 0.1) {
            //shoot close
            wheel.FlywheelMotorOn(1040);
            cannon.setActuatorPos(.8); //.53
        }
        if (gamepad2.right_trigger > 0.1) {
            // shoot far
            wheel.FlywheelMotorOn(1320);
            cannon.setActuatorPos(1);
        }
        if(gamepad2.x) {
            wheel.FlywheelMotorOff();
            cannon.stopLaunch();
        }

        //test launch in case break
        if(rBumperPressed && !oldRBumperPressed){
            cannon.setGatePosition(.15); //closed
            cannon.setLightColor();
            oldRBumperPressed = true;
        }

        if(lBumperPressed && oldRBumperPressed){
            cannon.setGatePosition(0); //open
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
            chassis.drive(drive, -leftX, -rightX);
        }
        //chassis
        if(gamepad1.x){

            //was 3750
            chassis.setHalfPark(1000, 1.0);
        }
        if(gamepad1.y){
            chassis.setHalfPark(0, 1.0);
        }
//        if(gamepad1.left_stick_y > 0.1){
//            halfParkPos+= 50;
//            chassis.setHalfPark(halfParkPos,1);
//        }
//
//        if(gamepad1.left_stick_y < -0.1){
//            halfParkPos -= 50;
//            chassis.setHalfPark(halfParkPos,1);
//        }

        if(gamepad1.a){
            if(!posLock) {
                shootX = follower.getPose().getX();
                shootY = follower.getPose().getY();
                //shootH = headingDegrees; // sets heading to 0-360 degree range
                shootH = (follower.getPose().getHeading() * 180 / Math.PI) % 360; // sets heading to -180-180 degree range, default for pedro pathing
                posLock = true;
            } else {
                follower.breakFollowing();
                automatedDrive = false;
                posLock = false;
            }
        }

        if(posLock && !automatedDrive && (Math.abs(follower.getPose().getX() - shootX) > 0.5 || Math.abs(follower.getPose().getY() - shootY) > 0.5 || Math.abs(follower.getPose().getHeading() - shootH) > 1)){
            follower.followPath(shootPos.get());
            automatedDrive = true;
        } else {
            automatedDrive = false;
        }

        if(gamepad1.b){
            follower.followPath(parkPos.get());
            automatedDrive = true;
        }

        if(!follower.isBusy() && automatedDrive){
            follower.breakFollowing();
            automatedDrive = false;
        }

//
//        if(gamepad1.a){
//            if(!posLock) {
//                shootX = follower.getPose().getX();
//                shootY = follower.getPose().getY();
//                //shootH = headingDegrees; // sets heading to 0-360 degree range
//                shootH = (follower.getPose().getHeading() * 180 / Math.PI) % 360; // sets heading to -180-180 degree range, default for pedro pathing
//                posLock = true;
//            } else {
//                posLock = false;
//            }
//        }
//
//        if(posLock && (Math.abs(follower.getPose().getX() - shootX) > 0.5 || Math.abs(follower.getPose().getY() - shootY) > 0.5 || Math.abs(follower.getPose().getHeading() - shootH) > 1)){
//            follower.followPath(shootPos.get());
//            automatedDrive = true;
//        }
//
//        if(!posLock && !follower.isBusy()){
//            follower.breakFollowing();
//            follower.update();
//            automatedDrive = false;
//        }
//        if(follower.isBusy())automatedDrive = false;

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




        if (gamepad2.y && !oldYPressed) {
            if (actuatorIsDown) {
                cannon.setActuatorPos(0.0); // "Up" position
                actuatorIsDown = false;
            } else {
                cannon.setActuatorPos(0.8); // "Down" position
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
        oldDPadLeftPressed = gamepad2.dpad_left;
        oldDPadRightPressed = gamepad2.dpad_right;
        oldDPadUpPressed = gamepad2.dpad_up;

        telemetry.addData("Tlock on", tLock);
        vision.feed(telemetry);
        telemetry.addLine();
        telemetry.addData("Velocity (m/s)", velMPS);
        telemetry.addLine();
        telemetry.addData("Gate Closed", gateOn);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addLine();
        cannon.feed(telemetry);
        telemetry.addData("Limit Reached", limReached);
        telemetry.addData("Tilt Park Position:",chassis.getTiltPark());
        telemetry.addLine();
        telemetry.addData("left joystick x:",leftX);
        telemetry.addData("left joystick y:",leftY);
        telemetry.addData("right joystick x", rightX);
        telemetry.addData("half park position",halfParkPos);

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
