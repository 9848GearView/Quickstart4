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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.IntakeV3;
import org.firstinspires.ftc.teamcode.mech.FlyWheelMech4;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;
import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.mech.RobotStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV3;

import java.util.ArrayList;
import java.util.function.Supplier;
//guarantee this wont work whatsoever

@Config
@TeleOp(name="Atlas Blue TeleOp", group="Iterative OpMode")
public class AtlasTeleOpBlue extends OpMode {
    MecanumDrive chassis = null;
    IntakeV3 cannon = null;
    FlyWheelMech4 wheel = null;
    private Paths paths;
    private Supplier<PathChain> shootPos;
    private Supplier<PathChain> parkPos;
    public Follower follower;
    BlueLimelightAutoAim vision = null;

    //Mrs. B moved instantiation of hubs ArrayList to init method
    ArrayList<LynxModule> hubs;

    //pidfs for limelight
    private PIDFController turretController;
    public static double kP = 0.001175, kI = 0.000015, kD = 0.00041, ff = 0.0, kS = 0.000015;

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
        vision = new BlueLimelightAutoAim(hardwareMap);
        cannon.setLightColor();
        cannon.setActuatorPos(.53);

        turretController = new PIDFController(kP, kI, kD, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        //set bulk read mode to manual for hubs
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
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
        double drive = leftY;
        double strafe = leftX;
        if (Math.abs(drive) <= 0.01 && Math.abs(strafe) <= 0.01) {
            //drive = follower.getVelocity().getXComponent() * -forwardBrakingGain;
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
        }

        if(cannon.getGatePosition() == 0){
            gateOn = false;
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
                double turretIncrement = turretController.calculate(vision.getTx() , -0.4);
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

        //altitude actuator & shooting
        if (gamepad2.left_trigger > 0.1) {
            //shoot close
            wheel.FlywheelMotorOn(1040);
            cannon.setActuatorPos(.8);
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
        //tiltpark
        if(gamepad1.x){

            //was 3750
            cannon.setTurret(.3);
            chassis.setHalfPark(1200, 1.0);
        }
        if(gamepad1.y){
            chassis.setHalfPark(0, 1.0);
        }

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
//        telemetry.addData("left joystick x:",leftX);
//        telemetry.addData("left joystick y:",leftY);
//        telemetry.addData("right joystick x", rightX);
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
