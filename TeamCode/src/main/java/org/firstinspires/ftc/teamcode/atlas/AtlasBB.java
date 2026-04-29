package org.firstinspires.ftc.teamcode.atlas;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.mech.RobotStorage;
import org.firstinspires.ftc.teamcode.mech.IntakeV3;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV2;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;

import java.util.TimerTask;

@Autonomous(name = "Atlas Blue Big", group = "Autonomous")
public class AtlasBB extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV3 cannon = null;
    BlueLimelightAutoAim vision = null;

    private PIDFController turretController;
    public static double kP = 0.001075, kI = 0.000015, kD = 0.0003, ff = 0.0, kS = 0.00002;


    private Timer pathTimer;
    private Timer globalTimer;

    @Override
    public void init() {
        follower = ConstantsV2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15.5, 112, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths

        cannon = new IntakeV3(hardwareMap);
        cannon.setTurret(0.413);
        cannon.setGatePosition(0.15);
        //cannon.setTargetRotation(.25);
        vision = new BlueLimelightAutoAim(hardwareMap);

        pathTimer = new Timer();
        globalTimer = new Timer();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        vision.update();
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
        follower.update(); // Update Pedro Pathing
        RobotStorage.PoseX = follower.getPose().getX();
        RobotStorage.PoseY = follower.getPose().getY();
        RobotStorage.PoseH = follower.getPose().getHeading();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Velocity", cannon.getLauncherVelocity());

        telemetry.update();
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public int getPathState() {
        return pathState;
    }

    public static class Paths {
        public PathChain shoot1;
        public PathChain intake1;
        public PathChain gate;
        public PathChain shoot2;
        public PathChain intakeGate;
        public PathChain shootGate;
        public PathChain intakeFinal;
        public PathChain parkShoot;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.000, 124.000),
                                    new Pose(49.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(49.000, 84.000),
                                    new Pose(51.000, 54.600),
                                    new Pose(50.600, 60.200),
                                    new Pose(7.900, 59.300)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(7.900, 59.300),
                                    new Pose(23.163, 58.617),
                                    new Pose(16.700, 67.100)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.700, 67.100),
                                    new Pose(47.200, 69.500),
                                    new Pose(49.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intakeGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(49.000, 84.000),
                                    new Pose(39.000, 62.700),
                                    new Pose(6.400, 61.300)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            shootGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(6.400, 61.300),
                                    new Pose(38.700, 67.300),
                                    new Pose(49.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            intakeFinal = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(49.000, 84.000),
                                    new Pose(18.000, 81.600)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            parkShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.000, 81.600),
                                    new Pose(60.500, 106.100)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                globalTimer.resetTimer();
                timer.schedule(new LaunchAuto(835), 0);
                timer.schedule(new IntakeAuto(1), 750);
                timer.schedule(new ActuatorAuto(.8),0);
                //timer.schedule(new AutoAim(0.7),0);
                follower.followPath(paths.shoot1,  true);
                pathTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    follower.followPath(paths.intake1, true);
                    cannon.setGatePosition(0.15);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .7) {
                    follower.followPath(paths.gate, true);
                    timer.schedule(new IntakeAuto(0), 500);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot2, true);
                    timer.schedule(new IntakeAuto(1), 200);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    follower.followPath(paths.intakeGate, true);
                    cannon.setGatePosition(0.15);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5.5) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shootGate, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    if(globalTimer.getElapsedTimeSeconds() < 20){
                        setPathState(6);
                    } else {
                        setPathState(9); // 10 to continue to intake 3, 12 to go to park early
                    }
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    follower.followPath(paths.intakeFinal, true);
                    cannon.setGatePosition(0.15);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.parkShoot, true);
                    cannon.setTurret(0.45);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    timer.schedule(new IntakeAuto(0), 0);
                    timer.schedule(new StopLaunchAuto(), 0);
                    setPathState(13);
                }
            break;
        }
        return pathState;
    }
    //classes for timer tasks
    public class IntakeAuto extends TimerTask {
        double power;

        public IntakeAuto(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            cannon.intake(power);
        }
    }

    public class GateAuto extends TimerTask {
        double pos;

        public GateAuto(double p) {
            this.pos = p;
        }

        @Override
        public void run() {
            cannon.setGatePosition(pos);
        }
    }

    public class LaunchAuto extends TimerTask {
        double pow;

        public LaunchAuto(double p) {
            this.pow = p;
        }

        @Override
        public void run() {
            cannon.setVelocity(pow);
        }
    }


    public class StopLaunchAuto extends TimerTask {
        @Override
        public void run() {
            cannon.stopLaunch();
        }
    }


    public class ActuatorAuto extends TimerTask {
        double pos;

        public ActuatorAuto(double p) {
            this.pos = p;
        }

        @Override
        public void run() {
            cannon.setActuatorPos(pos);
        }
    }

    public class AutoAim extends TimerTask {
        boolean tlock;

        public AutoAim(boolean t) {
            this.tlock = t;
        }

        @Override
        public void run() {
            //cannon.setTurretAngle(pos);
        }
    }

}