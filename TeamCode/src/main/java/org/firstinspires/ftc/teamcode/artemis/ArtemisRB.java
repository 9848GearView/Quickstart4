package org.firstinspires.ftc.teamcode.artemis;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV2;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;
import org.firstinspires.ftc.teamcode.mech.RedLimelightAutoAim;

import java.util.TimerTask;

@Disabled
@Autonomous(name = "Artemis Red Big", group = "Autonomous")
public class ArtemisRB extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV2 cannon = null;
    RedLimelightAutoAim vision = null;
    private Timer pathTimer;


    @Override
    public void init() {
        follower = ConstantsV2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(124, 124, Math.toRadians(306.2)));

        paths = new Paths(follower); // Build paths

        cannon = new IntakeV2(hardwareMap);

        pathTimer = new Timer();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        vision.update();
        if (vision.hasTarget()){
            float Kp = -0.0004f; //proportional control constant
            //double feedForward = ((rightX + leftX)/2.0) * .005;
            double tx = vision.getTx();
            double botCorr = (Kp * tx)/* - feedForward*/;
            if(Math.abs(tx) > 1) {
                cannon.setTurret(cannon.getTurretPos() + botCorr);
            }

        }
        follower.update(); // Update Pedro Pathing
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
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain intake3;
        public PathChain shoot4;
        public PathChain park;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124.000, 124.000),
                                    new Pose(95.000, 85.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(306.2), Math.toRadians(0))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 85.000),
                                    new Pose(126.000, 83.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            gate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 83.500),
                                    new Pose(121, 74),
                                    new Pose(125, 71)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(129, 70),
                                    new Pose(95.000, 85)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 85.000),
                                    new Pose(99, 56),
                                    new Pose(89, 60),
                                    new Pose(134, 57)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134, 58),
                                    new Pose(101, 58),
                                    new Pose(95.000, 85.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 85.000),
                                    new Pose(98.500, 33.000),
                                    new Pose(90, 35),
                                    new Pose(102, 35),
                                    new Pose(134, 35.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134, 35.500),
                                    new Pose(95.000, 85.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 85.000),
                                    new Pose(105, 78.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();
        }
    }

    //change this to big triangle auto
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.schedule(new LaunchAuto(1100), 0);
                timer.schedule(new IntakeAuto(.8), 0);
                timer.schedule(new ActuatorAuto(.8),0);
                timer.schedule(new AutoAim(0.7),0);
                follower.followPath(paths.shoot1,  true);
                pathTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3200);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(paths.intake1, true);
                    pathTimer.resetTimer();
                    timer.schedule(new TransferAuto(.35), 0);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .7) {
                    follower.followPath(paths.gate, true);
                    timer.schedule(new TransferAuto(0), 200);
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
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3200);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(paths.intake2, true);
                    timer.schedule(new TransferAuto(.35), 0);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot3, true);
                    timer.schedule(new TransferAuto(0), 0);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3200);
                    pathTimer.resetTimer();
                    setPathState(12); // 10 to continue to intake 3, 12 to go to park early
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(paths.intake3, true);
                    timer.schedule(new TransferAuto(.35), 0);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot4, true);
                    timer.schedule(new TransferAuto(0), 0);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3200);
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(paths.park,true);
                    timer.schedule(new IntakeAuto(0), 0);
                    timer.schedule(new TransferAuto(0), 0);
                    timer.schedule(new StopLaunchAuto(), 0);
                    setPathState(13);
                }

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

    public class TransferAuto extends TimerTask {
        double power;

        public TransferAuto(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            cannon.transfer(power);
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
        double pos;

        public AutoAim(double p) {
            this.pos = p;
        }

        @Override
        public void run() {
            cannon.setTurret(pos);
        }
    }
}