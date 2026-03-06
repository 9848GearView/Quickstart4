package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;

import java.util.TimerTask;

@Autonomous(name = "Artemis Red Big", group = "Autonomous")
@Configurable // Panels
public class ArtemisRB extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV2 cannon = null;
    private Timer pathTimer;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(124, 124, Math.toRadians(306.2)));

        paths = new Paths(follower); // Build paths

        cannon = new IntakeV2(hardwareMap);

        pathTimer = new Timer();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine



        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
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
                                    new Pose(125, 69)
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
                timer.schedule(new LaunchAuto(), 0);
                timer.schedule(new IntakeAuto(1), 0);
                timer.schedule(new ActuatorAuto(1),0);
                timer.schedule(new AutoAim(0.7),0);
                follower.followPath(paths.shoot1,  true);
                pathTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.3) {
                    follower.followPath(paths.intake1, true);
                    timer.schedule(new TransferAuto(.5), 0);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .7) {
                    follower.followPath(paths.gate, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.3) {
                    follower.followPath(paths.intake2, true);
                    timer.schedule(new TransferAuto(.5), 0);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(12); // 10 to continue to intake 3, 12 to go to park early
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.3) {
                    follower.followPath(paths.intake3, true);
                    timer.schedule(new TransferAuto(.5), 0);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot4, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.3) {
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
        @Override
        public void run() {
            cannon.launchAutoClose();
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