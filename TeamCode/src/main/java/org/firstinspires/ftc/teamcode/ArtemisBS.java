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

@Autonomous(name = "Artemis Blue Small", group = "Autonomous")
@Configurable // Panels
public class ArtemisBS extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV2 cannon = null;
    private Timer pathTimer;
    public boolean shouldShoot = false;
    boolean hasShot = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

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

        if (shouldShoot && !hasShot) { cannon.launchAutoFar(true);} // constantly updates launcher state machine
        else { cannon.launchAutoFar(false);} // keeps launcher idle

        // Detect when the launcher finishes its cycle
        if (shouldShoot && !hasShot && cannon.hasFinishedShot()) {
            hasShot = true;       // mark that we already shot
            shouldShoot = false;  // stop shooter from cycling again
        }

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
        public PathChain intake1End;
        public PathChain shoot2;
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain intake3;
        public PathChain shoot4;
        public PathChain humanplayer;
        public PathChain shoothp;
        public PathChain park;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(57, 12)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(117.5))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.000, 12),
                                    new Pose(57, 11),
                                    new Pose(10.000, 8)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intake1End = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10, 8),
                                    new Pose(11.000, 12.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(170))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.000, 12.000),
                                    new Pose(57.000, 12)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(117.5))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.000, 12),
                                    new Pose(53.000, 39.000),
                                    new Pose(52.000, 34.5),
                                    new Pose(9.5, 35.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(9.5, 35.000),
                                    new Pose(57.000, 12)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117.5))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.000, 12),
                                    new Pose(53.000, 59.000),
                                    new Pose(67.400, 60),
                                    new Pose(9.5, 59.00)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(9.5, 59),
                                    new Pose(57.000, 12)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117.5))
                    .build();

            humanplayer = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(57.000, 12),
                                    new Pose(10.000, 12.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(117.5), Math.toRadians(180))
                    .build();

            shoothp = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.000, 12.000),
                                    new Pose(58.000, 13.000),
                                    new Pose(57.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117.5))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(57.000, 12),
                                    new Pose(36, 20)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(117.5), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.schedule(new LaunchAuto(), 0);
                timer.schedule(new ActuatorAuto(1), 0);
                follower.followPath(paths.shoot1,  true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new IntakeAuto(1), 300);
                    timer.schedule(new GateAuto(.25), 100);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.45) {
                    follower.followPath(paths.intake1, true);
                    pathTimer.resetTimer();
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .4) {
                    follower.followPath(paths.intake1End, true);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .7) {
                    follower.followPath(paths.shoot2, true);
                    timer.schedule(new IntakeAuto(0),700);
                    timer.schedule(new ActuatorAuto(1), 0);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 300);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.45) {
                    follower.followPath(paths.intake2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot3, true);
                    timer.schedule(new IntakeAuto(0), 700);
                    timer.schedule(new ActuatorAuto(1), 0);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 300);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(8); // 8 to continue to pickup 3, 11 to go to park early
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(paths.intake3, true);
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .5) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot4, true);
                    timer.schedule(new IntakeAuto(0),700);
                    timer.schedule(new ActuatorAuto(1), 0);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 63);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.45) {
                    follower.followPath(paths.park,true);
                    timer.schedule(new GateAuto(0.38), 200);
                    timer.schedule(new IntakeAuto(0), 200);
                    timer.schedule(new StopLaunchAuto(), 200);
                    setPathState(12);
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
            cannon.launchFar();
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

}