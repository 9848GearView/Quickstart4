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

@Autonomous(name = "TestBlueSmall", group = "Autonomous")
@Configurable // Panels
public class TestBlueSmall extends OpMode {
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
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

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
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.000, 20.000),
                                    new Pose(58.000, 13.000),
                                    new Pose(10.000, 12.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 12.000),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.000, 20.000),
                                    new Pose(53.000, 39.000),
                                    new Pose(52.000, 34.526),
                                    new Pose(10.000, 36.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 36.000),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.000, 20.000),
                                    new Pose(53.000, 59.000),
                                    new Pose(67.400, 59.712),
                                    new Pose(10.000, 59.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 59.400),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                    .build();

            humanplayer = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.000, 20.000),
                                    new Pose(10.000, 12.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                    .build();

            shoothp = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.000, 12.000),
                                    new Pose(58.000, 13.000),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.000, 20.000),
                                    new Pose(44.000, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:// start to launch
                follower.followPath(paths.shoot1, .8, true);
                shouldShoot = true; // starts launching
                hasShot = false;
                setPathState(10);

                break;
            case 10: //reset timer
                if (!follower.isBusy()) {
                    if (cannon.hasFinishedShot()) {
                        pathTimer.resetTimer();
                        setPathState(1);
                    }
                }
                break;
            case 1: //launch to intake1
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake1, .8, true);
                }
                cannon.intake(1);
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(2);
                }
                break;
            case 2: //intake1 to launch
                if (!follower.isBusy()) {
                    shouldShoot = true; // starts launching
                    hasShot = false;
                    follower.followPath(paths.shoot2, .8, true);
                    setPathState(11);
                }
                break;
            case 11: //reset timer
                if (!follower.isBusy()) {
                    if (cannon.hasFinishedShot()) {
                        pathTimer.resetTimer();
                        setPathState(3);
                    }
                }
                break;
            case 3: //launch to intake2
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake2, .8, true);
                }
                cannon.intake(1);
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(4);
                }
                break;
            case 4: //intake2 to launch
                if (!follower.isBusy()) {
                    shouldShoot = true; // starts launching
                    hasShot = false;
                    follower.followPath(paths.shoot3, .8, true);
                    setPathState(12);
                }
                break;
            case 12: //reset timer
                if(!follower.isBusy()){
                    if(cannon.hasFinishedShot()) {
                        pathTimer.resetTimer();
                        setPathState(6);
                    }
                }
                break;
            case 6: //launch to human player
                if(!follower.isBusy()) {
                    follower.followPath(paths.humanplayer,.8, true);
                }
                cannon.intake(1);
                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(7);
                }
                break;
            case 7: //human player to launch
                if(!follower.isBusy()){
                    shouldShoot = true; // starts launching
                    hasShot = false;
                    follower.followPath(paths.shoothp,.8,true);
                    setPathState(8);
                }
                break;
            case 8: //launch to park
                if (!follower.isBusy()) {
                    if (cannon.hasFinishedShot()) {
                        follower.followPath(paths.park, .8, true);
                        cannon.intake(0);
                        setPathState(-1);
                    }
                }
                break;
        }
        return pathState;

    }
}