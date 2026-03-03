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

@Autonomous(name = "TestRedSmall", group = "Autonomous")
@Configurable // Panels
public class TestRB extends OpMode {
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
        follower.setStartingPose(new Pose(124, 124, Math.toRadians(36.5)));

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
                                    new Pose(95.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 84.000),
                                    new Pose(126.000, 83.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            gate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 83.500),
                                    new Pose(120.622, 73.831),
                                    new Pose(129.241, 70.465)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(129.241, 70.465),
                                    new Pose(95.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 84.000),
                                    new Pose(99.442, 56.372),
                                    new Pose(88.868, 60.474),
                                    new Pose(136.100, 59.300)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(136.100, 59.300),
                                    new Pose(101.268, 57.627),
                                    new Pose(95.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 84.000),
                                    new Pose(98.500, 33.000),
                                    new Pose(90.200, 34.500),
                                    new Pose(101.700, 35.400),
                                    new Pose(135.800, 35.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(135.800, 35.000),
                                    new Pose(95.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 84.000),
                                    new Pose(100.000, 78.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
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
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(paths.gate,0.8,true);
                    setPathState(3);
                }
                break;
            case 3: //intake1 to launch
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
                        setPathState(4);
                    }
                }
                break;
            case 4: //launch to intake2
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake2, .8, true);
                }
                cannon.intake(1);
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(5);
                }
                break;
            case 5: //intake2 to launch
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
            case 6: //launch to intake3
                if(!follower.isBusy()) {
                    follower.followPath(paths.intake3,.8, true);
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
                    follower.followPath(paths.shoot4,.8,true);
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