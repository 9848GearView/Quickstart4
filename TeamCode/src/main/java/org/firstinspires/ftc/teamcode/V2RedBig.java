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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.TimerTask;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;

@Disabled
@Autonomous(name = "V2RedBig", group = "Examples")
@Configurable
public class V2RedBig extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV2 cannon = null;
    private int dbm = 100;
    private Timer pathTimer;
    public boolean shouldShoot = false;
    boolean hasShot = false;



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(124, 124, Math.toRadians(36.5)));

        follower.setMaxPowerScaling(.8);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();

        cannon = new IntakeV2(hardwareMap);

    }

    @Override
    public void start() {
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing

        if (shouldShoot && !hasShot) { cannon.launchAutoClose(true);} // constantly updates launcher state machine
        else { cannon.launchAutoClose(false);} // keeps launcher idle

        // Detect when the launcher finishes its cycle
        if (shouldShoot && !hasShot && cannon.hasFinishedShot()) {
            hasShot = true;       // mark that we already shot
            shouldShoot = false;  // stop shooter from cycling again
        }

        cannon.setTurret(.5);

        pathState = autonomousPathUpdate(); // Update autonomous state machine

        //constantly check if its true


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
        public PathChain park;


        private Pose startPosition = new Pose(124,124, Math.toRadians(36.5));
        private Pose launchPosition = new Pose(94, 85, Math.toRadians(53));
        private Pose intake1End = new Pose(134, 53, Math.toRadians(0));
        private Pose intake1Curve1 = new Pose(103, 54);
        private Pose intake1Curve2 = new Pose(105, 60);
        private Pose intake2End = new Pose(126.5, 83.5, Math.toRadians(0));
        private Pose parkPosition = new Pose(100,78, Math.toRadians(270));


        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder().addPath(new BezierLine(startPosition, launchPosition))
                    .setLinearHeadingInterpolation(startPosition.getHeading(), launchPosition.getHeading())
                    .build();
            intake1 = follower.pathBuilder().addPath(new BezierCurve(launchPosition, intake1Curve1, intake1Curve2, intake1End))
                    .setLinearHeadingInterpolation(intake1End.getHeading(), intake1End.getHeading())
                    .build();
            shoot2 = follower.pathBuilder().addPath(new BezierLine(intake1End, launchPosition))
                    .setLinearHeadingInterpolation(intake1End.getHeading(), launchPosition.getHeading())
                    .build();
            intake2 = follower.pathBuilder().addPath(new BezierLine(launchPosition, intake2End))
                    .setLinearHeadingInterpolation(intake2End.getHeading(), intake2End.getHeading())
                    .build();
            shoot3 = follower.pathBuilder().addPath(new BezierLine(intake2End, launchPosition))
                    .setLinearHeadingInterpolation(intake2End.getHeading(), launchPosition.getHeading())
                    .build();
            park = follower.pathBuilder().addPath(new BezierLine(launchPosition, parkPosition))
                    .setConstantHeadingInterpolation(parkPosition.getHeading())
                    .build();
        }
    }




    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:// start to launch
                follower.followPath(paths.shoot1, .8, true);
                shouldShoot = true; // starts launching
                hasShot = false;
                setPathState(10);

                break;
            case 10: //reset timer
                if(!follower.isBusy()){
                    if(cannon.hasFinishedShot()) {
                        pathTimer.resetTimer();
                        setPathState(1);
                    }
                }
                break;
            case 1: //launch to intake1
                if(!follower.isBusy()) {
                    follower.followPath(paths.intake1,.8, true);
                }
                cannon.intake(1);
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    cannon.intake(0);
                    setPathState(2);
                }
                break;
            case 2: //intake1 to launch
                if(!follower.isBusy()){
                    shouldShoot = true; // starts launching
                    hasShot = false;
                    follower.followPath(paths.shoot2,.8,true);
                    setPathState(11);
                }
                break;
            case 11: //reset timer
                if(!follower.isBusy()){
                    if(cannon.hasFinishedShot()) {
                        pathTimer.resetTimer();
                        setPathState(3);
                    }
                }
                break;
            case 3: //launch to intake2
                if(!follower.isBusy()) {
                    follower.followPath(paths.intake2,.8, true);
                }
                cannon.intake(1);
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    cannon.intake(0);
                    setPathState(4);
                }
                break;
            case 4: //intake2 to launch
                if(!follower.isBusy()){
                    shouldShoot = true; // starts launching
                    hasShot = false;
                    follower.followPath(paths.shoot3,.8,true);
                    setPathState(5);
                }
                break;
            case 5: //launch to park
                if(!follower.isBusy()){
                    if(cannon.hasFinishedShot()) {
                        follower.followPath(paths.park,.8,true);
                        setPathState(-1);
                    }
                }
                break;

        }


        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
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


}//closes class

