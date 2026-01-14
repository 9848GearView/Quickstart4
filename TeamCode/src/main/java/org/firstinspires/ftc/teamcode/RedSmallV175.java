package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.TimerTask;
import org.firstinspires.ftc.teamcode.mech.IntakeV175;

@Autonomous(name = "RedSmallV175", group = "Examples")
@Configurable
public class RedSmallV175 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV175 cannon = null;
    private int dbm = 100;
    private Timer pathTimer;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(90, 8, Math.toRadians(90)));

        follower.setMaxPowerScaling(.6);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();

        cannon = new IntakeV175(hardwareMap);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
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

        public PathChain launch;
        public PathChain park;
        public PathChain park2;


        private Pose startPosition = new Pose(88,8, Math.toRadians(270));//need real angle
        private Pose launchPosition = new Pose(84, 10, Math.toRadians(250));
        private Pose parkPosition = new Pose(105,13, Math.toRadians(270) );//need real angle


        public Paths(Follower follower) {
            launch = follower.pathBuilder().addPath(new BezierLine(startPosition, launchPosition))
                    .setConstantHeadingInterpolation(launchPosition.getHeading())
                    .build();
            park = follower.pathBuilder().addPath(new BezierLine(launchPosition, parkPosition))
                    .setConstantHeadingInterpolation(parkPosition.getHeading())
                    .build();
            park2 = follower.pathBuilder().addPath(new BezierLine(startPosition, parkPosition))
                    .setConstantHeadingInterpolation(parkPosition.getHeading())
                    .build();

        }
    }


    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:// Starting Pos to Launch 
                /*timer.schedule(new RedSmallV175.ShootAuto(.9), 0);
                timer.schedule(new RedSmallV175.GateAuto(1), 3500);
                timer.schedule(new RedSmallV175.GateAuto(0), 6800);
                timer.schedule(new RedSmallV175.ShootAuto(0), 5900);

                 */
                follower.followPath(paths.park2, true);  //change back
                setPathState(-1);
                break;
            case 1: //launch to park
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6.85){
                    pathTimer.resetTimer();
                    follower.followPath(paths.park,true);
                    setPathState(-1);
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
            cannon.intake(power, power, -power);
        }
    }

    public class ShootAuto extends TimerTask {
        double power;

        public ShootAuto(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            cannon.upToSpeed(power);
        }
    }

    public class GateAuto extends TimerTask {
        double power;

        public GateAuto(double p) {
            this.power = p;
        }

        @Override
        public void run() {
            cannon.intake(power, power, power);
        }
    }


}//closes class

