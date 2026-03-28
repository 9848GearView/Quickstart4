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

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV175;
import java.util.TimerTask;
import org.firstinspires.ftc.teamcode.mech.IntakeV175;

@Disabled
@Autonomous(name = "dont use", group = "Examples")
@Configurable
public class RedBigV175 extends OpMode {

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

        follower = ConstantsV175.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(124, 124, Math.toRadians(222)));

        follower.setMaxPowerScaling(.8);

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

        public PathChain launch1;
        //public PathChain intake1S;
        //public PathChain intake1E;
        public PathChain intake1;
        public PathChain launch2;
        //public PathChain intake2S;
        //public PathChain intake2E;
        public PathChain intake2;
        public PathChain launch3;
        public PathChain intake3S;
        public PathChain intake3E;
        public PathChain launch4;
        public PathChain bonuspt;

        private Pose startPosition = new Pose(124,124, Math.toRadians(222) );//need real angle
        private Pose launchPosition = new Pose(91, 90, Math.toRadians(224.5));//need real angle
        private Pose intake1Curve1 = new Pose(94, 81);//need real x & y
        private Pose intake1Curve2 = new Pose(85, 84);//need real x & y
        //private Pose intake1Start = new Pose(91, 67, Math.tRadians(0));
        private Pose intake1End = new Pose(124.5, 83.5, Math.toRadians(0));//need real x & y
        private Pose intake2Curve1 = new Pose(94, 55);//need real x & y
        private Pose intake2Curve2 = new Pose(80, 60);//need real x & y
        //private Pose intake2Start = new Pose(91, 41, Math.toRadians(0));//need real x & y
        private Pose intake2End = new Pose(135, 57, Math.toRadians(0));//need real x & y
        private Pose intake2Curve = new Pose(94, 60);//need real x & y

        //dont use for 1.75
        private Pose intake3Start = new Pose(91, 21, Math.toRadians(0));//need real x & y
        private Pose intake3End = new Pose(124, 21, Math.toRadians(0));//need real x & y

        //DO USE for 1.75
        private Pose park = new Pose(110, 85, Math.toRadians(270));


        public Paths(Follower follower) {
            launch1 = follower.pathBuilder().addPath(new BezierLine(startPosition, launchPosition))
                    .setLinearHeadingInterpolation(startPosition.getHeading(), launchPosition.getHeading())
                    //.setTimeoutConstraint(1000)
                    .build();

            /*intake1S = follower.pathBuilder().addPath(new BezierLine(launchPosition, intake1Start))
                    .setConstantHeadingInterpolation(intake1Start.getHeading())
                    .build();

            intake1E = follower.pathBuilder().addPath(new BezierLine(intake1Start, intake1End))
                    .setConstantHeadingInterpolation(intake1End.getHeading())
                    .build();

             */

            intake1 = follower.pathBuilder().addPath(new BezierCurve(launchPosition, intake1Curve1, intake1Curve2, intake1End))
                    .setConstantHeadingInterpolation(intake1End.getHeading())
                    .build();

            launch2 = follower.pathBuilder().addPath(new BezierLine(intake1End, launchPosition))
                    .setLinearHeadingInterpolation(intake1End.getHeading(), launchPosition.getHeading())
                    .build();

            /*intake2S = follower.pathBuilder().addPath(new BezierLine(launchPosition, intake2Start))
                    .setConstantHeadingInterpolation(intake2Start.getHeading())
                    .build();

            intake2E = follower.pathBuilder().addPath(new BezierLine(intake2Start, intake2End))
                    .setConstantHeadingInterpolation(intake2End.getHeading())
                    .build();

             */

            intake2 = follower.pathBuilder().addPath(new BezierCurve(launchPosition, intake2Curve1, intake2Curve2, intake2End))
                    .setConstantHeadingInterpolation(intake1End.getHeading())
                    .build();

            launch3 = follower.pathBuilder().addPath(new BezierCurve(intake2End, intake2Curve, launchPosition))
                    .setLinearHeadingInterpolation(intake2End.getHeading(), launchPosition.getHeading())
                    .build();

            intake3S = follower.pathBuilder().addPath(new BezierLine(launchPosition, intake3Start))
                    .setConstantHeadingInterpolation(intake3Start.getHeading())
                    .build();

            intake3E = follower.pathBuilder().addPath(new BezierLine(intake3Start, intake3End))
                    .setConstantHeadingInterpolation(intake3End.getHeading())
                    .build();

            launch4 = follower.pathBuilder().addPath(new BezierLine(intake3End, launchPosition))
                    .setLinearHeadingInterpolation(intake3End.getHeading(), launchPosition.getHeading())
                    .build();

            bonuspt = follower.pathBuilder().addPath(new BezierLine(launchPosition, park))
                    .setLinearHeadingInterpolation(launchPosition.getHeading(), park.getHeading())
                    .build();
        }
    }


    public int autonomousPathUpdate() {
            switch(pathState) {
                case 0:// Starting Pos to Launch 1
                    timer.schedule(new ShootAuto(.59), 0);
                    timer.schedule(new GateAuto(1), 3500);
                    timer.schedule(new GateAuto(0), 7100);
                    timer.schedule(new ShootAuto(0), 6200);
                    follower.followPath(paths.launch1, true);
                    setPathState(1);

                    break;
                /*case 1: //launch 1 to intake 1 start
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6.85){
                        pathTimer.resetTimer();
                        follower.followPath(paths.intake1S,true);
                        setPathState(2);
                    }
                    break;

                 */
                case 1: //launch 1 to intake 1
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6.9) {
                        timer.schedule(new IntakeAuto(1.0), 500);
                        timer.schedule(new IntakeAuto(0), 3600);
                        follower.followPath(paths.intake1, .4, true);
                        setPathState(2);
                        pathTimer.resetTimer();
                    }
                    break;
                case 2: //intake 1 to launch 2
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.15) {
                        pathTimer.resetTimer();
                        timer.schedule(new ShootAuto(.59), 0);
                        timer.schedule(new GateAuto(1), 3500);
                        timer.schedule(new GateAuto(0), 6800);
                        timer.schedule(new ShootAuto(0), 5900);
                        follower.followPath(paths.launch2,true);
                        setPathState(3);
                        pathTimer.resetTimer();
                    }
                    break;
                /*case 4: //launch 2 to intake 2 start
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6.85) {
                        follower.followPath(paths.intake2S,true);
                        setPathState(5);
                        pathTimer.resetTimer();
                    }
                    break;

                 */
                case 3: //launch 2 to intake 2
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6.85) {
                        pathTimer.resetTimer();
                        timer.schedule(new IntakeAuto(1.0), 1600);
                        timer.schedule(new IntakeAuto(0), 4700);
                        follower.followPath(paths.intake2, .45,true);
                        setPathState(4);
                    }
                    break;
                case 4: //intake 2 to launch 3
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.1) {
                        pathTimer.resetTimer();
                        timer.schedule(new ShootAuto(.59), 0);
                        timer.schedule(new GateAuto(1), 3500);
                        timer.schedule(new GateAuto(0), 6800);
                        timer.schedule(new ShootAuto(0), 5900);
                        follower.followPath(paths.launch3,true);
                        setPathState(10);
                    }
                    break;
                /*case 7: //launch 3 to intake start 3
                    if(!follower.isBusy()) {
                        follower.followPath(paths.intake3S,true);
                        setPathState(8);
                    }
                    break;
                case 8: //intake start 3 to intake end 3
                    if(!follower.isBusy()) {
                        timer.schedule(new IntakeAuto(1.0), 0);
                        follower.followPath(paths.intake3E,true);
                        setPathState(9);
                    }
                    break;
                case 9: //intake end 3 to launch 4
                    if(!follower.isBusy()) {
                        timer.schedule(new GateAuto(1), 4000);
                        timer.schedule(new ShootAuto(1), 0);
                        follower.followPath(paths.launch4,true);
                        setPathState(10);
                    }
                    break;

                 */
                case 10: //launch 4 to off launch line for movement pts
                    if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6.9) {
                        pathTimer.resetTimer();
                        follower.followPath(paths.bonuspt,true);
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
            cannon.intake(power, power, -power);
        }
    }


}//closes class

