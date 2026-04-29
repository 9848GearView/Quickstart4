package org.firstinspires.ftc.teamcode.atlas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.mech.RTPAxon;
import org.firstinspires.ftc.teamcode.mech.RobotStorage;
import org.firstinspires.ftc.teamcode.mech.IntakeV3;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV2;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;

import java.util.TimerTask;

@Autonomous(name = "Atlas Red Small", group = "Autonomous")
public class AtlasRS extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV3 cannon = null;
    //RTPAxon axon = null;
    BlueLimelightAutoAim vision = null;
    private Timer pathTimer;
    private Timer matchTimer;
    public int timesHumanIntake = 0;
    public int setTimesHumanIntake = 1;
    //can run and tune limelight w/ this
    private PIDFController turretController;
    //public static double kP = 0.000675, kI = 0.0, kD = 0.0001, ff = 0.0, kS = 0.0;
    public static double kP = 0.001175, kI = 0.000015, kD = 0.00041, ff = 0.0, kS = 0.000015;


    @Override
    public void init() {
        follower = ConstantsV2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths

        cannon = new IntakeV3(hardwareMap);
        //axon = cannon.getRTPAxon();
        turretController = new PIDFController(kP, kI, kD, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //axon.setTargetRotation(-45);
        vision = new BlueLimelightAutoAim(hardwareMap);

        cannon.setGatePosition(0.15);
        cannon.setTurret(0.610);

        pathTimer = new Timer();
        matchTimer = new Timer();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start(){
        matchTimer.resetTimer();
    }

    @Override
    public void loop() {
        vision.update();
        turretController.setPIDF(kP,kI,kD,0);
        if (vision.hasTarget()){
            float Kp = -0.0004f; //proportional control constant
            //double feedForward = ((rightX + leftX)/2.0) * .005;
            double tx = vision.getTx() - 0.2;
            double botCorr = (Kp * tx)/* - feedForward*/;
            double turretIncrement = turretController.calculate(vision.getTx() , 0);
            telemetry.addData("Turret Increment", turretIncrement);

            if(Math.abs(tx) > .5) {
                //cannon.setTurretAngle(cannon.getTurretPos() + turretIncrement);
            }

        } /* else {
            cannon.setTurret(.149);
        }*/

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
//        public PathChain intake1Mid;
//        public PathChain intake1End;
        public PathChain shoot2;
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain intake3;
        public PathChain shoot4;
        public PathChain humanplayer;
        public PathChain humanplayerMid;
        public PathChain humanplayerEnd;
        public PathChain shoothp;
        public PathChain park;
        public PathChain humanplayerPark;
        public PathChain dynamicPark;


        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),
                                    new Pose(85.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85, 20.000),
                                    new Pose(101.8, 10.800),
                                    new Pose(120.3, 9.000),
                                    new Pose(136.3, 9)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

//            intake1Mid = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(135.300, 8.400),
//                                    new Pose(135.800, 22.400),
//                                    new Pose(125.900, 19.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-15))
//                    .build();
//
//            intake1End = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(125.900, 19.900),
//                                    new Pose(132.600, 14.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(-15))
//                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(135.300, 9),
                                    new Pose(85.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 20.000),
                                    new Pose(91.000, 39.000),
                                    new Pose(92.000, 34.500),
                                    new Pose(134.000, 36.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 36.000),
                                    new Pose(85.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 20.000),
                                    new Pose(91.000, 59.000),
                                    new Pose(76.600, 59.712),
                                    new Pose(134.000, 59.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 59.400),
                                    new Pose(85.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            humanplayer = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    /*new Pose(85, 20.000),
                                    new Pose(104.7, 18.700),
                                    new Pose(131.2, 17.3),
                                    new Pose(135.3, 9)*/
                                    new Pose(85.000, 20.000),
                                    new Pose(129.000, 59.300),
                                    new Pose(129.200, 13.600)
                            )
                    )
                    //.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setConstantHeadingInterpolation(Math.toRadians(-70))
                    .build();

//            humanplayerMid = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(135.300, 8.400),
//                                    new Pose(135.800, 22.400),
//                                    new Pose(125.900, 19.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-15))
//                    .build();

//            humanplayerEnd = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(125.900, 19.900),
//                                    new Pose(132, 14.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(-15))
//                    .build();
            humanplayerMid = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.000, 20.000),
                                    new Pose(117.100, 10.900),
                                    new Pose(128.416, 11.588)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            humanplayerEnd = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(128.416, 11.588),
                                    new Pose(136.300, 8.400),
                                    new Pose(135.000, 40.300)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30),0.25)
                    .build();


            shoothp = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    //new Pose(135.3, 9),
                                    new Pose(135.000, 40.300),
                                    new Pose(85.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.000, 20.000),
                                    new Pose(100.000, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            humanplayerPark = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(135.3, 9),
                                    new Pose(100.000, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

//
//            shoot1 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(88.000, 8.000),
//                                    new Pose(85.000, 20.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            intake1 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(85.000, 20.000),
//                                    new Pose(84.600, 9.200),
//                                    new Pose(134.900, 8.400)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
//                    .build();
//
//            shoot2 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(134.900, 8.400),
//                                    new Pose(85.000, 20.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            intake2 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(85.000, 20.000),
//                                    new Pose(91.000, 39.000),
//                                    new Pose(92.000, 34.500),
//                                    new Pose(134.000, 36.000)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(-2))
//                    .build();
//
//            shoot3 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(134.000, 36.000),
//                                    new Pose(85.000, 20.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            humanplayerMid = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(85.000, 20.000),
//                                    new Pose(119.400, 30.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-70))
//                    .build();
//
//            humanplayerEnd = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(119.400, 30.900),
//                                    new Pose(132.500, 23.100),
//                                    new Pose(131.800, 10.300)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(-71))
//                    .build();
//
//            shoothp = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(131.800, 10.300),
//                                    new Pose(84.300, 20.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(-70), Math.toRadians(0))
//                    .build();
//
//            park = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(84.300, 20.000),
//                                    new Pose(100.000, 22.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(6))
//                    .build();
//
//            dynamicPark = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    follower.getPose(),
//                                    new Pose(44, 22.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
//                    .build();
        }
    }



    public int autonomousPathUpdate() {
        // park if less than 3 seconds
        /*if (matchTimer.getElapsedTimeSeconds() >= 27) {
            setPathState(113);
        }
         */
        switch (pathState) {
            case 0:
                timer.schedule(new LaunchAuto(), 0);
                timer.schedule(new IntakeAuto(1), 0);
                timer.schedule(new ActuatorAuto(1), 0);
                follower.followPath(paths.shoot1,  true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 600);
                    timer.schedule(new GateAuto(0.15), 3000);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    follower.followPath(paths.intake1, true);
                    timer.schedule(new GateAuto(0.15),100);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
//            case 22:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
//                    follower.followPath(paths.intake1Mid, true);
//                    pathTimer.resetTimer();
//                    setPathState(23);
//                }
//                break;
//            case 23:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
//                    follower.followPath(paths.intake1End, true);
//                    pathTimer.resetTimer();
//                    setPathState(3);
//                }
//                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 2.8 || (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1)) {
                    follower.followPath(paths.shoot2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.7) {
                    follower.followPath(paths.intake2,.85, true);
                    timer.schedule(new GateAuto(0.15), 100);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .2) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    setPathState(83); // 8 to continue to pickup 3, 82 to go second human player, 11 to go to park early
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    follower.followPath(paths.intake3,.85, true);
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot4, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 82:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    follower.followPath(paths.humanplayer, true);
                    timer.schedule(new GateAuto(0.15), 100);
                    pathTimer.resetTimer();
                    timesHumanIntake++;
                    setPathState(92); // 83 to do loop tech, 92 to just launch immediately
                }
                break;
            case 83:
                if (pathTimer.getElapsedTimeSeconds() > 1.8 && !follower.isBusy() /*&& pathTimer.getElapsedTimeSeconds() > 0 */) {
                    follower.followPath(paths.humanplayerMid, true);
                    timer.schedule(new GateAuto(0.15), 100);
                    pathTimer.resetTimer();
                    setPathState(822);
                }
                break;
            case 822:
                if (pathTimer.getElapsedTimeSeconds() > 2.5 || (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    follower.followPath(paths.humanplayerEnd, true);
                    pathTimer.resetTimer();
                    timesHumanIntake++;
                    setPathState(92);// 92 to continue to launch, 112 to park early after picking up human player
                }
                break;
            case 92:
                if (pathTimer.getElapsedTimeSeconds() > 2.8 || !follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoothp, true);
                    setPathState(102);
                }
                break;
            case 102:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    timer.schedule(new GateAuto(0), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    pathTimer.resetTimer();
                    setPathState(83);
                    /*if(timesHumanIntake < setTimesHumanIntake){
                        setPathState(82);
                    } else {
                        setPathState(11);
                    }
                     */
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.6) {
                    follower.followPath(paths.park,true);
                    timer.schedule(new IntakeAuto(0), 200);
                    timer.schedule(new StopLaunchAuto(), 200);
                    setPathState(12);
                }
                break;
            case 112:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    follower.followPath(paths.humanplayerPark,true);
                    timer.schedule(new IntakeAuto(0), 200);
                    timer.schedule(new StopLaunchAuto(), 200);
                    setPathState(12);
                }
                break;
            case 113:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    follower.followPath(paths.dynamicPark,true);
                    timer.schedule(new IntakeAuto(0), 200);
                    timer.schedule(new StopLaunchAuto(), 200);
                    setPathState(67);
                }
                break;
        }
        if(matchTimer.getElapsedTimeSeconds() > 999){
            setPathState(113);
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
            cannon.launchAutoFar();
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