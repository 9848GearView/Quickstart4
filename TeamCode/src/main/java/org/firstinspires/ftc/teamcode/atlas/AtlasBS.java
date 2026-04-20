package org.firstinspires.ftc.teamcode.atlas;

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
import org.firstinspires.ftc.teamcode.mech.RTPAxon;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;

import java.util.TimerTask;

@Autonomous(name = "Atlas Blue Small", group = "Autonomous")
public class AtlasBS extends OpMode {
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
    public int setTimesHumanIntake = 3;//change this for the amount of times it does human player (not including first intake)

    @Override
    public void init() {
        follower = ConstantsV2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths

        cannon = new IntakeV3(hardwareMap);
        //axon = cannon.getRTPAxon();

        //axon.setTargetRotation(0);
        vision = new BlueLimelightAutoAim(hardwareMap);
        cannon.setGatePosition(0.15);
        cannon.setTurret(0.385);

        pathTimer = new Timer();
        matchTimer = new Timer();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop(){
        //axon.update();
        //telemetry.addData("Limit Reached", limReached);
        //telemetry.addLine(axon.log());

    }

    @Override
    public void loop() {
        vision.update();
        if (vision.hasTarget()){
            float Kp = -0.00041f;
            double tx = vision.getTx();
            double deadband = vision.getDeadband();
            double botCorr = (Kp * tx);
            if(Math.abs(tx) > deadband) {
                //cannon.setTurret(cannon.getTurretPos() + botCorr);
            }

        }

        follower.update(); // Update Pedro Pathing
        //axon.update();
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
        //public PathChain intake1Mid;
        //public PathChain intake1End;
        public PathChain shoot2;
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain intake3;
        public PathChain shoot4;
        public PathChain humanplayer;
        //public PathChain humanplayerMid;
        //public PathChain humanplayerEnd;
        public PathChain shoothp;
        public PathChain park;
        public PathChain humanplayerPark;
        public PathChain dynamicPark;


        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.000, 20.000),
                                    new Pose(39.300, 18.700),
                                    new Pose(12.790, 17.303),
                                    new Pose(8.700, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

//            intake1Mid = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(8.300, 8.400),
//                                    new Pose(8.200, 22.400),
//                                    new Pose(18.100, 19.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(195))
//                    .build();

//            intake1End = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(18.100, 19.900),
//                                    new Pose(11.400, 14.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(195))
//                    .build();


            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.7, 9),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            humanplayer = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    /*new Pose(59.000, 20.000),
                                    new Pose(39.300, 18.700),
                                    new Pose(12.8, 17.3),
                                    new Pose(8.700, 9.000)*/
                                    new Pose(59.000, 20.000),
                                    new Pose(14.100, 42.300),
                                    new Pose(12.200, 10.300)
                            )
                    )
                    //.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setConstantHeadingInterpolation(Math.toRadians(250))
                    .build();

//            humanplayerMid = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(8.300, 8.400),
//                                    new Pose(8.200, 22.400),
//                                    new Pose(18.100, 19.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(195))
//                    .build();

//            humanplayerEnd = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(18.100, 19.900),
//                                    new Pose(12, 14.900)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(195))
//                    .build();

            shoothp = follower.pathBuilder()
                    .addPath(
                            /*new BezierLine(
                                    new Pose(8.7, 9),
                                    new Pose(59.000, 20.000)
                            )*/
                            new BezierLine(
                                    new Pose(12.200, 10.300),
                                    new Pose(59.000, 20.000)
                            )
                    )
                    //.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(180))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.000, 20.000),
                                    new Pose(44.000, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            humanplayerPark = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.7, 9),
                                    new Pose(44, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
            dynamicPark = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    follower.getPose(),
                                    new Pose(44, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

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
                    timer.schedule(new GateAuto(0), 500);
                    timer.schedule(new GateAuto(0.15), 3000);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.7) {
                    follower.followPath(paths.intake1, true);
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
                    timer.schedule(new GateAuto(0.15), 2950);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.7) {
                    follower.followPath(paths.intake2,.85, true);
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
                    timer.schedule(new GateAuto(0.15), 2950);
                    pathTimer.resetTimer();
                    setPathState(82); // 8 to continue to pickup 3, 82 to go second human player, 11 to go to park early
                }
                break;
                case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.7) {
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
                    timer.schedule(new GateAuto(0.15), 2950);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 82:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.7) {
                    follower.followPath(paths.humanplayer, true);
                    pathTimer.resetTimer();
                    timesHumanIntake++;
                    setPathState(92); // 83 to do loop tech, 92 to just launch immediately
                }
                break;
//            case 83:
//                if (pathTimer.getElapsedTimeSeconds() > 2.5 || !follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
//                    follower.followPath(paths.humanplayerMid, true);
//                    pathTimer.resetTimer();
//                    setPathState(822);
//                }
//                break;
//            case 822:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .4) {
//                    follower.followPath(paths.humanplayerEnd, true);
//                    pathTimer.resetTimer();
//                    setPathState(112);// 92 to continue to launch, 112 to park early after picking up human player
//                }
//                break;
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
                    timer.schedule(new GateAuto(0.15), 2950);
                    pathTimer.resetTimer();
                    setPathState(82);
                    /*if(timesHumanIntake < setTimesHumanIntake){
                        setPathState(82);
                    } else {
                        setPathState(11);
                    }
                     */
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.7) {
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