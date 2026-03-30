package org.firstinspires.ftc.teamcode.atlas;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsV2;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;

import java.util.TimerTask;

@Autonomous(name = "Atlas Red Small", group = "Autonomous")
public class AtlasRS extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    java.util.Timer timer = new java.util.Timer();
    IntakeV2 cannon = null;
    BlueLimelightAutoAim vision = null;
    private Timer pathTimer;
    public int timesHumanIntake = 0;
    public int setTimesHumanIntake = 1;

    @Override
    public void init() {
        follower = ConstantsV2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths

        cannon = new IntakeV2(hardwareMap);

        cannon.setTurret(.149);
        vision = new BlueLimelightAutoAim(hardwareMap);


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
            double tx = vision.getTx() - 0.2;
            double botCorr = (Kp * tx)/* - feedForward*/;
            if(Math.abs(tx) > .5) {
                cannon.setTurret(cannon.getTurretPos() + botCorr);
            }

        } /* else {
            cannon.setTurret(.149);
        }*/

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
//        public PathChain intake1Mid;
//        public PathChain intake1End;
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
                                    new Pose(85.000, 20.000),
                                    new Pose(84.600, 9.200),
                                    new Pose(134.900, 8.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

//            intake1Mid = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(134.900, 8.400),
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
                                    new Pose(132.600, 14.900),
                                    new Pose(85.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(0))
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
                                    new Pose(85.000, 20.000),
                                    new Pose(84.600, 9.200),
                                    new Pose(134.900, 8.400)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

//            humanplayerMid = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(134.900, 8.400),
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

            shoothp = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(132, 14.900),
                                    new Pose(85.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(0))
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
                                    new Pose(132.600, 14.900),
                                    new Pose(100.000, 22.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }



    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.schedule(new LaunchAuto(), 0);
                timer.schedule(new IntakeAuto(1), 0);
                timer.schedule(new TransferAuto(.55), 0);
                timer.schedule(new ActuatorAuto(1), 0);
                follower.followPath(paths.shoot1,  true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(.25), 100);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 3000);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                    follower.followPath(paths.intake1, true);
                    timer.schedule(new TransferAuto(.55), 200);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 22:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
//                    follower.followPath(paths.intake1Mid, true);
//                    timer.schedule(new TransferAuto(.55), 200);
//                    pathTimer.resetTimer();
//                    setPathState(23);
//                }
//                break;
//            case 23:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
//                    follower.followPath(paths.intake1End, true);
//                    timer.schedule(new TransferAuto(.55), 200);
//                    pathTimer.resetTimer();
//                    setPathState(3);
//                }
//                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 3 || !follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(paths.shoot2, true);
                    timer.schedule(new TransferAuto(0), 200);

                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 2950);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                    follower.followPath(paths.intake2,.85, true);
                    timer.schedule(new TransferAuto(.55), 200);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .2) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot3, true);
                    timer.schedule(new TransferAuto(0), 200);

                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 2950);
                    pathTimer.resetTimer();
                    setPathState(82); // 8 to continue to pickup 3, 82 to go second human player, 11 to go to park early
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                    follower.followPath(paths.intake3,.85, true);
                    timer.schedule(new TransferAuto(.55), 200);
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoot4, true);
                    timer.schedule(new TransferAuto(0), 200);

                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 2950);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 82:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                    follower.followPath(paths.humanplayer, true);
                    timer.schedule(new TransferAuto(.55), 200);
                    pathTimer.resetTimer();
                    timesHumanIntake++;
                    setPathState(92); // 83 to do loop tech, 92 to just launch immediately
                }
                break;
//            case 83:
//                if (pathTimer.getElapsedTimeSeconds() > 2.5 || !follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {
//                    follower.followPath(paths.humanplayerMid, true);
//                    timer.schedule(new TransferAuto(.55), 200);
//                    pathTimer.resetTimer();
//                    setPathState(822);
//                }
//                break;
//            case 822:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .4) {
//                    follower.followPath(paths.humanplayerEnd, true);
//                    timer.schedule(new TransferAuto(.55), 200);
//                    pathTimer.resetTimer();
//                    setPathState(112);// 92 to continue to launch, 112 to park early after picking up human player
//                }
//                break;
            case 92:
                if (pathTimer.getElapsedTimeSeconds() > 3 || !follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    pathTimer.resetTimer();
                    follower.followPath(paths.shoothp, true);
                    timer.schedule(new TransferAuto(0), 200);

                    setPathState(102);
                }
                break;
            case 102:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    timer.schedule(new GateAuto(0.25), 100);
                    timer.schedule(new IntakeAuto(1), 200);
                    timer.schedule(new TransferAuto(1), 200);
                    timer.schedule(new GateAuto(0.38), 2950);
                    pathTimer.resetTimer();
                    if(timesHumanIntake < setTimesHumanIntake){
                        setPathState(82);
                    } else {
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                    follower.followPath(paths.park,true);
                    timer.schedule(new IntakeAuto(0), 200);
                    timer.schedule(new TransferAuto(0), 200);
                    timer.schedule(new StopLaunchAuto(), 200);
                    setPathState(12);
                }
                break;
            case 112:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > .1) {
                    follower.followPath(paths.humanplayerPark,true);
                    timer.schedule(new IntakeAuto(0), 200);
                    timer.schedule(new TransferAuto(0), 200);
                    timer.schedule(new StopLaunchAuto(), 200);
                    setPathState(12);
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