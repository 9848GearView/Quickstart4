package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;


import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp (name = "Judging Op/Limelight Testing", group = "Iterative OpMode")
public class Judging extends OpMode {

    private Limelight3A limelight;
    public LLResult llResult;
    public BlueLimelightAutoAim vision = null;
    public MecanumDrive chassis = null;
    IntakeV2 cannon = null;
    private IMU imu;
    private double Kp;

    //GP2 booleans
    private boolean rBumperPressed;
    private boolean oldRBumperPressed;
    private boolean lBumperPressed;
    private double tInc = .01;

    //GP1 booleans
    private boolean tiltOn = true;
    @Override
    public void init() {
        vision = new BlueLimelightAutoAim(hardwareMap);
        cannon = new IntakeV2(hardwareMap);

        cannon.setTurret(.5);
        // we want jonah to be able to move it
        //cannon.setActuatorPos(0);
        chassis.setHalfPark(0.60);
        cannon.setGatePosition(.38);
        cannon.setLightColor();

        // 0 = #20, 1 = #24, 2 = #21, #22, #23
        // 21-23 is the obelisk patterns
        // 20 is blue, 24 is red
       // limelight.pipelineSwitch(0);//should be the pipeline for the april tag search you want
    }

    @Override
    public void start() { //we want limelight to engage when the button is pressed
        limelight.start();

    }

    @Override
    public void loop() {

        //GP2 booleans
        lBumperPressed = gamepad2.left_bumper;
        rBumperPressed = gamepad2.right_bumper;

        //begin auto aim
        vision.update();
        if (vision.hasTarget()) {
            telemetry.addData("Tx", vision.getTx());
        } else {
                telemetry.addLine("Tag not found");
        }
        if (vision.hasTarget()){
            Kp = -0.0004;
            double tx = llResult.getTx();
            double botCorr = (Kp * tx);
            if(Math.abs(tx) > .5) {
                cannon.setTurret(cannon.getTurretPos() + botCorr);
            }
        }
        //end auto aim

        //begin launcher (alt. actuator, gate, manual aim, ind. light)
        // alt. actuator (not used atm)
//        if (gamepad2.left_trigger > 0.1) {
//            //shoot close
//            cannon.setActuatorPos(.8); //.53
//        }
//        if (gamepad2.right_trigger > 0.1) {
//            // shoot far
//            cannon.setActuatorPos(1);
//        }
        // gate + ind. light
        if(rBumperPressed && !oldRBumperPressed){
            cannon.setGatePosition(.38);
            cannon.setLightColor();
            oldRBumperPressed = true;
        }

        if(lBumperPressed && oldRBumperPressed){
            cannon.setGatePosition(0.25);
            cannon.setLightColor();
            oldRBumperPressed = false;
        }
        //manual aim
        if(gamepad2.dpad_left) {
            //if(cannon.getTurretPos() < 1 && cannon.getTurretPos() > 0) {
            cannon.setTurret(cannon.getTurretPos() + tInc);
            //}
        }
        if(gamepad2.dpad_right) {
            // if (cannon.getTurretPos() < 1 && cannon.getTurretPos() > 0) {
            cannon.setTurret(cannon.getTurretPos() - tInc);
            // }
        }
        if(gamepad2.dpad_up) {
            cannon.setTurret(.5);
        }

        // end launcher
        //begin tilt park (ON GAMEPAD 2, NOT THE NORMAL CONTROLS)
//        if(gamepad2.x && !tiltOn){
//            chassis.setHalfPark(0.1);
//            tiltOn = true;
//        }
//        if (gamepad2.dpad_down){
//            if (chassis.getHalfPark() < .6){
//                chassis.setHalfPark(chassis.getHalfPark() + .05);
//            }
//            if (chassis.getHalfPark() ==.6){
//                tiltOn = false;
//            }
//        }




    }


}


