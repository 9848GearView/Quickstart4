package org.firstinspires.ftc.teamcode.mech;

import org.firstinspires.ftc.teamcode.mech.IntakeV2;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp (name = "Judging Op/Limelight Testing", group = "Iterative OpMode")
public class Limelight extends OpMode {

    private Limelight3A limelight;
    IntakeV2 cannon = null;
    private IMU imu;




    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limabean");
        cannon = new IntakeV2(hardwareMap);
        cannon.setGatePosition(.5);
        cannon.setTurret(.5);
        cannon.setActuatorPos(0);

        // 0 = #20, 1 = #24, 2 = #21, #22, #23
        // 21-23 is the obelisk patterns
        limelight.pipelineSwitch(0);//should be the pipeline for the april tag search you want
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    @Override
    public void start() { //we want limelight to engage when the button is pressed
        limelight.start();

    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
            } else {
                telemetry.addLine("Tag not found");
            }


            if (llResult!= null && llResult.isValid()){
                    float Kp = -0.0002f; //proportional control constant
                    double tx = llResult.getTx();
                    double botCorr = (Kp * tx);
                    cannon.setTurret(cannon.getTurretPos() + botCorr);

            }





    }


}


