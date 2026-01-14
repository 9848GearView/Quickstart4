package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp (name = "Limelight Testing", group = "Iterative OpMode")
public class Limelight extends OpMode {

    private Limelight3A limelight;
    private IMU imu;




    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limabean");
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
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("BotPose", botPose.toString());
                telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            }

            //in theory this is metatag, getting robot position
            if (llResult != null && llResult.isValid()) {
                Pose3D botpose = llResult.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }
            }


    }
}


