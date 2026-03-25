package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class BlueLimelightAutoAim {
    private Limelight3A limelight;
    private LLResult result;
    private int targetTagID = 24; // default is 20 for blue, 24 is red
    //List<LLResultTypes.FiducialResult> fudicials = result.getFiducialResults();

    public BlueLimelightAutoAim(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limabean");
        limelight.pipelineSwitch(0);// AprilTag pipeline
        limelight.start();
    }

    public void setTargetTag(int id){
        targetTagID = id;
    }

    public void update(){
        result = limelight.getLatestResult();
    }

    public boolean hasTarget(){
        return result != null && result.isValid() /*&& result.getTid() == targetTagID*/;
    }

    public double getTx() {
        if(result == null) {
            return 0;
        }

        return result.getTx();
    }
    public double getTa(){
        if (result == null){
            return 0;
        }
        return result.getTa();
    }
    public double getDeadband(){
        if (result == null){
            return .5;
        }
        return (.3 * getTa())+ .44;
    }
    //to track distance
    public double getDistanceMeters() {

        if (result == null || !result.isValid())
            return -1;

//        Pose3D[] pose = {result.getBotpose(),}; // [x,y,z,roll,pitch,yaw]
//
//        double x = pose[0];
//        double z = pose[2];
//
//        return Math.sqrt(x*x + z*z);
        return result.getBotposeAvgDist();
    }


    /*public int getTid() {
        if(result == null){
            return -1;
        }
        return result.getTid();
    }*/
}
