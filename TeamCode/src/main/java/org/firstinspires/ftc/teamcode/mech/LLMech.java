
package org.firstinspires.ftc.teamcode.mech;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LLMech {
    private static final float Kp = -0.1f; // Proportional control constant
    private Limelight3A limelight;
    private LLResult llResult;


    public LLMech(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class,"limabean");
        // 0 = #20, 1 = #24, 2 = #21, #22, #23
        limelight.setPollRateHz(90);
        // 21-23 is the obelisk patterns
        limelight.pipelineSwitch(0);
    }
    



    public void startLL() { //we want limelight to engage when the button is pressed
        limelight.start();

    }
    public void updateResult() {
        llResult = limelight.getLatestResult();
    }


    public double getTx() {
        return llResult.getTx();
    }
    public double getTy(){
        return llResult.getTy();
    }
    public double getTa(){
        return llResult.getTa();
    }


    public void updateLLTelemetry(Telemetry telemetry){
        
        if (llResult != null) {
            telemetry.addData("Tx", getTx());
            telemetry.addData("Ty", getTy());
            telemetry.addData("Ta", getTa());
        } else{
            telemetry.addLine("Tag not Found");
        }
    }


    public void targetLockdata(){
        // for now, I'll want the heading of the robot using the camera, this was a test
        if (llResult != null && llResult.isValid()){
            float Kp = -0.f; //proportional control constant

            double tx = llResult.getTx();
            double heading_err= tx;

            double steer_adj = Kp * tx;

        }
    }
    public float botCorrection(){//this actually sends the data
        float Kp = -0.0003f;//proportional control constant
        double tx = llResult.getTx();
        double heading_err= tx;
        double steer_adj = Kp * tx;
        return (float) steer_adj;
    }
}
