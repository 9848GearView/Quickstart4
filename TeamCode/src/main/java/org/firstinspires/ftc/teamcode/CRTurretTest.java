package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.IntakeV3;

@TeleOp(name="States_BLUE-DecodeV2TeleOp", group="Iterative OpMode")
public class CRTurretTest extends OpMode{
    IntakeV3 cannon = null;
    @Override
    public void init(){
        cannon = new IntakeV3(hardwareMap);
    }
    @Override
    public void loop(){
        if(gamepad1.a){
            
        }
    }
}
