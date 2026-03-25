package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mech.IntakeV2;

@TeleOp(name="DistanceTest", group="Iterative OpMode")
public class DistanceSensorTest extends OpMode {
    IntakeV2 thing = null;

    @Override
    public void init(){
        thing = new IntakeV2(hardwareMap);
    }

    @Override
    public void loop() {
        //telemetry.addData("Distance", thing.getDistance());
    }
}

