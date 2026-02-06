package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.Intake;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;
@Disabled
@TeleOp(name="USS Lavender", group="Iterative OpMode")
public class DecodeV1TeleOp extends OpMode {
    MecanumDrive chassis = null;
    Intake cannon = null;
    ColorSensor colSens = null;

    ColorSensor.detectedColor detectedColor;


    private boolean dPadUpPressed;
    private boolean dPadDownPressed;
    private boolean oldDPadUpPressed;
    private boolean oldDPadDownPressed;
    private boolean rStickPressed;
    private boolean oldrStickPressed;
    private double leftX;
    private double leftY;
    private double rightX;


    @Override
    public void init(){
        chassis = new MecanumDrive(hardwareMap);
        cannon = new Intake(hardwareMap);
        colSens = new ColorSensor(hardwareMap);
        //chassis.resetRobotAngle();//should be commented out to run teleOp after Auto & keep angle
    }

    @Override
    public void loop(){
        dPadUpPressed = gamepad2.dpad_up;

        //controlled turning
        rStickPressed = gamepad1.right_stick_button;
        oldrStickPressed = rStickPressed;

        dPadDownPressed = gamepad2.dpad_down;
        oldDPadDownPressed = dPadDownPressed;
        oldDPadUpPressed = dPadUpPressed;

        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;

        //intake wheel begin
        if (gamepad2.a){
           cannon.intake(1);
        }
        if (gamepad2.b){
            cannon.intake(0);
        }
        if (gamepad2.x) {
            cannon.launch(.085);//.13
        }
        if (gamepad2.y) {
            cannon.stopLaunch();
        }
        if (gamepad2.dpad_down) {
            cannon.launch(-.5);
            cannon.intake(-.5);
        }
        //intake wheel end

        if(gamepad2.dpad_right){
            cannon.launchSmarter(true);
        }



        //angle recognition end

        // (FI) color sensor begin
        detectedColor = colSens.getDetectedColor(telemetry);
        // color sensor end


        //controlled drive begin not finished
/*
        if (rStickPressed && !oldrStickPressed) {
            leftX = leftX/1.5;
            leftY = leftY/1.5;
            rightX = rightX/1.5;
            oldrStickPressed = true;
        }

        if (rStickPressed && oldrStickPressed){
            chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x / 1.5);
            oldrStickPressed = false;

        }
        */
        //Slow turn code :D
        if (gamepad1.right_bumper){
            chassis.slowTurn(0.1);
        }
        else if(gamepad1.left_bumper){
            chassis.slowTurn(-0.1);
        }
        else {
            chassis.drive(-leftY, leftX, rightX);
        }

        //chassis.setLightColor();

        //cannon.launchSmarter(gamepad2.right_bumper);

        colSens.getDetectedColor(telemetry);
        telemetry.addData("Methinks the color is", detectedColor);
        telemetry.addData("stick pressed", rStickPressed);
        telemetry.addData("did it stick", oldrStickPressed);
        telemetry.addData("Left Launcher Velocity", cannon.getLeftLauncherVelocity());
        telemetry.addData("Right Launcher Velocity", cannon.getRightLauncherVelocity());
        telemetry.addData("Launch Angle", chassis.getLaunchAngle());
        telemetry.addData("IMU", chassis.getRobotAngle());
        //t
        telemetry.update();
    }


}
