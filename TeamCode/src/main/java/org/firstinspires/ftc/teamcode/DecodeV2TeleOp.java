package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.IntakeV2;
import org.firstinspires.ftc.teamcode.mech.LLMech;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;

//guarantee this wont work whatsoever

@TeleOp(name="DecodeV2TeleOp", group="Iterative OpMode")
public class DecodeV2TeleOp extends OpMode {
    MecanumDrive chassis = null;
    IntakeV2 cannon = null;
    LLMech camera = null;
    //ColorSensor colSens = null;



    ColorSensor.detectedColor detectedColor;


    private boolean dPadUpPressed;


    private boolean rStickPressed;
    private boolean oldrStickPressed;

    //booleans for a button
    private boolean aPressed;
    private boolean oldAPressed;
    private boolean intakeOn = true;

    //booleans for down
    private boolean dPadDownPressed;
    private boolean oldDPadDownPressed;
    private boolean pushDown = true;

    //booleans for b button
    private boolean bPressed;
    private boolean oldBPressed;
    private boolean gateOn = true;

    private double leftX;
    private double leftY;
    private double rightX;

    private double tInc;


    @Override
    public void init(){
        chassis = new MecanumDrive(hardwareMap);
        cannon = new IntakeV2(hardwareMap);
        camera = new LLMech(hardwareMap);
        cannon.setTurret(.5);
        cannon.setActuatorPos(0);
        //colSens = new ColorSensor(hardwareMap);
        //chassis.resetRobotAngle();//should be commented out to run teleOp after Auto & keep angle
    }
    @Override
    public void start(){
        camera.startLL();
    }
    @Override
    public void loop(){
        dPadUpPressed = gamepad2.dpad_up;

        //controlled turning
        rStickPressed = gamepad1.right_stick_button;
        oldrStickPressed = rStickPressed;

        dPadDownPressed = gamepad2.dpad_down;

        aPressed = gamepad2.a;
        bPressed = gamepad2.b;


        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        tInc = 0.05;


            //intake
            if (gamepad2.a && !oldAPressed){
                intakeOn = !intakeOn;
                if(intakeOn){
                    cannon.intake(0);
                }else {
                    cannon.intake(1);
                }
            }

        /*
        if (gamepad2.b && !oldBPressed){
            gateOn = !gateOn;
            if(gateOn){
                cannon.intake(0, 0, 0);
            }else {
                cannon.intake(1, 1, 1);
            }
        }

         */

            /*if (dPadDownPressed && !oldDPadDownPressed) {
            cannon.intake(-.5);
            oldDPadDownPressed = true;
        }
        if (dPadDownPressed && oldDPadDownPressed){
            cannon.intake(0);
            oldDPadDownPressed = false;
        }*/

            //spits out balls
            if(gamepad2.dpad_down){
                pushDown = !pushDown;
                if(pushDown) {
                    cannon.intake(0);
                } else {
                    cannon.intake(-1);
                }
            }


            //intake wheel end

       /* if(gamepad2.dpad_right){
            cannon.launchSmarter(true);
        }
        */


            //angle recognition end

            // (FI) color sensor begin
            //detectedColor = colSens.getDetectedColor(telemetry);
            // color sensor end

            if(gamepad2.dpad_left) {
                //if(cannon.getTurretPos() < 1 && cannon.getTurretPos() > 0) {
                    cannon.setTurret(cannon.getTurretPos() - tInc);
                //}
            }
            if(gamepad2.dpad_right) {
               // if (cannon.getTurretPos() < 1 && cannon.getTurretPos() > 0) {
                    cannon.setTurret(cannon.getTurretPos() + tInc);
               // }
            }
            if(gamepad2.dpad_up) {
                cannon.setTurret(.5);
            }
            //auto-aim
//            camera.getLlResult();
//            if (camera.getLlResult() != null && camera.getLlResult().isValid()){
//                float botCorr = camera.botCorrection();
//                cannon.setTurret(botCorr);
//            }



            //test launch in case break
            if(gamepad2.right_bumper){
                cannon.testLaunch();
            }

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
        // bracket was here

        if (cannon.getActuatorPos() < .5) {
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                cannon.setActuatorPos(cannon.getActuatorPos() + 0.1);
            }
        }

        //shoot
        if (gamepad2.x) {
            cannon.launch(true);//.13
        }
        if (gamepad2.y) {
            cannon.stopLaunch();
        }






        oldAPressed = gamepad2.a;
        oldBPressed = gamepad2.b;
        oldDPadDownPressed = gamepad2.dpad_down;

        //chassis.setLightColor();

        //cannon.launchSmarter(gamepad2.right_bumper);

        //colSens.getDetectedColor(telemetry);
        telemetry.addData("aPressed: ", aPressed);
        telemetry.addData("Was a pressed before?: ", oldAPressed);
        //telemetry.addData("Methinks the color is", detectedColor);
        telemetry.addData("stick pressed", rStickPressed);
        telemetry.addData("did it stick", oldrStickPressed);
        //
        // telemetry.addData("Launch Angle", chassis.getLaunchAngle());
        telemetry.addData("Launcher Pos", cannon.getTurretPos());
        telemetry.addData("Elevator Actuation:",cannon.getActuatorPos());
        telemetry.addData("Launch State: ", cannon.getLaunchState());
        telemetry.addData("Launcher Velocity: ", cannon.getLauncherVelocity());
        //telemetry.addData();
        telemetry.update();
    }


}
