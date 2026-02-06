package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mech.ColorSensor;
import org.firstinspires.ftc.teamcode.mech.IntakeV175;
import org.firstinspires.ftc.teamcode.mech.MecanumDrive;
import org.firstinspires.ftc.teamcode.mech.LLMech;
import org.firstinspires.ftc.teamcode.mech.LauncherV175;


//this is a new version for the new intake system;
//didnt want to deal with the mess of forgetting how i did something

@TeleOp(name="DecodeV175TeleOp", group="Iterative OpMode")
public class DecodeV175TeleOp extends OpMode {
    MecanumDrive chassis = null;
    IntakeV175 cannon = null;
    ColorSensor colSens = null;
    LLMech camera = null;


    ColorSensor.detectedColor detectedColor;


    private boolean dPadUpPressed;


    private boolean rStickPressed;
    private boolean oldrStickPressed;

    //booleans for a button
    private boolean aPressed;
    private boolean oldAPressed;
    private boolean intakeOn = true;

    private boolean driveAPressed;
    private boolean oldDriveAPressed;
    private boolean tLockOn = false;

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

    @Override
    public void init(){
        chassis = new MecanumDrive(hardwareMap);
        cannon = new IntakeV175(hardwareMap);
        colSens = new ColorSensor(hardwareMap);
        camera = new LLMech(hardwareMap);
        camera.getLlResult();
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


        leftX = -gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;

        driveAPressed = gamepad1.a;
        oldDriveAPressed = driveAPressed;

        //intake wheel begin
        if (gamepad2.a && !oldAPressed){
            intakeOn = !intakeOn;
            if(intakeOn){
                cannon.intake(0, 0, 0);
            }else {
                cannon.intake(1, 1, 1);
            }
        }



        //intake wheel begin
        if (gamepad2.b && !oldBPressed){
            gateOn = !gateOn;
            if(gateOn){
                cannon.intake(0, 0, 0);
            }else {
                cannon.intake(1, 1, -1);
            }
        }

        if (gamepad2.x) {
            cannon.launch(.59);//.13
        }
        if (gamepad2.y) {
            cannon.stopLaunch();
        }
        //spits out balls
        if(gamepad2.dpad_down){
            pushDown = !pushDown;
            if(pushDown) {
                cannon.intake(0, 0, 0);
            } else {
                cannon.intake(1, -0.5, -0.5);
            }
        }


        //intake wheel end

        // (FI) color sensor begin
        detectedColor = colSens.getDetectedColor(telemetry);
        // color sensor end

        //Slow turn code :D
        if (gamepad1.right_bumper){
            chassis.slowTurn(0.1);
        }
        else if(gamepad1.left_bumper){
            chassis.slowTurn(-0.1);
        } else {
            chassis.drive(-leftY, leftX, rightX);
        }

        if (gamepad1.a){
            tLockOn = true;
            if (camera.getLlResult() != null && camera.getLlResult().isValid()){ //checks to see if camera is seeing sonething that it is supposed to see
                float botCorr = camera.botCorrection();
                chassis.drive(-leftY, leftX, botCorr*.5);
            }
        } else {
            tLockOn = false;
            chassis.drive(-leftY, leftX, rightX);
        }
        oldAPressed = gamepad2.a;
        oldBPressed = gamepad2.b;
        oldDPadDownPressed = gamepad2.dpad_down;

        //chassis.setLightColor();

        //cannon.launchSmarter(gamepad2.right_bumper);

        colSens.getDetectedColor(telemetry);
        camera.updateLLTelemetry(telemetry);
        telemetry.addData("aPressed: ", aPressed);
        telemetry.addData("Was a pressed before?: ", oldAPressed);
        telemetry.addData("Methinks the color is", detectedColor);
        telemetry.addData("stick pressed", rStickPressed);
        telemetry.addData("did it stick", oldrStickPressed);
        telemetry.addData("Launcher Velocity", cannon.getLauncherVelocity());

        //telemetry.addData();
        telemetry.update();
    }


}
