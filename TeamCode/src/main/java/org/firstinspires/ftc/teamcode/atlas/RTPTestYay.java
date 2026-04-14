package org.firstinspires.ftc.teamcode.atlas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.mech.RTPAxon;
import org.firstinspires.ftc.teamcode.mech.GamepadPair;

import org.firstinspires.ftc.teamcode.mech.IntakeV3;
import org.firstinspires.ftc.teamcode.mech.BlueLimelightAutoAim;


// TeleOp test class for manual tuning and testing
@TeleOp(name = "YAYYYYYY", group = "test")
public class RTPTestYay extends OpMode {
    IntakeV3 cannon = null;
    BlueLimelightAutoAim visionAid = null;
    RTPAxon axon = null;

    private boolean rStickPressed;
    private boolean oldrStickPressed;

    //booleans for a button
    private boolean aPressed;
    private boolean a1Pressed;
    private boolean oldAPressed;
    private boolean oldA1Pressed;
    private boolean intakeOn = true;

    //booleans for down
    private boolean dPadDownPressed;
    private boolean oldDPadDownPressed;

    //booleans for up
    private boolean dPadUpPressed;
    private boolean oldDPadUpPressed;

    //booleans for left
    private boolean dPadLeftPressed;
    private boolean oldDPadLeftPressed;

    //booleans for right
    private boolean dPadRightPressed;
    private boolean oldDPadRightPressed;


    //booleans for turret
    private boolean rBumperPressed;
    private boolean oldRBumperPressed;
    private boolean lBumperPressed;
    private boolean oldLBumperPressed;
    private String LState;
    private boolean tLock;
    private boolean pushDown = true;
    private double turretIncrement;

    //booleans for b button
    private boolean b1Pressed;
    private boolean bPressed;
    private boolean oldBPressed;
    private boolean oldB1Pressed;
    private boolean gateOn = true;

    //booleans for x button
    private boolean xPressed;
    private boolean x1Pressed;
    private boolean oldXPressed;
    private boolean oldX1Pressed;
    private boolean launchTrigger = false;

    //booleans for y
    private boolean yPressed;
    private boolean y1Pressed;
    private boolean oldYPressed;
    private boolean oldY1Pressed;
    private boolean oldGP1Y;
    private boolean actuatorIsDown;

    private double leftX;
    private double leftY;
    private double rightX;

    public double shootX;
    public double shootY;
    public double shootH;
    public double headingDegrees = 0;
    public double correctionX = 0;
    public double correctionY = 0;
    public double correctionH = 0;
    public boolean posLock = false;
    public boolean automatedDrive = false;
    public boolean needsCorrection = false;
    public double goalDistance;
    public double goalAngle;
    public double launchAngleL = 60;
    public double launchAngleS = 45;
    public double leftLim = 270;
    public boolean leftLimReached = false;
    public double rightLim = -270;
    public boolean rightLimReached = false;
    public double Kp = 0;
    public double KpInc = .005;
    public double dB = 0;
    public double dBInc = .05;
    public static double GRAVITY = -9.81;

    private boolean far;
    private boolean close;

    private double tInc;

    private double velMPS;

    public double maxRotation = 270;


    @Override
    public void init() {
        cannon = new IntakeV3(hardwareMap);
        visionAid = new BlueLimelightAutoAim(hardwareMap);
        axon = cannon.getRTPAxon();


    }

    @Override
    public void loop() {
        axon.update();
        visionAid.update();

        //controlled turning
        rStickPressed = gamepad1.right_stick_button;

        dPadDownPressed = gamepad2.dpad_down;

        dPadUpPressed = gamepad2.dpad_up;

        dPadRightPressed = gamepad2.dpad_right;

        dPadLeftPressed = gamepad2.dpad_left;

        aPressed = gamepad2.a;
        a1Pressed = gamepad1.a;

        bPressed = gamepad2.b;
        b1Pressed = gamepad1.b;

        xPressed = gamepad2.x;

        lBumperPressed = gamepad2.left_bumper;
        rBumperPressed = gamepad2.right_bumper;

        axon.setMaxPower(.7);

        if(axon.getTargetRotation() > 270){
            axon.setTargetRotation(270);
        }

        if(axon.getTargetRotation() < -270){
            axon.setTargetRotation(-270);
        }


        // Manual controls for target and PID tuning
        if ((gamepad2.dpad_right && !oldDPadRightPressed)) {
            axon.changeTargetRotation(-40);
        }
        if ((gamepad2.dpad_left && !oldDPadLeftPressed)) {
            axon.changeTargetRotation(40);
        }
        if (gamepad2.dpad_up && !oldDPadUpPressed) {
            axon.setTargetRotation(0);
        }



        if (gamepad2.y && !oldYPressed) {
            axon.setKP(axon.getKP() + 0.001);
        }
        if (xPressed && !oldXPressed) {
            axon.setKP(Math.max(0, axon.getKP() - 0.001));
        }

        if (bPressed && !oldBPressed) {
            axon.setKI(axon.getKI() + 0.0001);
        }
        if (aPressed && !oldAPressed) {
            axon.setKI(Math.max(0, axon.getKI() - 0.0001));
        }

        if (rBumperPressed && !oldRBumperPressed) {
            axon.setKD(axon.getKD() + 0.0001);
        }
        if (lBumperPressed && !oldLBumperPressed) {
            axon.setKD(Math.max(0, axon.getKD() - 0.0001));
        }


        /*if (rBumperPressed && !oldRBumperPressed) {
            axon.setKP(0.015);
            axon.setKI(0.0005);
            axon.setKD(0.0025);
            axon.resetPID();
        }

         */

        if (gamepad1.y && !oldY1Pressed) {
            Kp = Kp - KpInc;
        }
        if (gamepad1.x && !oldX1Pressed) {
            Kp = Kp + KpInc;
        }

        if (gamepad1.a && !oldA1Pressed) {
            dB = dB - dBInc;
        }
        if (gamepad1.b && !oldB1Pressed) {
            dB = dB + dBInc;
        }

        if (visionAid.hasTarget()){
            double tx = visionAid.getTx();
            double deadband = dB;
            double botCorr = (Kp * tx);
//
            if(Math.abs(tx) == deadband + 2){ //added so it takes less time to know what the perfect point is
                double perf = axon.getTotalRotation();
                axon.setTargetRotation(perf);
            } else if(Math.abs(tx) > deadband && (axon.getTargetRotation() + botCorr > -270 && axon.getTargetRotation() - botCorr < 270)) {
                axon.changeTargetRotation(botCorr);
            }

        }





        // old button presses at the bottom of loop
        oldAPressed = gamepad2.a;
        oldBPressed = gamepad2.b;
        oldXPressed = gamepad2.x;
        oldYPressed = gamepad2.y;

        oldA1Pressed = gamepad1.a;
        oldB1Pressed = gamepad1.b;
        oldX1Pressed = gamepad1.x;
        oldY1Pressed = gamepad1.y;

        oldrStickPressed = rStickPressed;
        oldDPadDownPressed = gamepad2.dpad_down;
        oldDPadLeftPressed = gamepad2.dpad_left;
        oldDPadRightPressed = gamepad2.dpad_right;
        oldDPadUpPressed = gamepad2.dpad_up;
        oldLBumperPressed = gamepad2.left_bumper;
        oldRBumperPressed = gamepad2.right_bumper;


        telemetry.addData("Starting angle", axon.STARTPOS);
        telemetry.addLine(axon.log());
        telemetry.addData("NTRY", axon.ntry);
        telemetry.addLine();
        telemetry.addData("Tx", visionAid.getTx());
        telemetry.addData("Kp", Kp);
        telemetry.addData("Deadband", dB);
        telemetry.addLine();

    }
}