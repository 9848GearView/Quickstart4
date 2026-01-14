package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Timer;

public class CoolConstants {
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor IntakeL = null;
    private DcMotor IntakeR = null;

    private boolean dPadUpPressed;
    private boolean dPadDownPressed;
    private boolean dPadLeftPressed;
    private boolean dPadRightPressed;
    private boolean leftBumperPressed;
    private boolean rightBumperPressed;
    private boolean xButtonPressed;
    private boolean aButtonPressed;
    private boolean bButtonPressed;
    private boolean yButtonPressed;
    private double lTriggerPressed;
    private double rTriggerPressed;

    private final int DELAY_BETWEEN_MOVES = 100;

    Timer timer;
}
