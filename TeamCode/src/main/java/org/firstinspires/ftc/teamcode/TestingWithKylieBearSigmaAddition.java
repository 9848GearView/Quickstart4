package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;

import java.util.Timer;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="TestingWithKylieBearSigmaAddition", group="Iterative OpMode")

public class TestingWithKylieBearSigmaAddition extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor flMotor = null;
    private DcMotor frMotor = null;
    private DcMotor blMotor = null;
    private DcMotor brMotor = null;

    private CRServo ICLServo;
    private CRServo ICRServo;
    private Servo Blinky = null;

    private Servo tcs1000 = null;  // Goon Goon Goon Goon Goon Goon Goon Goon Goon Goon Goon Goon Goon Goon Goon Goon for the funny servo blocker

    private DcMotor OWLMotor;
    private DcMotor OWRMotor;
    //private DcMotor LiftTest = null;

    private IMU imu;

    //public Servo oServo = null;

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
    private boolean rStickPressed;



    private boolean isBPressed = false;

    private boolean oldDPadUpPressed = false;
    private boolean oldDPadUpPressed1 = false;
    private boolean oldDPadDownPressed1 = false;
    private boolean oldDPadLeftPressed = false;
    private boolean oldDPadDownPressed = false;
    private boolean oldDPadRightPressed = false;
    private boolean oldLeftBumperPressed = false;
    private boolean oldRightBumperPressed = false;
    private boolean oldxButtonPressed = false;
    private boolean oldaButtonPressed = false;
    private boolean oldbButtonPressed = false;
    private boolean oldyButtonPressed = false;
    private boolean oldrStickPressed = false;

    private boolean intakeManualMode = true;
    private boolean wasBPressed = false;

    private int LAUNCH_ANGLE = 23;


    private final int DELAY_BETWEEN_MOVES = 100;
    private final int DELAY_BETWEEN_MOVES1 = 75;

    Timer timer;


// ex how to write a timer task constructor
    /*class vExPosition extends TimerTask {
        int i;
        double power;

        public vExPosition(int i, double power) {
            this.i = i;
            this.power = power;

        }

        public void run() {
            vExMotor.setTargetPosition(vExPositions[i]);
            vExMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vExMotor.setPower(power);
        }
    }*/

   /* class PAPower extends TimerTask {
        double power;

        public PAPower(double power) {
            this.power = power;

        }

        public void run() {
            PAServo.setPower(power);
        }
    }*/

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        flMotor = hardwareMap.get(DcMotor.class, "FL");
        frMotor = hardwareMap.get(DcMotor.class, "FR");
        blMotor = hardwareMap.get(DcMotor.class, "BL");
        brMotor = hardwareMap.get(DcMotor.class, "BR");

        ICLServo = hardwareMap.get(CRServo.class, "ICLServo");
        ICRServo = hardwareMap.get(CRServo.class, "ICRServo");
        Blinky = hardwareMap.get(Servo.class, "blinky");

        OWLMotor = hardwareMap.get(DcMotor.class, "OWLMotor");
        OWRMotor = hardwareMap.get(DcMotor.class, "OWRMotor");


        //ICLServo.setDirection(CRServo.Direction.FORWARD);
        ICLServo.setDirection(DcMotorSimple.Direction.FORWARD);
        ICRServo.setDirection(DcMotorSimple.Direction.REVERSE);
        //LiftTest = hardwareMap.get(DcMotor.class, "MrSpin");

        imu = hardwareMap.get(IMU.class,"imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();



        //oServo = hardwareMap.get(Servo.class, "oServo");


//        wristJinkServo = hardwareMap.get(Servo.class, "Wrist (Jink) Servo");
        timer = new Timer();

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OWLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OWRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //OWLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //OWRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //LiftTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.FORWARD);

        //Note, I'll set motor directions once the thing is installed



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        if(gamepad2.dpad_left){
            LAUNCH_ANGLE = 23;
        }
        if(gamepad2.dpad_right){
            LAUNCH_ANGLE = -23;
        }

        // initalized position
        if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > LAUNCH_ANGLE+4) {
            Blinky.setPosition(.28);
        }
        else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < LAUNCH_ANGLE-4){
            Blinky.setPosition(.611);
        }
        else {
            Blinky.setPosition(.500);
        }

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
// - This uses basic math to combine motions and is easier to drive straight.
        //this was taken straight from 20204 code so idk the logic behind the math - Mitch
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double FLPower = r * Math.cos(robotAngle) + rightX;
        final double FRPower = r * Math.sin(robotAngle) - rightX;
        final double BLPower = r * Math.sin(robotAngle) + rightX;
        final double BRPower = r * Math.cos(robotAngle) - rightX;




        // Send calculated power to wheels
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            if (-gamepad1.right_stick_y > 0) {
                if (gamepad1.right_bumper) {
                    flMotor.setPower(1);
                    frMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                    blMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                    brMotor.setPower(1);
                } else {
                    flMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                    frMotor.setPower(1);
                    blMotor.setPower(1);
                    brMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                }
            } else {
                if (gamepad1.right_bumper) {
                    flMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                    frMotor.setPower(-1);
                    blMotor.setPower(-1);
                    brMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                } else {
                    flMotor.setPower(-1);
                    frMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                    blMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                    brMotor.setPower(-1);
                }
            }

            //   }
        } else if (rStickPressed && !oldrStickPressed){
            oldrStickPressed = true;
            flMotor.setPower(FLPower/1.75);
            frMotor.setPower(FRPower/1.75);
            blMotor.setPower(BLPower/1.75);
            brMotor.setPower(BRPower/1.75);
        } else if (rStickPressed && oldrStickPressed) {
            oldrStickPressed = false;
            flMotor.setPower(FLPower);
            frMotor.setPower(FRPower);
            blMotor.setPower(BLPower);
            brMotor.setPower(BRPower);
        } /*else {
            flMotor.setPower(FLPower);
            frMotor.setPower(FRPower);
            blMotor.setPower(BLPower);
            brMotor.setPower(BRPower);
        }*/


        //cannon
        //front wheels start spinning
        //intake
        if (gamepad2.a){
            //LiftTest.setPower(1);
            ICLServo.setPower(1);
            ICRServo.setPower(1);
        }
        if (gamepad2.b){
            ICLServo.setPower(0);
            ICRServo.setPower(0);
            //LiftTest.setPower(0);
        }
      // linear servo for RoboCon
        if (gamepad2.x){
            OWLMotor.setPower(.4);
            OWRMotor.setPower(.4);

            //OWLMotor.
            //oServo.setPosition(1);
        }
        if (gamepad2.y){
            OWLMotor.setPower(0.);
            OWRMotor.setPower(0.);
        }
//uses shooting wheels to intake if ball is stuck
        if (gamepad2.dpad_down) {
            OWLMotor.setPower(-.1);
            OWRMotor.setPower(-.1);
        }
//expands wheels at higher speed
        if (gamepad2.dpad_up) {
            OWLMotor.setPower(.5);
            OWRMotor.setPower(.5);
        }

        if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > LAUNCH_ANGLE+4) {
            Blinky.setPosition(.28);
        }
        else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < LAUNCH_ANGLE-4){
            Blinky.setPosition(.611);
        }
        else {
            Blinky.setPosition(.500);
        }
        //end of cannon code

        //ill need encoders for the Cannon angle
       /* if (dPadUpPressed && !oldDPadUpPressed){
            
        }
*/
        //More buttons
        //Cannon Angle
        dPadUpPressed = gamepad2.dpad_up;
        dPadDownPressed = gamepad2.dpad_down;
        oldDPadDownPressed = dPadDownPressed;
        oldDPadUpPressed = dPadUpPressed;
        //end canon angle

      //  double OWLV = OWLMotor.getPower();

        dPadLeftPressed = gamepad2.dpad_left;
        dPadRightPressed = gamepad2.dpad_right;
        leftBumperPressed = gamepad2.left_bumper;
        rightBumperPressed = gamepad2.right_bumper;
        xButtonPressed = gamepad2.x;
        aButtonPressed = gamepad2.a;
        bButtonPressed = gamepad2.b;
        yButtonPressed = gamepad2.y;
        lTriggerPressed = gamepad2.left_trigger;
        rTriggerPressed = gamepad2.right_trigger;
        rStickPressed = gamepad1.right_stick_button;
        oldrStickPressed = rStickPressed;

        //Buttons



        oldDPadLeftPressed = dPadLeftPressed;
        oldDPadRightPressed = dPadRightPressed;
        oldLeftBumperPressed = leftBumperPressed;
        oldRightBumperPressed = rightBumperPressed;
        oldaButtonPressed = aButtonPressed;
        oldbButtonPressed = bButtonPressed;
        oldxButtonPressed = xButtonPressed;
        oldyButtonPressed = yButtonPressed;

        telemetry.addData("Degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
       // telemetry.addData("Speeeed", OWLMotor.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }
}
//https://www.google.com/search?sca_esv=c7c67eb20f0b5775&rlz=1C1GCEA_enUS1131US1131&q=clearly+you+don%27t+own+an+air+fryer&udm=2&fbs=AEQNm0Aa4sjWe7Rqy32pFwRj0UkWd8nbOJfsBGGB5IQQO6L3JyWp6w6_rxLPe8F8fpm5a57iruiBaetC-P1z8A1EgSEtGoKiI-tyuuiDuAjQZN76zaAbPytU70vrRXfg6Tgzjij5R_Re136YiAiZQmK01ZhFDaBKvuWzjRrVqF2bxrJnMYbpGsRQzdzMtgTRsg_T6B4z0T9loWGkBjDF7Xezy_v0ygoVag&sa=X&ved=2ahUKEwjq9qWr3raKAxVQElkFHYH2IEQQtKgLegQIFBAB&biw=1536&bih=695&dpr=1.25&safe=active&ssui=on#vhid=wbIFEtSOIZAwvM&vssid=mosaic