/* Written by Lawson Wright and Andrew Ogundimu
 * Created on: 10/20/19
 * Last updates on: 10/20/19
 * Modified version of the BasicOpMode_Iterative external sample.
 * Discription: This is a baisc teleop code used for control of a robot with a gamepad. It has basic movement and turning.
 * It also has speed control with the dpad.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Iterative OpMode3", group="Iterative Opmode")

public class BasicOpMode_Iterative_Strafe extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Drive
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    //SpeedFactor
    private double speedFactor = 1;
    private boolean lastPressed = false;

    //Xrail Y Movement
    private DcMotor stringWheel = null;
    private double wheelSpeed = .5;
    private double XrailWheelSpeed = .15;
    private boolean lastButtonPressed = false;

    //Foundation Grabber
    private boolean foundationGrabberDeployed = false;

    //XRail DeploymentstringWheel
    private DcMotor xrail = null;

    //Intake Grabber
    private Servo intakeServo = null;
    private double intakeIncrement = .05;

    //Intake Arms
    private Servo armLeft = null;
    private Servo armRight = null;
    private double armSpeed = .05;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //Drive init
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fl"); //Port 3
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br"); //Port 0
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Xrail init
        stringWheel = hardwareMap.get(DcMotor.class,"string_wheel"); //Port 0
        stringWheel.setDirection(DcMotor.Direction.REVERSE);

        xrail = hardwareMap.get(DcMotor.class, "xrail");
        xrail.setDirection(DcMotor.Direction.FORWARD);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.0);

        armLeft = hardwareMap.get(Servo.class, "intakeServo");
        armRight = hardwareMap.get(Servo.class, "intakeServo");
        armLeft.setPosition(0.0);
        armRight.setPosition(0.0);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
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
        // Setup a variable for each drive wheel to save power level for telemetry

        //Set power for wheels based on math
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        final double v1 = drive - strafe + turn;
        final double v2 = drive + strafe - turn;
        final double v3 = drive + strafe + turn;
        final double v4 = drive - strafe - turn;


        //Change speed factor
        if (gamepad1.dpad_down && speedFactor >= 0.2 && lastPressed!=gamepad1.dpad_down) {
            speedFactor -= .1;
        } else if (gamepad1.dpad_up && speedFactor <= 0.9 && lastPressed!=gamepad1.dpad_up) {
            speedFactor += .1;
        }
        lastPressed = gamepad1.dpad_down || gamepad1.dpad_up;


        //Locking Mechanics for the xrail string wheel



        //Movement for the xrail string wheel
        if (gamepad1.left_trigger > 0){
            stringWheel.setPower(wheelSpeed);
        } else if (gamepad1.right_trigger > 0){
            stringWheel.setPower(wheelSpeed * -1);
        } else {
            stringWheel.setPower(0);
        }

        if (gamepad1.left_bumper){
            xrail.setPower(-XrailWheelSpeed);
        } else if (gamepad1.right_bumper){
            xrail.setPower(XrailWheelSpeed);
        } else {
            xrail.setPower(0);
        }

        if (gamepad1.a){
            intakeServo.setPosition(0);
        } else if (gamepad1.b){
            intakeServo.setPosition(1);
        }

        if (gamepad1.x){
            armLeft.setPosition(0);
            armRight.setPosition(0);
        } else if (gamepad1.y){
            armLeft.setPosition(1);
            armRight.setPosition(1);
        }


        // Send calculated power to wheels
        frontLeftDrive.setPower(v1*speedFactor);
        frontRightDrive.setPower(v2*speedFactor);
        backLeftDrive.setPower(v3*speedFactor);
        backRightDrive.setPower(v4*speedFactor);

        // Telemetry output
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Speed Factor", "Speed Factor" + Math.round(speedFactor*10));
        telemetry.addData("Power", "Left Front Power: " + v1);
        telemetry.addData("Power", "Left Right Power: " + v2);
        telemetry.addData("Power", "Back Left Power: " + v3);
        telemetry.addData("Power", "Left Right Power: " + v4);
        telemetry.addData("control", "x " + gamepad1.x);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}