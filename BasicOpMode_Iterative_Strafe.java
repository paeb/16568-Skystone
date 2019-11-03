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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")

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


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //Drive declaration
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");


        //Drive settings
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


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
        final double v1 = drive + strafe + turn;
        final double v2 = drive - strafe - turn;
        final double v3 = drive - strafe + turn;
        final double v4 = drive + strafe - turn;

        //Change speed factor
        if (gamepad1.dpad_down && speedFactor>=0.2 && lastPressed!=gamepad1.dpad_down) {
            speedFactor -= .1;
        } else if (gamepad1.dpad_up && speedFactor<=0.9 && lastPressed!=gamepad1.dpad_up) {
            speedFactor += .1;
        }
        lastPressed = gamepad1.dpad_down || gamepad1.dpad_up;


        // Send calculated power to wheels
        frontLeftDrive.setPower(v1*speedFactor);
        frontRightDrive.setPower(v2*speedFactor);
        backLeftDrive.setPower(v3*speedFactor);
        backRightDrive.setPower(v4*speedFactor);



        // Give information
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Speed Factor", "Speed Factor" + Math.round(speedFactor*10));
        telemetry.addData("Power", "Left Front Power: "+v1);
        telemetry.addData("Power", "Left Right Power: "+v2);
        telemetry.addData("Power", "Back Left Power: "+v3);
        telemetry.addData("Power", "Left Right Power: "+v4);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
