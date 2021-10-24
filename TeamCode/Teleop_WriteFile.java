package teamcode2.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.File;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="Teleop_FoundationLeft_ParkHigh", group="Iterative Opmode")
//@Disabled
public class Teleop_WriteFile
        extends OpMode
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
    private double wheelSpeed = 1;
    private double XrailWheelSpeed = .6;
    private boolean lastButtonPressed = false;
    private int iters = 0;
    private ArrayList<String> commands = new ArrayList<String>();

    //Foundation Grabber
    private boolean foundationGrabberDeployed = false;

    //XRail DeploymentstringWheel
    private DcMotor xrail = null;

    //Intake Grabber
    private Servo intakeServo = null;
    private double intakeIncrement = .05;

    //Intake Arms
    private Servo armLeft = null;

    private int startingPos = 0;

    private int CurrentGamepad = 1;
    private boolean bothPressed = false;

    private boolean strafing = false;
    private double strafeStartingAngle = 0;
    private BNO055IMU imu;
    private Orientation angles;
    private BNO055IMU.Parameters parameters;
    private int encoderSum = 0;

    private boolean locked = false;
    private int holdPos = 0;

    private double prevFlPos = 0;
    private double prevFrPos = 0;
    private double prevBlPos = 0;
    private double prevBrPos = 0;
    //private double prevSwPos = 0;

    private double prevFlDif = 0;
    private double prevFrDif = 0;
    private double prevBlDif = 0;
    private double prevBrDif = 0;
    //private double prevSwDif = 0;

    private double flSum = 0;
    private double frSum = 0;
    private double blSum = 0;
    private double brSum = 0;

    private int actions = 0;
    private boolean isStopped = true;
    private Servo foundationGrabber = null;
    private double realFlPos = 0;
    private double realFrPos = 0;
    private double realBlPos = 0;
    private double realBrPos = 0;

    private Servo found1 = null;
    private Servo found2 = null;
    private boolean out = false;
    private boolean hasStartedMoving = false;
    private String lastPres = "";
    private String lastDPres = "";
    private boolean hasFinishedFirstAction = false;
    private boolean xrailOut = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public int getSign(double num) {
        if (num > 0) {
            return 1;
        }
        else if (num < 0){
            return -1;
        }
        else {
            return 0;
        }
    }
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

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Xrail init
        stringWheel = hardwareMap.get(DcMotor.class,"string_wheel"); //Port 0
        stringWheel.setDirection(DcMotor.Direction.REVERSE);
        stringWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        startingPos = stringWheel.getCurrentPosition();
        holdPos = stringWheel.getCurrentPosition();

        xrail = hardwareMap.get(DcMotor.class, "xRailMotor");
        xrail.setDirection(DcMotor.Direction.FORWARD);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        armLeft = hardwareMap.get(Servo.class, "armLeft");

        //foundationGrabber = hardwareMap.get(Servo.class, "found1");

        found1 = hardwareMap.get(Servo.class, "found1");
        found2 = hardwareMap.get(Servo.class, "found2");

        telemetry.addData("Status", "Initialized");

        //gamepad1.setJoystickDeadzone((float) .3);
        //gamepad2.setJoystickDeadzone((float) .3);
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        runtime.reset();
    }

    public double compareAngles(AngleUnit angleUnit, double angle) {
        double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);

        return AngleUnit.DEGREES.normalize(degrees);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Setup a variable for each drive wheel to save power level for telemetry

        //Set power for wheels based on math

        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        if (Math.abs(gamepad1.left_stick_x) >= .1){
            if (!strafing) {
                strafing = true;
                encoderSum = frontLeftDrive.getCurrentPosition()+frontRightDrive.getCurrentPosition()+backLeftDrive.getCurrentPosition()+backRightDrive.getCurrentPosition();
                strafeStartingAngle = compareAngles(angles.angleUnit, angles.firstAngle);
            }
        } else {
            strafing = false;
        }
        double v1 = 0;
        double v2 = 0;
        double v3 = 0;
        double v4 = 0;

        double v1correction = 0;
        double v2correction = 0;
        double v3correction = 0;
        double v4correction = 0;
        if (strafing){
            double increment = -.25;
            int acceptableRange = 3;
            int range = 500;
            int newSum = frontLeftDrive.getCurrentPosition()+frontRightDrive.getCurrentPosition()+backLeftDrive.getCurrentPosition()+backRightDrive.getCurrentPosition();
            if (newSum-range > encoderSum){
                v1correction = -increment;
                v2correction = -increment;
                v3correction= -increment;
                v4correction = -increment;
            } else if (newSum+range < encoderSum) {
                v1correction = increment;
                v2correction = increment;
                v3correction = increment;
                v4correction = increment;
            } else if ((compareAngles(angles.angleUnit, angles.firstAngle) - strafeStartingAngle) < -acceptableRange){
                v1correction = -increment;
                v2correction = increment;
                v3correction= -increment;
                v4correction = increment;
                encoderSum = 0;
            } else if ((compareAngles(angles.angleUnit, angles.firstAngle) - strafeStartingAngle) > acceptableRange){
                v1correction = increment;
                v2correction = -increment;
                v3correction= increment;
                v4correction = -increment;
                encoderSum = 0;
            }
            v1 = -strafe + v1correction;
            v2 = strafe + v2correction;
            v3 = strafe + v3correction;
            v4 = -strafe + v4correction;
        } else {
            v1 = drive - strafe - turn + v1correction;
            v2 = drive + strafe + turn + v2correction;
            v3 = drive + strafe - turn + v3correction;
            v4 = drive - strafe + turn + v4correction;
        }

        //Change speed factor
        if (gamepad1.left_bumper && speedFactor >= 0.25 && lastPressed != gamepad1.left_bumper) {
            speedFactor -= .2;
        } else if (gamepad1.right_bumper && speedFactor <= 8.0 && lastPressed != gamepad1.right_bumper) {
            speedFactor += .2;
        }
        lastPressed = gamepad1.left_bumper || gamepad1.right_bumper;

        //Locking Mechanics for the xrail string wheel
        //Movement for the xrail string wheel
        if (gamepad2.left_trigger > 0 ){
            stringWheel.setPower(wheelSpeed);
            holdPos = stringWheel.getCurrentPosition();
        } else if (gamepad2.right_trigger > 0){
            holdPos = stringWheel.getCurrentPosition();
            if ((stringWheel.getCurrentPosition()-1500) > startingPos){
                stringWheel.setPower(wheelSpeed * -1);
            }
        } else {
            stringWheel.setPower(0);
        }
        //gamepad1.right_trigger > 0 && ((stringWheel.getCurrentPosition()-1000)-startingPos

        if (stringWheel.getCurrentPosition() < holdPos && !(gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0)){
            stringWheel.setPower(.1);
        }

        if (gamepad2.dpad_down){
            xrail.setPower(-XrailWheelSpeed);
            xrailOut = true;
        } else if (gamepad2.dpad_up){ //will push it forwards
            xrail.setPower(XrailWheelSpeed);
            xrailOut = false;
        }

        if (xrailOut) { //this way it will set even if we are not always pressing the xRail down
            xrail.setPower(-XrailWheelSpeed);
        }
        else if (!xrailOut && !gamepad2.dpad_up) { //not dpad down and also not current dpad up
            xrail.setPower(0); //prevent it from breaking from going forwards
        }

        //this will only update when driver explicitly wants to go in the other direction

        if (gamepad2.dpad_left){
            lastDPres = "left";
            found1.setPosition(1);
            found2.setPosition(.95);
        } else if (gamepad2.dpad_right){
            lastDPres = "right";
            found1.setPosition(0);
            found2.setPosition(0);
        }

        if (gamepad2.a){
            armLeft.setPosition(.91);
            out = true;
        } else if (gamepad2.b){
            armLeft.setPosition(.1);
            out = false;
        }

        if (out){
            armLeft.setPosition(.91);
        }

        if (gamepad2.x){
            intakeServo.setPosition(1);
            lastPres = "x";
        } else if (gamepad2.y){
            intakeServo.setPosition(0);
            lastPres = "y";
        } else if (lastPres.equals("y")){
            intakeServo.setPosition(.5);
        }

        //find the difference between current and previous encoder values
        //negative if decreasing, positive if increasing
        double flPos = frontLeftDrive.getCurrentPosition();
        double frPos = frontRightDrive.getCurrentPosition();
        double blPos = backLeftDrive.getCurrentPosition();
        double brPos = backRightDrive.getCurrentPosition();

        if (!(flPos == 0 && frPos == 0 && blPos == 0 && brPos == 0)) { //if encoder positions have changed and it has started moving
            hasStartedMoving = true;
        }

        double flDif = flPos - prevFlPos;
        double frDif = frPos - prevFrPos;
        double blDif = blPos - prevBlPos;
        double brDif = brPos - prevBrPos;

        if (flDif == 0 && frDif == 0 && blDif == 0 && brDif == 0 && !isStopped) {
            //if the position of the bot is the same, and the bot is not stopped
            //set new positions
            realFlPos = flPos;
            realFrPos = frPos;
            realBlPos = blPos;
            realBrPos = brPos;

            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isStopped = true;
            //actions++;
        }
        else { //is moving, or has already stopped
            isStopped = false;
        }


        if (!(realFlPos == 0 && realFrPos == 0 && realBlPos == 0 && realBrPos == 0) && hasStartedMoving) { //if the robot has successfully updated new positions, and already has startedMoving
            hasFinishedFirstAction = true;
        }

        if (hasFinishedFirstAction) {
            commands.add((int) realFlPos + " " + (int) realFrPos + " " + (int) realBlPos + " " + (int) realBrPos + " " + gamepad2.dpad_up + " " + gamepad2.dpad_down + " " + gamepad2.a + " " + gamepad2.b + " " + gamepad2.x + " "
                    + gamepad2.y + " " + gamepad2.dpad_left + " " + gamepad2.dpad_right);
        }

        prevFlPos = flPos; //set the current position as the previous
        prevFrPos = frPos;
        prevBlPos = blPos;
        prevBrPos = brPos;

        frontLeftDrive.setPower(v1*speedFactor);
        frontRightDrive.setPower(v2*speedFactor);
        backLeftDrive.setPower(v3*speedFactor);
        backRightDrive.setPower(v4*speedFactor);

        iters+=1;
        telemetry.addData("Iterations:",""+iters);
        telemetry.addData("Actions:",""+actions);
        //telemetry.addData("ServoPosition",foundationGrabber.getPosition());
        telemetry.addData("FL:",frontLeftDrive.getCurrentPosition());
        telemetry.addData("FR:",frontRightDrive.getCurrentPosition());
        telemetry.addData("BL:",backLeftDrive.getCurrentPosition());
        telemetry.addData("BR:",backRightDrive.getCurrentPosition());
        telemetry.addData("FlDif:", flDif);
        telemetry.addData("FrDif:", frDif);
        telemetry.addData("BlDif:", blDif);
        telemetry.addData("BrDif:", brDif);
        //telemetry.addData("flDif")

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        String filename = "FoundationLeft_ParkHigh";
        File file = new File(Environment.getExternalStorageDirectory()+"/"+filename);
        file.delete();
        ReadWriteFile.writeFile(file, commands.toString());

        //telemetry.update();
        telemetry.update();
        //ReadWriteFile.writeFile(file, commands.toString());
    }
}