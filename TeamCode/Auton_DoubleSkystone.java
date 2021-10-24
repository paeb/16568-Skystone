package teamcode2.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Boolean.parseBoolean;
import static java.lang.Integer.parseInt;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "Auton_DoubleSkystone")
//@Disabled
public class Auton_DoubleSkystone extends OpMode{
    //Declare the DcMotors

    DcMotor fl; //front left motor
    DcMotor fr; //front right motor
    DcMotor bl; //back left motor
    DcMotor br; //back right motor
    private int step = 1;
    private double wheelCircum = (1.97 * 2) * Math.PI;
    private int ticksPerTurn = 1120; //our gear ratio is 1:1
    private int targetPosition = 0;
    private int iter = 0;
    private boolean oneIsNull = false;
    private boolean flReached = false;
    private boolean frReached = false;
    private boolean blReached = false;
    private boolean brReached = false;
    private int nextStep = 0;
    private BNO055IMU imu;
    private AngleUnit unit = AngleUnit.DEGREES; //for the gyro
    private double gyroTarget;
    private double gyroRange;
    private double gyroActual;
    private double minSpeed;
    private double addSpeed;
    private ColorSensor colorSensor;
    private BNO055IMU.Parameters parameters;
    private float lastAngle = 0;
    private static double kp = 3.5; //increased a lot
    private static final double kd = 0.8; //reduced to minimize slowed speed
    private static double ki = 0.0001; //small
    private double error = 0;
    private double prev_error;
    private double diff_error;
    private double sum_error;
    private double toBridgeDist;
    private double encoderStartingPosition = 0;
    private double encoderEndingPosition = 0;
    private double encoderMovedPosition = 0;
    double distanceToDrive = 0;
    private boolean secondSkystoneDetection = false;
    private ElapsedTime runTime = new ElapsedTime();
    private double previousRuntime = 0.1; //make it not 0 so it does not throw an error
    private double currentRuntime = 0;
    private DcMotor xrail = null;
    private double startingPos;
    private Servo intakeServo;
    private Servo armLeft;
    private double blackYellowRatioL2 = 0;
    private double blackYellowRatioC2 = 0;
    private double blackYellowRatioR2 = 0;
    private boolean detected = false;
    private double driveSpeed = 0;
    private boolean approachBlock = false;
    private boolean mustStrafe = false;
    private boolean largeDistance = false;

    //vuforia stuff
    private static final CameraDirection CAMERA_CHOICE = FRONT;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AbZUuPf/////AAAAGUmS0Chan00iu7rnRhzu63+JgDtPo889M6dNtjvv+WKxiMJ8w2DgSJdM2/zEI+a759I7DlPj++D2Ryr5sEHAg4k1bGKdo3BKtkSeh8hCy78w0SIwoOACschF/ImuyP/V259ytjiFtEF6TX4teE8zYpQZiVkCQy0CmHI9Ymoa7NEvFEqfb3S4P6SicguAtQ2NSLJUX+Fdn49SEJKvpSyhwyjbrinJbak7GWqBHcp7fGh7TNFcfPFMacXg28XxlvVpQaVNgkvuqolN7wkTiR9ZMg6Fnm0zN4Xjr5lRtDHeE51Y0bZoBUbyLWSA+ts3SyDjDPPUU7GMI+Ed/ifb0csVpM12aOiNr8d+HsfF2Frnzrj2";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    private boolean hasHitCase12 = false;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private VuforiaTrackables targetsSkyStone;
    private List<VuforiaTrackable> allTrackables;
    private double skystoneOffset;
    private double XrailWheelSpeed = .6;
    private boolean skystoneDetected = false;
    private double driveSpeed2 = 0;
    private Servo found1 = null;
    private Servo found2 = null;

    /**
     * file reading variables
     */

    private boolean out = false;
    private boolean locked = false;
    private boolean dLeft = false;
    private boolean dRight = false;
    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;
    private int swPos = 0;
    private boolean dUp = false;
    private boolean dDown = false;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private ArrayList<String> commands = null;
    private int commandIndex = 0;
    private boolean onFirstIteration = true;
    private SkystoneDetection sd = null;
    private double prev_error_turn;
    private double error_turn = 0;
    private double diff_error_turn;
    private double sum_error_turn;
    private String pos = null;
    private int numPos = 0;

    //strafe
    private int flPosition;
    private int frPosition;
    private int blPosition;
    private int brPosition;
    private double strafeAngle;
    private Orientation angles;
    private int acceptableRange = 3;
    private double increment = -.25;
    private double power = 1;
    private int dist = 0;
    private double lastFlPos = 1000000000;
    private double lastFrPos = 1000000000;
    private double lastBlPos = 1000000000;
    private double lastBrPos = 1000000000;
    private double flTargetPos = 0;

    private boolean out2 = true;
    private double gyroError = 0;
    private double gyroPrevError = 0;
    private double gyroDiffError = 0;
    private double gyroSumError = 0;
    private boolean firstTurnIteration = true;
    private int commandsLength = 0;
    private boolean xrailOut = true;
    private float prevHeading = 0;
    private int addDist = 0;

    private void DriveTrain(int distance) { //if positive, forwards. if negative, backwards

        int position = calculateTicks(distance);
        double power;
        if (approachBlock) {
            power = 0.3;
        }
        else {
            power = 0.9;
        }

        position = -position; //right now, positive number goes backwards, so do opposite

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //RunMode is an enum, a construct that defines
        fl.setTargetPosition(position);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setTargetPosition(position);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setTargetPosition(position);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setTargetPosition(position);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    private void DriveTrain2(int position1, int position2, int position3, int position4) { //if positive, forwards. if negative, backwards
        double power = 1; //starts at 0.7, then reduces as it gets closer
        //double power = powerScaleDistance(distance);

        position1 = -position1; //right now, positive number goes backwards, so do opposite
        position2 = -position2; //right now, positive number goes backwards, so do opposite
        position3 = -position3; //right now, positive number goes backwards, so do opposite
        position4 = -position4; //right now, positive number goes backwards, so do opposite

        fl.setTargetPosition(position1);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fr.setTargetPosition(position2);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bl.setTargetPosition(position3);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        br.setTargetPosition(position4);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    private void PIDcontrol() {

        prev_error = error;
        //get the error, or the distance from the target
        double averageCurrentPosition = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition()
                + br.getCurrentPosition()) / 4.0;

        error = Math.abs(targetPosition - averageCurrentPosition);

        error /= targetPosition; //scales it, halfway becomes error of 0.5

        diff_error = error - prev_error; //difference in the errors for the kd constant,

        sum_error += error; //add the error to the total number of errors

        driveSpeed = 1.8 * error + 1.6 * diff_error;

        fl.setPower(driveSpeed);
        fr.setPower(driveSpeed);
        bl.setPower(driveSpeed);
        br.setPower(driveSpeed);
    }
    private void strafe(int distance){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        strafeAngle = compareAngles(angles.angleUnit, angles.firstAngle);
        double circumTraveled = distance / wheelCircum;
        int position = (int) (ticksPerTurn * circumTraveled);

        flPosition = fl.getCurrentPosition() - position;
        frPosition = fr.getCurrentPosition() + position;
        blPosition = bl.getCurrentPosition() + position;
        brPosition = br.getCurrentPosition() - position;

        fl.setTargetPosition(flPosition);
        fr.setTargetPosition(frPosition);
        bl.setTargetPosition(blPosition);
        br.setTargetPosition(brPosition);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    private int calculateTicks(int distance) {
        double circumTraveled = distance / wheelCircum;
        int position = (int) (ticksPerTurn * circumTraveled); //encoder value
        return position;
    }
    private void gyroCorrect(double gyroTarget) { //turn function utilizing the imu

        double gyroActual = getCurrentHeading();
        /**
         * these are constants, should be defined here
         */

        final double gyroRange = 1;
        final double minSpeed = 0.2;
        final double addSpeed = 0.1;

        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; // in case it is negative

        if (delta > 180.0) { //normalize function
            delta -= 360.0; // delta becomes between -180 and 180
            //because the range is from 0-> 180 and -180-> 0 instead of 0-> 360
        }
        if (Math.abs(delta) > gyroRange) { //not close enough to end position
            double gyroMod = delta / 45.0; // if delta is less than 45 and bigger than -45, this will make a scale from
            // -1 to 1

            if (Math.abs(gyroMod) > 1.0) {
                gyroMod = Math.signum(gyroMod); //makes gyroMod -1 or 1 if error is more than 45
                // or less than -45 degrees
            }

            //if the error is more than 180, then the power is positive, and it turns to the left
            //if the error is less than 180, the power in the turn in negative, and it turns to the
            //right
            //if the error is larger, faster speed
            this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            turn(0.0);
        }
    }
    private void gyroTurn(double gyroTarget) { //turn function utilizing the imu, and for 90 degree turns
        double gyroActual = getCurrentHeading();
        /**
         * these are constants, should be defined here
         */
        final double gyroRange = 5; //range of 5 degrees
        final double minSpeed = 0.7;
        final double addSpeed = 0.2;

        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; // in case it is negative

        if (delta > 180.0) { //normalize function
            delta -= 360.0; // delta becomes between -180 and 180
            //because the range is from 0-> 180 and -180-> 0 instead of 0-> 360
        }
        if (Math.abs(delta) > gyroRange) { //not close enough to end position
            //double gyroMod = delta / 45.0; //if delta is less than 45 and bigger than -45, this will make a scale from
            // -1 to 1
            if (firstTurnIteration) {
                double gyroMod = delta / 45.0; // if delta is less than 45 and bigger than -45, this will make a scale from
                // -1 to 1

                if (Math.abs(gyroMod) > 1.0) {
                    gyroMod = Math.signum(gyroMod); //makes gyroMod -1 or 1 if error is more than 45
                    // or less than -45 degrees
                }

                //if the error is more than 180, then the power is positive, and it turns to the left
                //if the error is less than 180, the power in the turn in negative, and it turns to the
                //right
                //if the error is larger, faster speed

                //go full speed at first
                this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
                firstTurnIteration = false;
            }
            else { //PID control
                gyroError = delta / Math.abs(gyroTarget); //for a 90 degree turn, assuming 90 is the target position
                //and so that we do not change the sign of error

                //makes a scale for delta from 0 to 1, the closer it gets the smaller error becomes
                gyroDiffError = gyroError - gyroPrevError;

                gyroSumError += gyroError;

                double turnSpeed = gyroError * 1.3 + gyroDiffError * 0.8 + gyroSumError * 0.0001;
                this.turn(turnSpeed);

                gyroPrevError = gyroError;
            }
            //prevHeading = (float)getCurrentHeading(); //set the last heading
        }
        else {
            turn(0.0);
        }
    }


    private void setLastAngle() {
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getCurrentHeading() { //the relative one, which is after the heading becomes 0
        return gyroActual = getGyroRotation(unit) - lastAngle;
    }

    private void turn(double power) {
        //must set the runMode to run without encoder in order for it to run
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //changed the direction, changed it back bc of the issue
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }

    //gets current angle position, which is relative to initial initialization (not relative to last position)
    private float getGyroRotation(AngleUnit unit) {
        //first angle means the x coordinate, or the heading
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit).firstAngle;
    }
    private void PIDTurn() {
        prev_error_turn = error_turn;
        //0 to 360
        error_turn = gyroTarget - getCurrentHeading();
        error_turn /= gyroTarget;
        diff_error_turn = error_turn - prev_error_turn;

        sum_error_turn += error_turn; //add the error to the total number of errors

        //since error is too large in this case
        driveSpeed2 = 1.5 * error_turn + 0.0001 * sum_error_turn + 0.6 * diff_error_turn;

        this.turn(driveSpeed2);
    }
    private void correctAngle() {
        double angleOffset = getCurrentHeading() - 0;
        if (Math.abs(angleOffset) > 2.5) { //ideally, it should be at zero since
            //the method set last angle makes the new angle 0
            //but if the difference is greater than 2.5, we must correct
            gyroCorrect(-angleOffset); //we want to go in the correct direction
        }
        else {
            setLastAngle(); //if we no longer have to turn, we may have already completed it.
            //in that case, we set the last angle to current angle for future calculations
        }
    }
    public double compareAngles(AngleUnit angleUnit, double angle) {
        double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);

        return AngleUnit.DEGREES.normalize(degrees);
    }
    private boolean targetVisible() {//this now returns the offset of the skystone
        targetVisible = false;
        VuforiaTrackable trackable = allTrackables.get(0);
        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
            targetVisible = true;
        }
        return targetVisible;
    }
    private void simpleDrive(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    private void PIDcontrol3() {
        prev_error = error;

        error = Math.abs(fl.getCurrentPosition() - flTargetPos);

        if (flTargetPos == 0) {
            flTargetPos = 100; //random value so we don't throw a divide by 0 error
        }
        error /= flTargetPos; //scales it, halfway becomes error of 0.5

        diff_error = error - prev_error; //difference in the errors for the kd constant,

        sum_error += error;

        driveSpeed = kp * error + ki * sum_error + kd * diff_error;

        fl.setPower(driveSpeed);
        fr.setPower(driveSpeed);
        bl.setPower(driveSpeed);
        br.setPower(driveSpeed);
    }


    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        fl.setDirection(DcMotor.Direction.FORWARD); //because the wheels are pointed in opposite directions
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        xrail = hardwareMap.get(DcMotor.class, "xRailMotor");
        xrail.setDirection(DcMotor.Direction.FORWARD);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        armLeft = hardwareMap.get(Servo.class, "armLeft");

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        found1 = hardwareMap.get(Servo.class, "found1");
        found2 = hardwareMap.get(Servo.class, "found2");

        //initialize with these parameters
        imu.initialize(parameters);
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        /**
         * VUFORIA
         */
        //camera initialization vuforia, we made a new parameter object because the gyro was the other one
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters2 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters2.vuforiaLicenseKey = VUFORIA_KEY;
        parameters2.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters2);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        //but for us there is only one
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        //set the stone target location
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        /**
         * initialize the phone and coordinates
         */

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 3.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 6.0f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


        ((VuforiaTrackableDefaultListener) allTrackables.get(0).getListener()).setPhoneInformation(robotFromCamera, parameters2.cameraDirection);

        /**
         * init for the file reading
         */
        commandIndex = 0;

        String filename = "SingleSkystoneFoundationLeft_ParkLow";
        File file = new File(Environment.getExternalStorageDirectory()+"/"+filename);
        commands = new ArrayList<String>(Arrays.asList(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(",")));
        commandsLength = commands.toArray().length;
        //File file = new File(Environment.getExternalStorageDirectory()+"/"+filename);
        //commands = new ArrayList<String>(Arrays.asList(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(",")));
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.addLine("Status: Intialized. Ready to go.");
        telemetry.update();
        runTime.reset();

        targetsSkyStone.activate();
    }
    @Override
    public void loop() {
        armLeft.setPosition(.91);

        switch(step) {
            case -7:
                DriveTrain(-5);
                targetPosition = calculateTicks(-5);
                step = 0;
                nextStep = 8;
                break;
            case -6:
                gyroTarget = 0; //go back to 0
                if (Math.abs(gyroTarget - getCurrentHeading()) < 2.5) { //if it veered off the path, turn to adjust
                    gyroTurn(gyroTarget); //if the current heading is not 0, then we have to adjust before driving back
                    step = -1;
                    nextStep = 20;
                }
                else {
                    step = 20;
                }
                break;
            case -5:
                DriveTrain(7);
                targetPosition = calculateTicks(7);
                step = 0;
                nextStep = 3;
                break;
            case -4:
                strafe(-4);
                nextStep = 21;
                step = -2;
                break;
            case -3:
                DriveTrain(-20);
                targetPosition = calculateTicks(-20);
                step = 0;
                nextStep = 15;
                break;
            case -2:
                if (!(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())){
                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);
                    step = nextStep;
                } else {
                    double v1correction = 0;
                    double v2correction = 0;
                    double v3correction = 0;
                    double v4correction = 0;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    if (compareAngles(angles.angleUnit, angles.firstAngle) - strafeAngle < -acceptableRange){
                        v1correction = -increment;
                        v2correction = increment;
                        v3correction= -increment;
                        v4correction = increment;
                    } else if (compareAngles(angles.angleUnit, angles.firstAngle) - strafeAngle > acceptableRange) {
                        v1correction = increment;
                        v2correction = -increment;
                        v3correction = increment;
                        v4correction = -increment;
                    }
                    fl.setPower(power+v1correction);
                    fr.setPower(power+v2correction);
                    bl.setPower(power+v3correction);
                    br.setPower(power+v4correction);
                }
                break;
            case -1:
                if (Math.abs(gyroTarget - getCurrentHeading()) < 5) { //reached target
                    setLastAngle(); //sets the last angle so the next turn will deduct the current angle and make the heading 0
                    turn(0); //set power to 0
                    step = nextStep;
                    gyroSumError = 0;
                }
                else {
                    firstTurnIteration = false; //PID time
                    gyroTurn(gyroTarget);
                }
                break;
            case 0:
                flReached = Math.abs(fl.getCurrentPosition()) >= Math.abs(targetPosition) - 20; //if it's negative it will never stop... but targetPosition would be positive
                frReached = Math.abs(fr.getCurrentPosition()) >= Math.abs(targetPosition) - 20;
                blReached = Math.abs(bl.getCurrentPosition()) >= Math.abs(targetPosition) - 20;
                brReached = Math.abs(br.getCurrentPosition()) >= Math.abs(targetPosition) - 20;

                if (flReached && frReached && blReached && brReached) {
                    //update to see if they are still under the target position
                    step = nextStep;
                    sum_error = 0; //now reset the KI constant
                }
                else {
                    if (!approachBlock && !largeDistance) { //if we are not going forward, use PIDcontrol()
                        PIDcontrol();
                    }
                    else if (largeDistance) {
                        simpleDrive(0.85);
                    }
                    else { //else, lower the speed
                        simpleDrive(0.3);
                    }
                }
                break;
            case 1:
                //all of these actions take time, so we do them while the robot drives to save time
                DriveTrain(21); //drive to get a closer view of the blocks, 7 extra inches to go closer
                targetPosition = calculateTicks(21);
                step = 0;
                nextStep = 2;
                break;
            case 2:
                firstTurnIteration = true;
                gyroTurn(-90); //negative values -> turn towards the right (with intake facing forward)
                gyroTarget = -90;
                step = -1;
                nextStep = -5;
                break;
            case 3:
                try {
                    Thread.sleep(1500);
                }
                catch (InterruptedException e){
                    telemetry.addLine("Error.");
                }
                skystoneDetected = targetVisible(); //will detect the stone
                if (skystoneDetected) { //first center block
                    step = -7;
                    pos = "RIGHT";
                }
                else {
                    step = 5;
                }
                break;
            case 5: //go to the leftmost block, accounts for the -5 offset
                DriveTrain(-10);
                targetPosition = calculateTicks(-10);
                step = 0;
                nextStep = 6;
                break;
            case 6: //check for the leftmost block
                try {
                    Thread.sleep(1500);
                }
                catch (InterruptedException e){
                    telemetry.addLine("Error.");
                }
                skystoneDetected = targetVisible(); //will detect the stone
                if (skystoneDetected) { //first center block
                    pos = "CENTER";
                    step = -7;
                }
                else {
                    step = 7;
                }
                break;
            case 7: //we know it is the right block
                DriveTrain(-14);
                targetPosition = calculateTicks(-14);
                pos = "LEFT";
                step = 0;
                nextStep = 8;
                break;
            case 8:
                firstTurnIteration = true;
                xrail.setPower(XrailWheelSpeed);
                gyroTurn(90); //negative values -> turn towards the right (with intake facing forward)
                gyroTarget = 90;
                step = -1;
                nextStep = 9;
                intakeServo.setPosition(0);
                break;
            case 9:
                //xrail.setPower(XrailWheelSpeed);
                DriveTrain(4); //we have one case to drive up to the block at full power
                targetPosition = calculateTicks(4); //13, but deduct 7
                step = 0;
                nextStep = 10;
                break;
            case 10:
                xrail.setPower(0);
                approachBlock = true;
                DriveTrain(10); //this case drives at lower power and grabs the block at the same time
                targetPosition = calculateTicks(10);
                intakeServo.setPosition(1);
                step = 0;
                nextStep = 11;
                break;
            case 11:
                approachBlock = false;
                xrail.setPower(-XrailWheelSpeed);
                try {
                    Thread.sleep(500);
                }
                catch (Exception e) {
                }
                DriveTrain(-13); //drive back
                targetPosition = calculateTicks(-13);
                step = 0;
                nextStep = 12;
                break;
            case 12:
                xrail.setPower(-0.1);
                firstTurnIteration = true;
                gyroTurn(-90); //negative values -> turn towards the right (with intake facing forward)
                gyroTarget = -90;
                step = -1;
                nextStep = 13;
                break;
            case 13:
                if (pos.equals("CENTER")) { //middle block, so go forward 8 inches
                    dist += 10;
                }
                else if (pos.equals("LEFT")){ //left block, so go forward 14 inches
                    dist += 19; //8 plus 9
                }
                else { //on the right block, no need to adjust
                    dist = 0;
                }
                //otherwise it is right block, and we don't go forward at all
                largeDistance = true;
                DriveTrain(dist + 40 + addDist);
                targetPosition = calculateTicks(dist + 40 + addDist);
                step = 0;
                nextStep = 14;
                break;
            //release the block
            case 14:
                intakeServo.setPosition(0);
                try {
                    Thread.sleep(250);
                }
                catch (Exception e){
                }
                intakeServo.setPosition(1);
                step = 69;
                break;
            case 15:
                xrail.setPower(XrailWheelSpeed);
                //DriveTrain(8);
                //targetPosition = calculateTicks(8);
                try {
                    Thread.sleep(500);
                }
                catch (Exception e){
                }
                xrail.setPower(0);
                //step = 0;
                step = 16;
                break;
            case 16:
                //xrail.setPower(0); //stop moving the xRail
                intakeServo.setPosition(0); //release the block
                try { //wait to release the block
                    Thread.sleep(400);
                }
                catch(Exception e){
                    telemetry.addLine("Error!");
                }
                step = 17;
                break;
            case 17:
                largeDistance = true;
                DriveTrain(-2);
                targetPosition = calculateTicks(-2);
                step = 0;
                nextStep = 18;
                break;
            case 18:
                largeDistance = false;
                xrail.setPower(-XrailWheelSpeed);
                try {
                    Thread.sleep(800);
                }
                catch (Exception e) {
                }
                step = 19;
                break;
            case 69:
                if (commandIndex < commandsLength) { //if there are more actions
                    telemetry.addLine("working.");

                    String s1 = "";
                    String s2 = "";
                    String s3 = "";
                    String s4 = "";
                    String s5 = "";
                    String s6 = "";
                    String s7 = "";
                    String s8 = "";
                    String s9 = "";
                    String s10 = "";
                    String s11 = "";
                    String s12 = "";

                    int count = 0;
                    String[] array = commands.get(commandIndex).replaceAll("  ", " ").split(" ", 0);
                    for (String e: array) { //each string in the line
                        if (count < array.length) {
                            if (e.equals("")) { //one of these strings is ""
                                oneIsNull = true;
                            }
                            else {
                                //telemetry.log().add(e);
                                if (s1.equals("")) {
                                    s1 = e;
                                }
                                else if (s2.equals("")) {
                                    s2 = e;
                                }
                                else if (s3.equals("")) {
                                    s3 = e;
                                }
                                else if (s4.equals("")) {
                                    s4 = e;
                                }
                                else if (s5.equals("")) {
                                    s5 = e;
                                }
                                else if (s6.equals("")) {
                                    s6 = e;
                                }
                                else if (s7.equals("")) {
                                    s7 = e;
                                }
                                else if (s8.equals("")) {
                                    s8 = e;
                                }
                                else if (s9.equals("")) {
                                    s9 = e;
                                }
                                else if (s10.equals("")) {
                                    s10 = e;
                                }
                                else if (s11.equals("")) {
                                    s11 = e;
                                }
                                else {
                                    s12 = e;
                                }
                            }
                            count++;
                        }
                    }

                    if (!s1.equals("") && !s2.equals("") && !s3.equals("") && !s4.equals("") && !s5.equals("") && !s6.equals("")
                            && !s7.equals("") && !s8.equals("") && !s9.equals("") && !s10.equals("") && !s11.equals("") && !s12.equals("")) { //if none are empty
                        flPos = -parseInt(s1);
                        frPos = -parseInt(s2);
                        blPos = -parseInt(s3);
                        brPos = -parseInt(s4); //it's usually the opposite
                        //swPos = parseInt(s5);
                        dUp = parseBoolean(s5);
                        dDown = parseBoolean(s6);
                        aPressed = parseBoolean(s7);
                        bPressed = parseBoolean(s8);
                        xPressed = parseBoolean(s9);
                        yPressed = parseBoolean(s10);
                        dLeft = parseBoolean(s11);
                        dRight = parseBoolean(s12);
                        //servoGrab = parseBoolean(s5);
                    }

                    //if the positions are not all equal, then we know we are on a new action
                    onFirstIteration = !(flPos == lastFlPos && frPos == lastFrPos && blPos == lastBlPos && brPos == lastBrPos);

                    if (onFirstIteration) { //new iteration, stop and reset
                        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        //frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }

                    flReached = Math.abs(fl.getCurrentPosition()) >= Math.abs(flPos) - 20;
                    //threshold value to account for the lag, proportionate to power set
                    frReached = Math.abs(fr.getCurrentPosition()) >= Math.abs(frPos) - 20;
                    blReached = Math.abs(bl.getCurrentPosition()) >= Math.abs(blPos) - 20;
                    brReached = Math.abs(br.getCurrentPosition()) >= Math.abs(brPos) - 20;

                    //we need to make a case, if the positions read by the file are not the same
                    //on first iteration and we haven't reached
                    if (onFirstIteration && !(flReached && frReached && blReached && brReached)) {
                        DriveTrain2(flPos, frPos, blPos, brPos); //stops and resets the encoders
                        flTargetPos = flPos;
                        onFirstIteration = false;
                    }
                    //not on first iteration, but still haven't reached
                    else if (!onFirstIteration && !(flReached && frReached && blReached && brReached)) {
                        PIDcontrol3();
                    }
                    else { //if target is reached
                        commandIndex++;
                        onFirstIteration = true;
                        sum_error = 0.001;
                    }

                    lastFlPos = flPos;
                    lastFrPos = frPos;
                    lastBlPos = blPos;
                    lastBrPos = brPos;

                    if (dDown){
                        xrail.setPower(-XrailWheelSpeed);
                        xrailOut = true;
                    } else if (dUp){ //will push it forwards
                        xrail.setPower(XrailWheelSpeed);
                        xrailOut = false;
                    }

                    if (xrailOut) { //this way it will set even if we are not always pressing the xRail down
                        xrail.setPower(-XrailWheelSpeed);
                    }
                    else if (!xrailOut && !dUp) { //not dpad down and also not current dpad up
                        xrail.setPower(0); //prevent it from breaking from going forwards
                    }

                    if (aPressed){
                        armLeft.setPosition(.91);
                        out = true;
                    } else if (bPressed) {
                        armLeft.setPosition(.1);
                        out = false;
                    }

                    if (out){
                        armLeft.setPosition(.91);
                    }

                    if (xPressed){
                        intakeServo.setPosition(1);
                        out2 = true;
                    } else if (yPressed){
                        intakeServo.setPosition(0);
                        out2 = false;
                    }

                    if (dLeft){
                        //lastDPres = "left";
                        found1.setPosition(1);
                        found2.setPosition(.95);
                    } else if (dRight){
                        //lastPres = "right";
                        found1.setPosition(0);
                        found2.setPosition(0);
                    }

                    if (out2){
                        intakeServo.setPosition(1);
                    }

                    // telemetry.log().add(oneIsNull + "");
                    // telemetry.log().add(count + "");
                    telemetry.log().add(flPos + "");
                    telemetry.log().add(frPos + "");
                    telemetry.log().add(blPos + "");
                    telemetry.log().add(brPos + "");
                    telemetry.log().add(driveSpeed + "");
                    //telemetry.log().add(servoGrab + "");
                    telemetry.log().add(" ");
                    telemetry.log().add(" ");
                    telemetry.log().add(" ");
                    telemetry.log().add(" ");
                    //commandIndex++;
                    iter++;

                    telemetry.addData("Iters:", iter);
                } else { //if we are finished
                    step = 19;
                }
                break;
            case 19:
                largeDistance = true;
                if (dist != 0) { //if the skystone is not the right block, then it is either the middle or left
                    dist = -7;
                    mustStrafe = true;
                }
                else { //three blocks away, for the right stone
                    dist = 0; //7 more
                    mustStrafe = false;
                }
                DriveTrain(dist - addDist); //go back a negative amount
                targetPosition = calculateTicks(dist - addDist);
                step = 0;
                nextStep = 20;
                break;
            case 20:
                largeDistance = false;
                firstTurnIteration = true;
                xrail.setPower(XrailWheelSpeed); //raise the xRail
                intakeServo.setPosition(0); //open for the grab
                gyroTurn(90); //negative values -> turn towards the right (with intake facing forward)
                gyroTarget = 90;
                step = -1;
                if (mustStrafe) {
                    nextStep = -4;
                }
                else {
                    nextStep = 21;
                }
                break;
            case 21:
                DriveTrain(10); //we have one case to drive up to the block at full power, closer to the block
                targetPosition = calculateTicks(10);
                step = 0;
                nextStep = 22;
                break;
            case 22:
                //xrail.setPower(0);
                approachBlock = true;
                DriveTrain(10); //this case drives at lower power and grabs the block at the same time
                targetPosition = calculateTicks(10);
                intakeServo.setPosition(1);
                step = 0;
                nextStep = 23;
                break;
            case 23:
                approachBlock = false;
                xrail.setPower(-XrailWheelSpeed);
                DriveTrain(-10); //drive back
                targetPosition = calculateTicks(-10);
                step = 0;
                nextStep = 24;
                break;
            case 24:
                xrail.setPower(-0.25);
                firstTurnIteration = true;
                gyroTurn(-90); //negative values -> turn towards the right (with intake facing forward)
                gyroTarget = -90;
                step = -1;
                nextStep = 25;
                break;
            case 25:
                largeDistance = true;
                DriveTrain(-dist + 38 + addDist); //now go forward, in the opposite direction
                targetPosition = calculateTicks(-dist + 38 + addDist);
                step = 0;
                nextStep = 29;
                break;
            case 26:
                //largeDistance = false;
                DriveTrain(27); //drive to the bridge
                targetPosition = calculateTicks(27);
                step = 0;
                nextStep = 29;
                break;
            case 27:
                xrail.setPower(XrailWheelSpeed);
                try {
                    Thread.sleep(800);
                }
                catch (Exception e) {
                }
                xrail.setPower(0);
                step = 28;
                break;
            case 28:
                xrail.setPower(0); //stop moving the xRail
                intakeServo.setPosition(0); //release the block
                try { //wait to release the block
                    Thread.sleep(750);
                }
                catch(Exception e){
                    telemetry.addLine("Error!");
                }
                step = 29;
                break;
            case 29:
                fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                simpleDrive(-1);
                try {
                    Thread.sleep(1000);
                }
                catch (Exception e) {
                    telemetry.addLine("Issue!");
                }
                simpleDrive(0);
                step = 30;
                break;
            case 30:
                xrail.setPower(0);
                telemetry.addLine("Finished!");
                break;
        }

        if (detected) {
            telemetry.addData("Skystone Position:", pos);
        }

        telemetry.addData("blackYellowRatioL", blackYellowRatioL2);
        telemetry.addData("blackYellowRatioC", blackYellowRatioC2);
        telemetry.addData("blackYellowRatioR", blackYellowRatioR2);
        telemetry.addData("Angle Difference", -getCurrentHeading());
        telemetry.addData("DriveSpeed:", driveSpeed2);
        telemetry.addData("Current Heading", getCurrentHeading());
        telemetry.addData("Error_turn,", error_turn);
        telemetry.addData("Prev_turn", prev_error_turn);
        telemetry.addData("diff_turn", diff_error_turn);
        telemetry.addData("Case:", step);
        telemetry.update();
    }
    @Override
    public void stop() {
    }
}