package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "AutonTest8.0.0")

public class BasicOpMode_Auton3 extends OpMode{
    //Declare the DcMotors

    DcMotor fl; //front left motor
    DcMotor fr; //front right motor
    DcMotor bl; //back left motor
    DcMotor br; //back right motor
    private int step = 1;
    private double wheelCircum = (1.97 * 2) * Math.PI;
    private int ticksPerTurn = 1120; //our gear ratio is 1:1
    private int targetPosition = 0;
    private boolean flReached = false;
    private boolean frReached = false;
    private boolean blReached = false;
    private boolean brReached = false;
    private int nextStep = 0;
    private BNO055IMU imu;
    private Orientation angles;
    private double gyroTarget;
    private double gyroRange;
    private double gyroActual;
    private double minSpeed;
    private double addSpeed;
    private double currentAngle = 0;
    private int target = 0;
    private BNO055IMU.Parameters parameters;


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".

    public int DriveTrain(int distance) { //if positive, forwards. if negative, backwards
        double circumTraveled = distance / wheelCircum;

        int position = (int) (ticksPerTurn * circumTraveled); //encoder value

        double power = 0.65;
        //double power = powerScaleDistance(distance);

        if (position < 0) {
            position = -position; //go backwards
        }
        else if (position > 0) {
            position = position; //go forwards
        }
        else {
            position = 0;
        }

        //power-scale factor, which is based on the distance needed to travel

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //RunMode is an enum, a construct that defines
        //possible values, like STOP_AND_RESET_ENCODER
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setTargetPosition(position);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setTargetPosition(position);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setTargetPosition(position);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setTargetPosition(position);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        return position;

    }
    //public double Turn

    public void turn(int goTo) { //this is opposite to the varsity because our gear train makes
        //forwards negative and backwards positive
        double power = .5;
        target = goTo;
        if (goTo < 0){
            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(power);
            br.setPower(-power);
        } else if (goTo > 0){
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
        }
    }

    public double compareAngles(AngleUnit angleUnit, double angle) {
        double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);

        return AngleUnit.DEGREES.normalize(degrees);
    }

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.FORWARD); //because the wheels are pointed in opposite directions
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        //initialize with these parameter
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }
    @Override
    public void start() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    @Override
    public void loop() {

        //switch-case structure, to iterate to different tasks. Once this is done, transition to
        //state machines like the other teams are doing

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        switch(step) {
            case -1:
                //note: you may have to change this to bigger than 0.5, since if there is a difference then you keep turning
                if (compareAngles(angles.angleUnit, currentAngle) >= target){
                    step = nextStep;
                    break;
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
                }

                break;
            case 1:
                targetPosition = DriveTrain(16); //test forward/backward
                nextStep = 2;
                step = 0;
                break;
            //this case is specifically for the color sensor, drives and stops right under the bridge. if it doesn't work pavan's a boomer
            case 2:
                //drive forward
                currentAngle = compareAngles(angles.angleUnit, angles.firstAngle);
                turn(90);
                nextStep = 3;
                step = -1;
            case 3:
                //vuforia case, if it detects it drives forward
        }


        telemetry.addLine("heading: " + compareAngles(angles.angleUnit, angles.firstAngle));
        telemetry.update();

    }
    @Override
    public void stop() {

    }
}
