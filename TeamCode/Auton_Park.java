package teamcode2.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "Auton_Park")

public class Auton_Park extends OpMode{
    //Declare the DcMotors

    DcMotor fl; //front left motor
    DcMotor fr; //front right motor
    DcMotor bl; //back left motor
    DcMotor br; //back right motor
    private int step = 1;
    private double wheelCircum = (1.97 * 2) * Math.PI;
    private int ticksPerTurn = 1120; //our gear ratio is 1:
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
    private double startingAngle = 0;
    private int target = 0;
    private BNO055IMU.Parameters parameters;

    private int flPosition;
    private int frPosition;
    private int blPosition;
    private int brPosition;

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".

    public void DriveTrain(int distance) { //if positive, forwards. if negative, backwards
        double power = .65;
        double circumTraveled = distance / wheelCircum;
        int position = (int) (ticksPerTurn * circumTraveled);

        if (position < 0) {
            position = -position; //go backwards
        }
        else if (position > 0) {
            position = position; //go forwards
        }
        else {
            position = 0;
        }

        flPosition = fl.getCurrentPosition() + position;
        frPosition = fr.getCurrentPosition() + position;
        blPosition = bl.getCurrentPosition() + position;
        brPosition = br.getCurrentPosition() + position;

        fl.setTargetPosition(flPosition);
        fr.setTargetPosition(frPosition);
        bl.setTargetPosition(blPosition);
        br.setTargetPosition(brPosition);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void starfe(int distance) { //if positive, forwards. if negative, backwards
        double power = .65;
        double circumTraveled = (distance / wheelCircum);
        int position = (int) (ticksPerTurn * circumTraveled);
        position = 1120;

        flPosition = fl.getCurrentPosition() + position;
        frPosition = fr.getCurrentPosition() + position;
        blPosition = bl.getCurrentPosition() + position;
        brPosition = br.getCurrentPosition() + position;

        fl.setTargetPosition(-flPosition);
        fr.setTargetPosition(frPosition);
        bl.setTargetPosition(blPosition);
        br.setTargetPosition(-brPosition);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void turn(int goTo) { //this is opposite to the varsity because our gear train makes
        //forwards negative and backwards positive
        double power = .65;
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

        fl.setDirection(DcMotor.Direction.REVERSE); //because the wheels are pointed in opposite directions
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);


        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        //switch-case structure, to iterate to different tasks

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        switch(step) {
            case -1:
                //note: you may have to change this to bigger than 0.5, since if there is a difference then you keep turning
                if (Math.abs(compareAngles(angles.angleUnit, angles.firstAngle) - startingAngle) >= Math.abs(target-(4+(target/90)*10))){
                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);
                    step = nextStep;
                    break;
                }
                break;
            case 0:
                if (!(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {
                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);

                    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    step = nextStep;
                }
                break;
            case 1:
                DriveTrain(5); //test forward/backward
                nextStep = 2;
                step = 0;
                break;
            case 2:
                //drive forward
                startingAngle = compareAngles(angles.angleUnit, angles.firstAngle);
                turn(-90);
                nextStep = 3;
                step = -1;
                break;
            case 3:
                DriveTrain(23); //test forward/backward
                nextStep = 4;
                step = 0;
                break;
        }


        telemetry.addLine("heading: " + compareAngles(angles.angleUnit, angles.firstAngle));
        telemetry.addLine("turn: "+ (compareAngles(angles.angleUnit, angles.firstAngle) - startingAngle));
        telemetry.addLine("fl: " + fl.getCurrentPosition());
        telemetry.addLine("fr: " + fr.getCurrentPosition());
        telemetry.addLine("bl: " + bl.getCurrentPosition());
        telemetry.addLine("br: " + br.getCurrentPosition());
        telemetry.update();

    }
    @Override
    public void stop() {

    }
}