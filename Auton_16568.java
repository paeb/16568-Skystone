package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name = "AutonTest4.0.1")

//@Disabled
public class Auton_16568 extends OpMode{
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
    private AngleUnit unit = AngleUnit.DEGREES; //for the gyro
    private double gyroTarget;
    private double gyroRange;
    private double gyroActual;
    private double minSpeed;
    private double addSpeed;

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

    public double powerScaleDistance(int distance) {
        double drivePower = 0;

        int distanceTravelled = Math.abs(distance);

        if (distanceTravelled >= 24) {
            drivePower = 1;
        }
        else if (distanceTravelled < 24 && distanceTravelled >= 12) {
            drivePower = 0.75;
        }
        else if (distanceTravelled < 12) {
            drivePower = 0.5;
        }

        return drivePower;

    }
    public int strafe(double distance, String direction) {
        //guess and check, then check formulas later on
        //assuming a similar relationship from distance to ticks in the drive train, probably going to
        //be much less

        //how much the wheel actually needs to turn
        double circumferenceTraveled = distance / wheelCircum;

        //here position must be positive
        int position = (int) (ticksPerTurn * circumferenceTraveled);

        double power = powerScaleDistance((int) distance);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (direction.equalsIgnoreCase("left")) {
            position = position;
        }
        else { //for right
            position = -position;
        }
        fl.setTargetPosition(position);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setTargetPosition(-position);
        if (direction.equalsIgnoreCase("left")) {
            position = -position;
        }
        else { //for right
            position = position;
        }
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setTargetPosition(-position);
        if (direction.equalsIgnoreCase("left")) {
            position = -position;
        }
        else { //for right
            position = position;
        }
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setTargetPosition(position);
        if (direction.equalsIgnoreCase("left")) {
            position = position;
        }
        else { //for right
            position = -position;
        }
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        return position;

    }
    public void gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed,
                            double addSpeed) { //complicated turn function

        //while ((gyroTarget - getGyroRotation(unit) + 360.0) % 360.0 < 0.5) {

            //this makes it an actual angle between 0 and 360 anyways, so it doesn't matter if the target
            //is weird for the gyro or actually between 0 and 360
            double delta = (gyroTarget - gyroActual + 360.0) % 360.0; // in case it is negative

            if (delta > 180.0) {
                delta -= 360.0; // delta becomes between -180 and 180
                //because the range is from 0-> 180 and -180-> 0 instead of 0-> 360
            }

            if (Math.abs(delta) > gyroRange) {
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
            } else {
                turn(0.0);
            }
        //}
    }

    public void turn(double power) { //this is opposite to the varsity because our gear train makes
        //forwards negative and backwards positive
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }

    //gets current angle position
    public float getGyroRotation(AngleUnit unit) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit).firstAngle;
    }

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        fl.setDirection(DcMotor.Direction.FORWARD); //because the wheels are pointed in opposite directions
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {

        //switch-case structure, to iterate to different tasks. Once this is done, transition to
        //state machines like the other teams are doing
        switch(step) {
            case -1:
                if ((gyroTarget - getGyroRotation(unit) + 360.0) % 360.0 < 0.5) { //if the target is off?
                    gyroCorrect(gyroTarget, gyroRange, gyroActual, minSpeed, addSpeed);
                }
                else {
                    step = nextStep;
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

            case 2:
                targetPosition = strafe(16, "left"); //test strafe left/right
                nextStep = 3;
                step = 0;
                break;
            case 3:
                gyroTarget = 270; //gyro, imu turn
                gyroRange = 5;
                gyroActual = getGyroRotation(unit);
                minSpeed = 0.4;
                addSpeed = 0.1;
                nextStep = 4;
                step = -1;
                break;


        }


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
