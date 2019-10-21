package org.firstinspires.ftc.teamcode;
//This is a test for state machines, iterative opmode

//import statements
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="State Machine", group="Iterative Opmode")

public class StateMachineExample extends OpMode
{
    //basic variable declarations, for motors, servos, gyro etc.
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo clasp;

    ColorSensor colorSensor;


    //MattBran mech materials
    DcMotor pulley;

    Servo leftHand;

    Servo rightHand;

    public timeState Test;// new timeState(2,5, motors, "forward");;
    DriveAvoidPIDState Testy;

    GyroTurnCWByPID Turn90;


    BNO055IMU imu;


    colorMoveState parkUnderBridge;
    ColorSenseStopState parkUnderBridge2;

    driveState driveToFoundation;
    driveState straighten;
    driveState getOffWall;

    timeState driveBack;

    timeState miniDrive2;

    OnlyClaspState justClasp;
    OnlyClaspState blockClasp;
    OnlyClaspState releaseClasp;


    ClaspState grabbyGrab;

    GyroTurnCCWByPID turnRightAngle;

    driveState moveOnce;

    driveState dragFoundationIn;


    driveState approachBlocks;

    GyroTurnCCWByPID turnBlock;

    OnlyClaspState grabBlock;

    driveState strafeToBlock;
    driveState getOffBlock;


    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        clasp = hardwareMap.servo.get("clasp");

        //leftHand = hardwareMap.servo.get("leftHand");
        //rightHand = hardwareMap.servo.get("rightHand");

        //pulley = hardwareMap.dcMotor.get("pulley");



        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //makes an list of motors, which contains the DcMotors

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);

        //gyro parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        // Set all motors to zero power
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        //State Declarations: initializes objects to represent actions. Parameters are the specifics of the actions
        
        //i.e driveToFoundation moves forwards at .4 speed
        driveToFoundation = new driveState(45, .4, motors, "forward");

        moveOnce = new driveState(30, .5, motors, "forwards");

        justClasp = new OnlyClaspState( clasp, 2,  1);

        blockClasp = new OnlyClaspState( clasp, 2,  .8);

        grabbyGrab = new ClaspState(motors, clasp, 5, "backward", 1, 1);

        parkUnderBridge2 = new ColorSenseStopState(motors, colorSensor, "red", .225, "forward");

        turnRightAngle = new GyroTurnCCWByPID(90, .5, motors, imu);
        driveBack = new timeState(5, .5, motors, "backward");

        releaseClasp = new OnlyClaspState( clasp, 2, 0 );
        straighten = new driveState(5,.5, motors, "right");
        getOffWall = new driveState(2, .5 , motors, "left");
        dragFoundationIn = new driveState(5, .5 , motors, "right");


        //GET BLOCKS, objects initialized that represent actions

        approachBlocks = new driveState(16.25,.5,motors,"forward");

        turnBlock = new GyroTurnCCWByPID(90,.5,motors,imu);

        strafeToBlock = new driveState(4,.5,motors, "left");

        grabBlock = new OnlyClaspState(clasp,1,1);

        getOffBlock = new driveState(2, .5 , motors, "left");











        //Setting up the order
//        driveToFoundation.setNextState(blockClasp);
//        blockClasp.setNextState(driveBack);
//        driveBack.setNextState(dragFoundationIn);
//        dragFoundationIn.setNextState(releaseClasp);
//        releaseClasp.setNextState(straighten);
//        straighten.setNextState(getOffWall);
//        getOffWall.setNextState(parkUnderBridge2);

        //set next states. This makes it so that when once one object/action has terminated, it calls the next object/action
        approachBlocks.setNextState(turnBlock);
        turnBlock.setNextState(strafeToBlock);
        strafeToBlock.setNextState(grabBlock);
        grabBlock.setNextState(getOffBlock);
        getOffWall.setNextState(null);






    }
    //initialize the stateMachine (which is what executes the states)
    @Override
    public void start(){
        machine = new StateMachine(approachBlocks);

    }


    @Override
    public void loop()  {


        telemetry.addData("state", machine.currentState());
        telemetry.addData("state", machine.currentState());

        telemetry.update();
        //machine.update() continuously runs through and executes each state
        machine.update();

    }

    private StateMachine machine;

    @Override
    public void stop() {
    }}
