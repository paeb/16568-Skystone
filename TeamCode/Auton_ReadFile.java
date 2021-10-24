package teamcode2.teamcode;

import android.content.Context;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Boolean.parseBoolean;
import static java.lang.Integer.parseInt;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled

@Autonomous(name="Auton_ReadFile", group="Iterative Opmode")

public class Auton_ReadFile extends OpMode {
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private double XrailWheelSpeed = .6;
    private double speedFactor = 1;
    private int commandIndex = 0;
    private Context readFile = null;
    private ArrayList<String> commands = null;
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
    private File directory = new File(String.valueOf(Environment.getExternalStorageDirectory()));
    private boolean flReached = false;
    private boolean frReached = false;
    private boolean blReached = false;
    private boolean brReached = false;
    private boolean swReached = false;
    private boolean notEmpty = false;
    private boolean oneIsNull = false;
    private boolean servoGrab = false;
    private int iter = 0;
    private DcMotor stringWheel = null;
    private DcMotor xrail = null;
    private Servo armLeft = null;
    private Servo intakeServo = null;
    private int lastflPos = 0;
    private int lastfrPos = 0;
    private int lastblPos = 0;
    private int lastbrPos = 0;
    private boolean onFirstIteration = true;
    private double error = 0;
    private double sum_error = 0;
    private double prev_error;
    private double diff_error;
    private double flTargetPos = 0;
    private double frTargetPos = 0;
    private double blTargetPos = 0;
    private double brTargetPos = 0;
    private double kp = 2;
    private double kd = 0.6;
    private double ki = 0.001;
    private double lastFlPos = 1000000000;
    private double lastFrPos = 1000000000;
    private double lastBlPos = 1000000000;
    private double lastBrPos = 1000000000;
    private boolean dLeft = false;
    private boolean dRight = false;
    private boolean locked = false;
    private boolean out = false;
    private double driveSpeed = 0;
    private Servo found1 = null;
    private Servo found2 = null;
    private boolean out2 = true;
    private boolean xrailOut = true;
    private int commandsLength;

    private void DriveTrain(int position1, int position2, int position3, int position4) { //if positive, forwards. if negative, backwards

        double power = 1; //starts at 1, then reduces as it gets closer

        position1 = -position1; //right now, positive number goes backwards, so do opposite
        position2 = -position2;
        position3 = -position3;
        position4 = -position4;

        frontLeftDrive.setTargetPosition(position1);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightDrive.setTargetPosition(position2);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeftDrive.setTargetPosition(position3);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightDrive.setTargetPosition(position4);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    private void PIDcontrol() {
        //here the distance represents the offset from the target Position
        //the error for each motor (Current position) seemed to be different for some reason

        prev_error = error;

        error = Math.abs(frontLeftDrive.getCurrentPosition() - flTargetPos);

        if (flTargetPos == 0) {
            flTargetPos = 100; //random value so we don't throw a divide by 0 error
        }
        error /= flTargetPos; //scales it, halfway becomes error of 0.5

        diff_error = error - prev_error; //difference in the errors for the kd constant,

        sum_error += error;

        driveSpeed = kp * error + ki * sum_error + kd * diff_error;

        frontLeftDrive.setPower(driveSpeed);
        frontRightDrive.setPower(driveSpeed);
        backLeftDrive.setPower(driveSpeed);
        backRightDrive.setPower(driveSpeed);

    }
    @Override
    public void init() {
        //telemetry logging to test recordings
        /**
         telemetry.log().add("Recordings:");
         for (int i=0; i<directory.listFiles().length; i++) {
         if (directory.listFiles()[i].getName().length()>4) {
         if (directory.listFiles()[i].getName().substring(directory.listFiles()[i].getName().length() - 4, directory.listFiles()[i].getName().length()).equals(".txt")) {
         telemetry.log().add(directory.listFiles()[i].getName());
         }
         }
         }
         telemetry.log().add("-----------------------");
         */
        commandIndex = 0;
        String filename = "SingleSkystoneFoundationLeft_ParkLow";

        //get the file from the external storage directory
        File file = new File(Environment.getExternalStorageDirectory()+"/"+filename);

        //telemetry logs for recordings
        /**
         telemetry.log().add(""+(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(",").length));
         telemetry.log().add(""+(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(","))[0]);
         */

        commands = new ArrayList<String>(Arrays.asList(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(",")));
        //turns the array of commands, of string into the arraylist

        //telemetry logs

        //telemetry.log().add(""+commands.get(0));
        telemetry.log().add(commands.toString());
        //telemetry.log().add("test");
        //telemetry.log().add(""+writeText.commands.toArray()[commandIndex].toString().split(",")[0].split(" "));
        telemetry.log().add(""+commands.size());

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fl"); //Port 3
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br"); //Port 0

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("anyCommandSplit",Arrays.toString(commands.get(0).split("~")[0].replaceAll("  "," ").split(" ",0)));

        //string wheel init
        stringWheel = hardwareMap.get(DcMotor.class,"string_wheel"); //Port 0
        stringWheel.setDirection(DcMotor.Direction.REVERSE);

        stringWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //stringWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        xrail = hardwareMap.get(DcMotor.class, "xRailMotor");

        found1 = hardwareMap.get(Servo.class, "found1");
        found2 = hardwareMap.get(Servo.class, "found2");

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        //startingPos = stringWheel.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        commandsLength = commands.toArray().length;

        telemetry.update();
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        //first "line" or iteration

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
                dUp = parseBoolean(s5);
                dDown = parseBoolean(s6);
                aPressed = parseBoolean(s7);
                bPressed = parseBoolean(s8);
                xPressed = parseBoolean(s9);
                yPressed = parseBoolean(s10);
                dLeft = parseBoolean(s11);
                dRight = parseBoolean(s12);
            }

            //if the positions are not all equal, then we know we are on a new action
            onFirstIteration = !(flPos == lastFlPos && frPos == lastFrPos && blPos == lastBlPos && brPos == lastBrPos);

            if (onFirstIteration) { //new iteration, stop and reset
                frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //threshold value to account for the lag, proportionate to power set
            flReached = Math.abs(frontLeftDrive.getCurrentPosition()) >= Math.abs(flPos) - 20;
            frReached = Math.abs(frontRightDrive.getCurrentPosition()) >= Math.abs(frPos) - 20;
            blReached = Math.abs(backLeftDrive.getCurrentPosition()) >= Math.abs(blPos) - 20;
            brReached = Math.abs(backRightDrive.getCurrentPosition()) >= Math.abs(brPos) - 20;

            //we need to make a case, if the positions read by the file are not the same
            //on first iteration and we haven't reached
            if (onFirstIteration && !(flReached && frReached && blReached && brReached)) {
                DriveTrain(flPos, frPos, blPos, brPos); //stops and resets the encoders
                flTargetPos = flPos;
                onFirstIteration = false;
            }
            //not on first iteration, but still haven't reached target
            else if (!onFirstIteration && !(flReached && frReached && blReached && brReached)) {
                PIDcontrol();
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

            if (out2) {
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
        }
        telemetry.update();
    }
    public void stop() {
    }
}