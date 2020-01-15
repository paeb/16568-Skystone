package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import android.content.Context;
import android.os.Environment;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.URL;
import java.util.Arrays;
import java.util.Scanner;
import java.util.ArrayList;

import static java.lang.Long.parseLong;
import static java.lang.Double.parseDouble;

@Autonomous(name="Auton With Driver", group="Iterative Opmode")
public class ReadFile extends OpMode {
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private double speedFactor = 1;
    private int commandIndex = 0;
    private Context readFile = null;
    private ArrayList<String> commands = null;
    private File directory = new File(String.valueOf(Environment.getExternalStorageDirectory()));
    public void init() {
        telemetry.log().add("Recordings:");
        for (int i=0; i<directory.listFiles().length; i++) {
            if (directory.listFiles()[i].getName().length()>4) {
                if (directory.listFiles()[i].getName().substring(directory.listFiles()[i].getName().length() - 4, directory.listFiles()[i].getName().length()).equals(".txt")) {
                    telemetry.log().add(directory.listFiles()[i].getName());
                }
            }
        }
        telemetry.log().add("-----------------------");
        commandIndex = 0;
        String filename = "autonRecording.txt";
        File file = new File(Environment.getExternalStorageDirectory()+"/"+filename);
        telemetry.log().add(""+(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(",").length));
        telemetry.log().add(""+(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(","))[0]);
        commands = new ArrayList<String>(Arrays.asList(ReadWriteFile.readFile(file).split("\\[")[1].split("\\]")[0].split(",")));
        telemetry.log().add(""+commands.get(0));
        //telemetry.log().add(commands.toString());
        telemetry.log().add("test");
        //telemetry.log().add(""+writeText.commands.toArray()[commandIndex].toString().split(",")[0].split(" "));
        telemetry.log().add(""+commands.size());

        /*for (int i=0; i<writeText.commands.toArray()[0].toString().length(); i++) {
            telemetry.log().add(""+writeText.commands.toArray()[0].toString().charAt(i));
        }*/
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fl"); //Port 3
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br"); //Port 0
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("anyCommandSplit",Arrays.toString(commands.get(0).split("~")[0].replaceAll("  "," ").split(" ",0)));
        telemetry.update();
    }
    public void loop() {
        if (System.currentTimeMillis()>parseLong(commands.get(commandIndex).split("~")[1]) && commandIndex<commands.size()-1) {
            commandIndex += 1;
        }
        if (commandIndex<commands.toArray().length) {
            telemetry.addData("anyCommandSplit",commands.get(0).split("~")[0].replaceAll("  "," ").split(" ", 0));
            telemetry.update();
            drive = parseDouble(commands.get(commandIndex).split("~")[0].replaceAll("  "," ").split(" ",0)[8]);
            strafe = parseDouble(commands.get(commandIndex).split("~")[0].replaceAll("  "," ").split(" ",0)[6]);
            turn = parseDouble(commands.get(commandIndex).split("~")[0].replaceAll("  "," ").split(" ",0)[10]);
            final double v1 = drive - strafe + turn;
            final double v2 = drive + strafe - turn;
            final double v3 = drive + strafe + turn;
            final double v4 = drive - strafe - turn;
            frontLeftDrive.setPower(v1*speedFactor);
            frontRightDrive.setPower(v2*speedFactor);
            backLeftDrive.setPower(v3*speedFactor);
            backRightDrive.setPower(v4*speedFactor);
        } else {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        }
        telemetry.update();
    }
    public void stop() {

    }
}
