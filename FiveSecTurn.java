package org.firstinspires.ftc.teamcode;

//Fixed Strafing
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Five Second Turn", group="Iterative Opmode")
public class FiveSecTurn extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private long startTime = 0;
    private double distance = 60.69;
    public int boolToInt(boolean bruh) {
        if (bruh==true) {
            return 1;
        } else {
            return 0;
        }
    }
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        startTime = System.currentTimeMillis();
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        //servo.setPosition(0);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        // Tell the driver that initialization is complete.
    }
    public void loop() {
        telemetry.addData("Large Brain",(((double)(System.currentTimeMillis()-startTime))/1000.0));
        if (((double)(System.currentTimeMillis()-startTime))/1000.0 < 5.0) {
            frontLeft.setPower(1.0);
            frontRight.setPower(-1.0);
            backLeft.setPower(1.0);
            backRight.setPower(-1.0);
        } else {
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
        }
    }
}
