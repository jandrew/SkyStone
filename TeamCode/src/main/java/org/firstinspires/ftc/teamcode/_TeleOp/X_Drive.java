package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode - X-drive
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="X_Drive", group="!Savvy is Awesome with a capital A")  // @Autonomous(...) is the other common choice
//@Disabled
public class X_Drive extends OpMode {
// variables
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

// debug set to false
    boolean bDebugFrontRight = false;
    boolean bDebugFrontLeft = false;
    boolean bDebugBackRight = false;
    boolean bDebugBackLeft = false;

// dont turn set to false
    boolean dontTurn = false;

// I don't know what this means
    public X_Drive() {
        telemetry.addData("status", String.format("ON"));

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
       // debugging

        // chassis motors
        try {
            motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        } catch (IllegalArgumentException iax) {
            bDebugFrontRight = true;
        }
        try {
            motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        } catch (IllegalArgumentException iax) {
            bDebugFrontLeft = true;
        }
        try {
            motorBackRight = hardwareMap.dcMotor.get("BackRight");
        } catch (IllegalArgumentException iax) {
            bDebugBackRight = true;
        }
        try {
            motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        } catch (IllegalArgumentException iax) {
            bDebugBackLeft = true;
        }

    }
    // mapping something to the gamepad. I don't see where these variables are
    // declared. Is this all you have to do to create a variable?
    // What does float mean?
    @Override
    public void loop() {

        float x1 = gamepad1.left_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float r1 = gamepad1.right_stick_x;
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;

        // lt = half speed
        float lt = gamepad1.left_trigger;
        // rt 2x speed
        float rt = gamepad1.right_trigger;
        // lt/rt = half speed
        float lt2 = gamepad2.left_trigger;
        float rt2 = gamepad2.right_trigger;

        // clip the left/right values so that the values never exceed +/- 1
        x1 = Range.clip(x1, -1, 1);
        y1 = Range.clip(y1, -1, 1);
        r1 = Range.clip(r1, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds
        x1 = (float)scaleInput(x1);
        y1 = (float)scaleInput(y1);
        r1 = (float)scaleInput(r1);

        // power set to the motors
        float FrontRight = (y1-x1)-r1;
        float BackRight = (y1+x1)-r1;
        float FrontLeft = (y1+x1+r1);
        float BackLeft= (y1-x1)+r1;

        // what is this math?
        if(Math.abs(FrontRight) > 1 && Math.abs(FrontRight) >= Math.abs(BackRight) && Math.abs(FrontRight) >= Math.abs(FrontLeft) && Math.abs(FrontRight) >= Math.abs(BackLeft)){
            BackRight = BackRight / Math.abs(FrontRight);
            FrontLeft = FrontLeft / Math.abs(FrontRight);
            BackLeft = BackLeft / Math.abs(FrontRight);
            FrontRight = FrontRight / Math.abs(FrontRight);
        } else if(Math.abs(BackRight) > 1 && Math.abs(BackRight) >= Math.abs(FrontRight) && Math.abs(BackRight) >= Math.abs(FrontLeft) && Math.abs(BackRight) >= Math.abs(BackLeft)){
            FrontRight = FrontRight / Math.abs(BackRight);
            FrontLeft = FrontLeft / Math.abs(BackRight);
            BackLeft = BackLeft / Math.abs(BackRight);
            BackRight = BackRight / Math.abs(BackRight);
        } else if(Math.abs(FrontLeft) > 1 && Math.abs(FrontLeft) >= Math.abs(FrontRight) && Math.abs(FrontLeft) >= Math.abs(BackRight) && Math.abs(FrontLeft) >= Math.abs(BackLeft)){
            FrontRight = FrontRight / Math.abs(FrontLeft);
            BackRight = BackRight / Math.abs(FrontLeft);
            BackLeft = BackLeft / Math.abs(FrontLeft);
            FrontLeft = FrontLeft / Math.abs(FrontLeft);
        } else if(Math.abs(BackLeft) > 1 && Math.abs(BackLeft) >= Math.abs(FrontRight) && Math.abs(BackLeft) >= Math.abs(BackRight) && Math.abs(BackLeft) >= Math.abs(FrontLeft)){
            FrontRight = FrontRight / Math.abs(BackLeft);
            BackRight = BackRight / Math.abs(BackLeft);
            FrontLeft = FrontLeft / Math.abs(BackLeft);
            BackLeft = BackLeft / Math.abs(BackLeft);
        }

        //why is this here?
        dontTurn = false;

        // value of motors/servos
        // why does this seem to be here twice? This feels like a code error.
        FrontRight /= 2;
        BackRight /= 2;
        FrontLeft /= 2;
        BackLeft /= 2;
        FrontRight *= (1-(lt/2));
        BackRight *= (1-(lt/2));
        FrontLeft *= (1-(lt/2));
        BackLeft *= (1-(lt/2));
        FrontRight *= (1+rt);
        BackRight *= (1+rt);
        FrontLeft *= (1+rt);
        BackLeft *= (1+rt);

        // if the motors didn't fail but their supposed to be zero then set power to zero
        // write the values to the motors - for now, front and back motors on each side are set the same
        if (!bDebugFrontRight || !bDebugBackRight || !bDebugFrontLeft || !bDebugBackLeft) {
            if (FrontRight == 0) {
                motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motorFrontRight.setPower(FrontRight);
            }
            if (BackRight == 0) {
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motorBackRight.setPower(BackRight);
            }
            if (FrontLeft == 0) {
                motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motorFrontLeft.setPower(FrontLeft);
            }
            if (BackLeft == 0) {
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motorBackLeft.setPower(BackLeft);
            }
        }

        /*
         * Send telemetry data back to driver station. Note that if we are using
         * a legacy NXT-compatible motor controller, then the getPower() method
         * will return a null value. The legacy NXT-compatible motor controllers
         * are currently write only.
         */
        telemetry.addData("status=", "ready");

        /*
         * Checks for each wheel's power, and if the wheel setup ran into an error,
         * will return 'not working' instead of a power.
         */
        if(!bDebugFrontRight){
            telemetry.addData("front right pwr", String.format("%.2f", FrontRight));
        } else{
            telemetry.addData("front right pwr", String.format("not working"));
        }
        if(!bDebugBackRight){
            telemetry.addData("back right pwr", String.format("%.2f", BackRight));
        } else{
            telemetry.addData("back right pwr", String.format("not working"));
        }
        if(!bDebugFrontLeft){
            telemetry.addData("front left pwr", String.format("%.2f", FrontLeft));
        } else{
            telemetry.addData("front left pwr", String.format("not working"));
        }
        if(!bDebugBackRight){
            telemetry.addData("back left pwr", String.format("%.2f", BackLeft));
        } else{
            telemetry.addData("back left pwr", String.format("not working"));
        }
        // gamepad1 controls that control speed
        telemetry.addData("left trigger on gamepad 1", String.format("%.2f",lt));
        telemetry.addData("right trigger on gamepad 1", String.format("%.2f",rt));
        telemetry.addData("gamepad1", gamepad1);


        // gamepad2 controls that control speed
        // do we need this?
        telemetry.addData("left trigger on gamepad 2", String.format("%.2f",lt2));
        telemetry.addData("right trigger on gamepad 2", String.format("%.2f",rt2));
        telemetry.addData("gamepad2", gamepad2);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        return dVal*dVal*dVal;    // maps {-1,1} -> {-1,1}
    }

}
