/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Gonna push this file.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="Simple_Drive", group="Savvy is Awesome with a capital A")  // @Autonomous(...) is the other common choice
//@Disabled
public class Simple_Drive extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
       //DcMotor motorArm;
       //Servo servoGrabber;


    boolean bDebugFrontRight = false;
    boolean bDebugFrontLeft = false;
    boolean bDebugBackRight = false;
    boolean bDebugBackLeft = false;
    //boolean bDebugArm = false;
    //boolean bDebugGrabber = false;
    boolean dontTurn = false;

    //float armRotation = 0;
    //float grabRotation = 0;

    /**
     * Constructor
     */
    public Simple_Drive() {
        telemetry.addData("Beep", String.format("Boop"));

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and created the configuration file.
         */

        /*
         * For this test, we assume the following,
         *   There are four motors
         *   "FrontLeft" and "BackLeft" are front and back left wheels
         *   "FrontRight" and "BackRight" are front and back right wheels
         */
        try {
            motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        }
        catch (IllegalArgumentException iax) {
            bDebugFrontRight = true;
        }
        try{
            motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (IllegalArgumentException iax) {
            bDebugFrontLeft = true;
        }
        try{
            motorBackRight = hardwareMap.dcMotor.get("BackRight");
        }
        catch (IllegalArgumentException iax) {
            bDebugBackRight = true;
        }
        try{
            motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (IllegalArgumentException iax) {
            bDebugBackLeft = true;
        }
//        try{
//            motorArm = hardwareMap.dcMotor.get("arm");
//        }
//        catch(IllegalArgumentException iax){
//            bDebugArm = true;
//        }
//
//        try{
//            servoGrabber = hardwareMap.servo.get("grab");
//        }
//        catch(IllegalArgumentException iax){
//            bDebugGrabber = true;
//        }
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float x1 = gamepad1.left_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float r1 = gamepad1.right_stick_x;
        float y2 = -gamepad2.left_stick_y;
        float y3 = -gamepad2.right_stick_y;

        //lt = half speed
        float lt = gamepad1.left_trigger;
        float lt2 = gamepad2.left_trigger;
        //rt = 2x speed
        float rt = gamepad1.right_trigger;
        float rt2 = gamepad2.right_trigger;

        // clip the right/left values so that the values never exceed +/- 1
        x1 = Range.clip(x1, -1, 1);
        y1 = Range.clip(y1, -1, 1);
        r1 = Range.clip(r1, -1, 1);
        y2 = Range.clip(y2, -1, 1);
        y3 = Range.clip(y3, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        x1 = (float)scaleInput(x1);
        y1 = (float)scaleInput(y1);
        r1 = (float)scaleInput(r1);
        y2 = (float)scaleInput(y2);
        y3 = (float)scaleInput(y3);

        float FrontRight = (y1-x1)-r1;
        float BackRight = (y1+x1)-r1;
        float FrontLeft = (y1+x1+r1);
        float BackLeft= (y1-x1)+r1;

//        float armRotationIncrement = y3;
//        armRotation += armRotationIncrement;
//
//        float grabRotationIncrement = y2/10;
//        grabRotation += grabRotationIncrement;


        if(Math.abs(FrontRight) > 1 && Math.abs(FrontRight) >= Math.abs(BackRight) && Math.abs(FrontRight) >= Math.abs(FrontLeft) && Math.abs(FrontRight) >= Math.abs(BackLeft)){
            BackRight = BackRight / Math.abs(FrontRight);
            FrontLeft = FrontLeft / Math.abs(FrontRight);
            BackLeft = BackLeft / Math.abs(FrontRight);
            FrontRight = FrontRight / Math.abs(FrontRight);
        }
        else if(Math.abs(BackRight) > 1 && Math.abs(BackRight) >= Math.abs(FrontRight) && Math.abs(BackRight) >= Math.abs(FrontLeft) && Math.abs(BackRight) >= Math.abs(BackLeft)){
            FrontRight = FrontRight / Math.abs(BackRight);
            FrontLeft = FrontLeft / Math.abs(BackRight);
            BackLeft = BackLeft / Math.abs(BackRight);
            BackRight = BackRight / Math.abs(BackRight);
        }
        else if(Math.abs(FrontLeft) > 1 && Math.abs(FrontLeft) >= Math.abs(FrontRight) && Math.abs(FrontLeft) >= Math.abs(BackRight) && Math.abs(FrontLeft) >= Math.abs(BackLeft)){
            FrontRight = FrontRight / Math.abs(FrontLeft);
            BackRight = BackRight / Math.abs(FrontLeft);
            BackLeft = BackLeft / Math.abs(FrontLeft);
            FrontLeft = FrontLeft / Math.abs(FrontLeft);
        }
        else if(Math.abs(BackLeft) > 1 && Math.abs(BackLeft) >= Math.abs(FrontRight) && Math.abs(BackLeft) >= Math.abs(BackRight) && Math.abs(BackLeft) >= Math.abs(FrontLeft)){
            FrontRight = FrontRight / Math.abs(BackLeft);
            BackRight = BackRight / Math.abs(BackLeft);
            FrontLeft = FrontLeft / Math.abs(BackLeft);
            BackLeft = BackLeft / Math.abs(BackLeft);
        }
/*
		if(armRotation >= 10){
			armRotation = 10;
			dontTurn = true;
		}
		else if(armRotation <= 0){
			armRotation = 0;
			dontTurn = true;
		}
		else{
			dontTurn = false;
		}

 */
//        dontTurn = false;
//
//        if(grabRotation > .5f){
//            grabRotation = .5f;
//        }
//        else if(grabRotation < 0){
//            grabRotation = 0;
//        }

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

        float arm = y3;
        arm *= (1-(lt2/2));


        // write the values to the motors - for now, front and back motors on each side are set the same
        if (!bDebugFrontRight || !bDebugBackRight || !bDebugFrontLeft || !bDebugBackLeft) {
            if(FrontRight == 0){
                motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else{
                motorFrontRight.setPower(FrontRight);
            }
            if(BackRight == 0){
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else{
                motorBackRight.setPower(BackRight);
            }
            if(FrontLeft == 0){
                motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else{
                motorFrontLeft.setPower(FrontLeft);
            }
            if(BackLeft == 0){
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else{
                motorBackLeft.setPower(BackLeft);
            }
/*
			if(dontTurn = true || arm == 0){
				motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			}
			else if(dontTurn = false){
				motorArm.setPower(arm);
			}

 */
//            motorArm.setPower(arm);
//
//            servoGrabber.setPosition(grabRotation);

      /*
      if(r != 0) {
         x = 0;
         y = 0;
         motorFrontRight.setPower(-r);
         motorBackRight.setPower(-r);
         motorFrontLeft.setPower(r);
         motorBackLeft.setPower(r);
      }
       */
        }

        /*
         * Send telemetry data back to driver station. Note that if we are using
         * a legacy NXT-compatible motor controller, then the getPower() method
         * will return a null value. The legacy NXT-compatible motor controllers
         * are currently write only.
         */
        telemetry.addData("Text", "*** Test v1.0 ***");

        /*
         * Checks for each wheel's power, and if the wheel setup ran into an error,
         * will return 'not working' instead of a power.
         */
        if(!bDebugFrontRight){
            telemetry.addData("front right pwr", String.format("%.2f", FrontRight));
        }
        else{
            telemetry.addData("front right pwr", String.format("not working"));
        }
        if(!bDebugBackRight){
            telemetry.addData("back right pwr", String.format("%.2f", BackRight));
        }
        else{
            telemetry.addData("back right pwr", String.format("not working"));
        }
        if(!bDebugFrontLeft){
            telemetry.addData("front left pwr", String.format("%.2f", FrontLeft));
        }
        else{
            telemetry.addData("front left pwr", String.format("not working"));
        }
        if(!bDebugBackRight){
            telemetry.addData("back left pwr", String.format("%.2f", BackLeft));
        }
        else{
            telemetry.addData("back left pwr", String.format("not working"));
        }
        telemetry.addData("left trigger", String.format("%.2f",lt));
        telemetry.addData("right trigger", String.format("%.2f",rt));
        telemetry.addData("gamepad1", gamepad1);

//        if(!bDebugGrabber){
//            telemetry.addData("grabber rotation", String.format("%.2f",grabRotation));
//        }
//        else{
//            telemetry.addData("grabber rotation", String.format("not working"));
//        }
//        if(!bDebugArm){
//            telemetry.addData("arm rotation", String.format("%.2f",armRotation));
//        }
//        else{
//            telemetry.addData("arm rotation", String.format("&.2f",grabRotation));
//        }
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

//Bruh
