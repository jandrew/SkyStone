package org.firstinspires.ftc.teamcode._Auto;

/**
 * OpMode to test AutoLib driving of motors by time or encoder counts
 * Created by phanau on 12/14/15.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode._Libs.AutoLib;


/**
 * A test example of autonomous opmode programming using AutoLib classes.
 * Created by phanau on 12/14/15.
 */



@Autonomous(name="AutoBlockRedRight", group ="Test")
//@Disabled
public class AutoBlockRedRight extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    boolean bFirst;                 // true first time loop() is called
    boolean pass = false;

    DcMotor mFr, mBr, mFl, mBl;     // four drive motors (front right, back right, front left, back left)
    DcMotor mArm;               // two arm motors (in-out, up-down) OPTIONAL
    ColorSensor mColorSensor;
    Servo mGrab;

    boolean debug = false;           // run in test/debug mode with dummy motors and data logging
    boolean haveEncoders = true;   // robot has Encoder-based motors

    public AutoBlockRedRight() {
    }

    public void init() {

        AutoLib.HardwareFactory mf = null;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware

        try {
            mFr = mf.getDcMotor("fr");
            mFr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFl = mf.getDcMotor("fl");
            mFl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBr = mf.getDcMotor("br");
            mBr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBl = mf.getDcMotor("bl");
            mBl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch(IllegalArgumentException iax){

        }
        // OPTIONAL arm motors
        try {
            mArm = mf.getDcMotor("arm");
            mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (IllegalArgumentException iax) {
            mArm = null;
        }
        try {
            mColorSensor = mf.getColorSensor("color");
        }
        catch (IllegalArgumentException iax) {
            mColorSensor = null;
        }
        try {
            mGrab = mf.getServo("grab");
        }
        catch (IllegalArgumentException iax) {
            mGrab = null;
        }

        mFl.setDirection(DcMotor.Direction.REVERSE);
        mBl.setDirection(DcMotor.Direction.REVERSE);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // === MAIN DRIVE STUFF ===

        // Opens grabber
        mGrab.setPosition(0);

        // Drives forward to second block
        mSequence.add(new AutoLib.MoveByTimeStep(mFr, mBr, mFl, mBl, .2, 1.25, true));

        if(mColorSensor.red() > 25){
            //doesn't use arm
        }
        else if(mColorSensor.red() < 25 && !pass){
            mSequence.add(new AutoLib.TimedMotorStep(mArm, .75, 2, true));
            mGrab.setPosition(1);
            pass = true;
            mSequence.add(new AutoLib.TimedMotorStep(mArm, -.75, 2, true));
        }

        //moves side to third block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.2,  -.2, .2,  .3, true));

        if(mColorSensor.red() > 25){
            //doesn't use arm
        }
        else if(mColorSensor.red() < 25 && !pass){
            mSequence.add(new AutoLib.TimedMotorStep(mArm, .75, 2, true));
            mGrab.setPosition(1);
            pass = true;
            mSequence.add(new AutoLib.TimedMotorStep(mArm, -.75, 2, true));
        }

        //moves side to fourth block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.2,  -.2, .2,  .3, true));

        if(mColorSensor.red() > 25){
            //doesn't use arm
        }
        else if(mColorSensor.red() < 25 && !pass){
            mSequence.add(new AutoLib.TimedMotorStep(mArm, .75, 2, true));
            mGrab.setPosition(1);
            pass = true;
            mSequence.add(new AutoLib.TimedMotorStep(mArm, -.75, 2, true));
        }

        //moves side to fifth block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.2,  -.2, .2,  .3, true));

        if(mColorSensor.red() > 25){
            //doesn't use arm
        }
        else if(mColorSensor.red() < 25 && !pass){
            mSequence.add(new AutoLib.TimedMotorStep(mArm, .75, 2, true));
            mGrab.setPosition(1);
            pass = true;
            mSequence.add(new AutoLib.TimedMotorStep(mArm, -.75, 2, true));
        }

        //moves side to sixth block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.2,  -.2, .2,  .3, true));

        if(mColorSensor.red() > 25){
            //doesn't use arm
        }
        else if(mColorSensor.red() < 25 && !pass){
            mSequence.add(new AutoLib.TimedMotorStep(mArm, .75, 2, true));
            mGrab.setPosition(1);
            pass = true;
            mSequence.add(new AutoLib.TimedMotorStep(mArm, -.75, 2, true));
        }

        // drives left to foundation
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.5, -.5,  -.5, .5,  1, true));

        // rotate left
        mSequence.add(new AutoLib.TurnByTimeStep(mFr, mBr, mFl, mBl, .2, -.2, .8,  true));

        // drops block and relifts
        mSequence.add(new AutoLib.TimedMotorStep(mArm,.75, 2, true));
        mGrab.setPosition(0);
        mSequence.add(new AutoLib.TimedMotorStep(mArm,-.75, 2, true));

        // start out not-done, first time
        bDone = false;
        bFirst = true;
    }

    public void loop() {
        // reset the timer when we start looping
        if (bFirst) {
            this.resetStartTime();      // OpMode provides a timer
            bFirst = false;
        }

        // until we're done with the root Sequence, perform the current Step(s) each time through the loop
        if (!bDone) {
            bDone = mSequence.loop();       // returns true when we're done

            if (debug)
                telemetry.addData("elapsed time", this.getRuntime());
        }
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }
}
//2ft & 3in