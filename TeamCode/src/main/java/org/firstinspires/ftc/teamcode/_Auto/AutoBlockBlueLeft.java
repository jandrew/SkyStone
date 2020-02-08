package org.firstinspires.ftc.teamcode._Auto;

/**
 * OpMode to test AutoLib driving of motors by time or encoder counts
 * Created by phanau on 12/14/15.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;


/**
 * A test example of autonomous opmode programming using AutoLib classes.
 * Created by phanau on 12/14/15.
 */



@Autonomous(name="AutoBlockBlueLeft", group ="Test")
//@Disabled
public class AutoBlockBlueLeft extends OpMode {

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

    double count = 500;
    boolean startCount = false;
    float powa = 0;
    float posi = 0;

    public AutoBlockBlueLeft() {
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

        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,-.2, .2,  .2, -.2,  2, true));

        // Drives forward to second block
        mSequence.add(new AutoLib.MoveByTimeStep(mFr, mBr, mFl, mBl, .2, 3.2, true));

        mSequence.add(new AutoLib.TimedMotorStep(mBl,0,2,true));

        //moves side to third block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.21,  -.2, .21,  .9, true));

        mSequence.add(new AutoLib.TimedMotorStep(mBl,0,2,true));

        //moves side to fourth block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.22,  -.2, .22,  .9, true));

        mSequence.add(new AutoLib.TimedMotorStep(mBl,0,2,true));

        //moves side to fifth block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.2,  -.2, .2,  .9, true));

        mSequence.add(new AutoLib.TimedMotorStep(mBl,0,2,true));

        //moves side to sixth block
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.2, -.2,  -.2, .2,  .9, true));

        mSequence.add(new AutoLib.TimedMotorStep(mBl,0,2,true));

        // drives left to foundation
        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,.5, -.5,  -.5, .5,  2, true));

        // rotate left
        //mSequence.add(new AutoLib.TurnByTimeStep(mFr, mBr, mFl, mBl, -.2, .2, .8,  true));

        // drops block and relifts
        mSequence.add(new AutoLib.TimedMotorStep(mArm,.75, 2, true));
        mSequence.add(new AutoLib.TimedMotorStep(mArm,-.75, 2, true));

        mSequence.add(new AutoLib.SideToSide(mFr, mBr, mFl, mBl,-.5, .5,  .5, -.5,  1, true));

        mSequence.add(new AutoLib.MoveByTimeStep(mFr, mBr, mFl, mBl, .2, 1, true));
        // start out not-done, first time
        bDone = false;
        bFirst = true;
        posi = 0;
    }

    public void loop() {


        if((mColorSensor.red() > 24 || this.getRuntime() < 7) && startCount == false){ // if yellow OR time is in the first 5 seconds
            //doesn't use arm
        }
        else if(pass && this.getRuntime() >= count+1 && this.getRuntime() <= count+2){
            powa = -1f;
        }
        else if(this.getRuntime() > count+.5){ // if count has passed one second after seeing a black block AND it hasn't passed a block yet
            pass = true;
            posi = 1;
        }
        else if(mColorSensor.red() <= 24 && !pass && this.getRuntime() <= 20){ // if black AND hasn't passed AND time is past 5 seconds AND time is less than last 10 seconds
            posi = 0;

            powa = .66f;
            if(startCount == false) {
                count = this.getRuntime();
                startCount = true;
            }
        }


        mGrab.setPosition(posi);
        mArm.setPower(powa);
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

        telemetry.addData("color", mColorSensor.red());
        telemetry.addData("pass?", pass);
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }
}
//2ft & 3in
