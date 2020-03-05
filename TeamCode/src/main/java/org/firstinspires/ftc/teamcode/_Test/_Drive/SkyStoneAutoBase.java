package org.firstinspires.ftc.teamcode._Test._Drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_SkyStone;

import java.util.ArrayList;


/**
 * a sample base class for Skystone autonomous modes using
 - VfSqPosIntDriveToStep: a Step that uses encoder and gyro input with occasional Vuforia updates to drive to various
 field positions assuming a "squirrely" drive (Meccanum wheels or X-drive) that can move in any direction independent
 of the direction the bot is facing.
 - some other steps that detect the Skystone using Vuforia and compute positions to go to based on where that one is
 * Created by phanau
 */

//@Autonomous(name="Test: Skystone Auto Blue 1", group ="Test")
//@Disabled
public class SkyStoneAutoBase extends OpMode {

    // derived opmodes should implement this function to add their particular sequences to mSequence
    protected void initSequence() {}

    // guide step that uses a gyro and a position integrator to determine how to guide the robot to the target
    class VfSqGyroPosIntGuideStep extends AutoLib.SqGyroPosIntGuideStep {

        VuforiaLib_SkyStone mVLib = null;       // (optional) Vuforia data source
        SensorLib.EncoderGyroPosInt mPosInt;    // Encoder/gyro-based position integrator to keep track of where we are

        public VfSqGyroPosIntGuideStep(OpMode opmode, VuforiaLib_SkyStone vfLib, SensorLib.EncoderGyroPosInt posInt, Position target, float heading,
                                       SensorLib.PID pid, ArrayList<AutoLib.SetPower> motorsteps, float power, double tol)
        {
            super(opmode, posInt, target, heading, pid, motorsteps, power, tol);
            mVLib = vfLib;
            mPosInt = posInt;
        }

        public boolean loop() {

            // if we have Vuforia enabled, try to use info from it, too
            if (mVLib != null) {

                // update Vuforia info and, if we have valid location data, update the position integrator with it
                mVLib.loop(true);       // update Vuforia location info

                // don't believe Vuforia data if we're currently turning (blurry image?)
                float angVel = mPosInt.getGyro().getHeadingVelocity();    // in deg/sec
                boolean turningTooFast = Math.abs(angVel) > 10.0;

                // if we have Vuforia location data, update the position integrator from it.
                // use STATUS and STATUS_INFO associated with the sample to decide how much to believe it.
                // and make sure we DON'T use a SkyStone as a reference!
                if (mVLib.haveLocation() && mVLib.getTrackableStatusInfo() == TrackableResult.STATUS_INFO.NORMAL && !turningTooFast
                        && !mVLib.getVisibleNames().contains(mVLib.getStoneTarget().getName())) {
                    if (mVLib.getTrackableStatus() == TrackableResult.STATUS.TRACKED)
                        mPosInt.setPosition(mVLib.getFieldPosition());
                    else if (mVLib.getTrackableStatus() == TrackableResult.STATUS.EXTENDED_TRACKED)
                        mPosInt.setPosition(mVLib.getFieldPosition(), 0.5f);
                }
            }

            // run the base Step and return what it returns for "done" -
            // note that we're running this code AFTER updating the posInt with Vuforia data ...
            return super.loop();
        }

    }

    // Step: drive to given absolute field position while facing in the given direction using given EncoderGyroPosInt
    // and (optional) Vuforia lib that supplies occasional position updates to that PositionIntegrator
    class VfSqPosIntDriveToStep extends AutoLib.GuidedTerminatedDriveStep {

        SensorLib.EncoderGyroPosInt mPosInt;
        Position mTarget;

        public VfSqPosIntDriveToStep(OpMode opmode, SensorLib.EncoderGyroPosInt posInt, DcMotor[] motors,
                                     float power, SensorLib.PID pid, Position target, float heading, double tolerance, boolean stop)
        {
            super(opmode, new VfSqGyroPosIntGuideStep(opmode, mVLib, posInt, target, heading, pid, null, power, tolerance),
                    new AutoLib.PositionTerminatorStep(opmode, posInt, target, tolerance, stop),
                    motors);

            mPosInt = posInt;
            mTarget = target;
        }

    }

    // Step: use Vuforia to locate the Skystone image and set the given position to where it is so we can go there ---
    // depends on the fact that the target Position object is passed by reference, so we can update it to pass that
    // information along to other steps that have been given the same object.
    class FindSkystoneStep extends AutoLib.LogTimeStep {

        Position mCurrPos;      // current position -- stone is relative to this
        Position mTarget;       // target position we'll update if we see Skystone

        public FindSkystoneStep(OpMode opmode, Position currPos, Position target, float timeout)
        {
            super(opmode, "FindSkystoneStep", timeout);
            mCurrPos = currPos;
            mTarget = target;
        }

        public boolean loop() {

            // if we have Vuforia enabled, try to use info from it, too
            if (mVLib != null) {

                // update Vuforia info
                mVLib.loop(true);       // update Vuforia location info

                // if we have Vuforia location data for a Skystone, update the target position from it.
                if (mVLib.haveLocation() && mVLib.getVisibleNames().contains(mVLib.getStoneTarget().getName())) {
                    if (mVLib.getTrackableStatus() == TrackableResult.STATUS.TRACKED) {
                        VectorF pos = mVLib.getFieldPosition();
                        // Stone "position" reported by Vuforia is actually the position of the camera relative to the Stone
                        // where +X is into the stone and +Y is to the left ...
                        // so we subtract or add depending on which way we're facing in the field coordinate system ...
                        // and remember, the field is +X to the rear while Stones are placed with +X to Blue or Red ...
                        // for now, just set the X position which determines which stone we grab -- they're all at the same Y.
                        mTarget.x = mCurrPos.x - pos.get(1)/MM_TO_INCH;                         // Xabs-Ystone
                        //mTarget.y = mCurrPos.y + pos.get(0)/MM_TO_INCH + ROBOT_LENGTH/2;        // yAbs+Xstone
                        //mTarget.z = mCurrPos.z - pos.get(2)/MM_TO_INCH;
                        return true;        // done!
                    }
                }
            }

            // run the base Step and return what it returns for "done" - i.e. have we timed out?
            // note that we're running this code AFTER trying to determine target with Vuforia data ...
            return super.loop();
        }
    }

    // Step: log the given Position variable to the console so we can see if/when it changes due to Vuforia hit ...
    // depends on the fact that the Position object is passed by reference, so if other steps update it we see those new values.
    class LogPosition extends AutoLib.LogTimeStep {

        OpMode mOpMode;
        Position mPosition;
        String mName;

        public LogPosition(OpMode opMode, String name, Position position, double seconds) {
            super(opMode, name, seconds);
            mOpMode = opMode;
            mName = name;
            mPosition = position;
        }

        public boolean loop()
        {
            mOpMode.telemetry.addData(mName, mPosition);
            return super.loop();
        }
    }

    // a simple Step that computes a new Position (c) by adding an offset (b) to Position a.
    // this happens during sequence run-time in loop(), not when the sequence is constructed in init().
    // depends on the fact that the Position objects a, b, and c are all passed by reference, so we see the latest
    // values of a and b as set by other steps and can supply a new value of c that will be seen by other steps.
    class ComputePositionStep extends AutoLib.Step {

        Position mA, mB, mC;

        public ComputePositionStep(Position a, Position b, Position c)
        {
            mA = a;  mB = b;  mC = c;
        }

        public boolean loop()
        {
            super.loop();
            if (firstLoopCall()) {
                mC.x = mA.x + mB.x;
                mC.y = mA.y + mB.y;
                mC.z = mA.z + mB.z;
            }
            return true;
        }

    }


    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    SensorLib.PID mPid;                     // PID controller for the sequence
    SensorLib.EncoderGyroPosInt mPosInt;    // Encoder/gyro-based position integrator to keep track of where we are
    RobotHardware rh;                       // standard hardware set for these tests
    VuforiaLib_SkyStone mVLib = null;       // (optional) Vuforia data source

    // some robot dimensions we'll need to e.g. position the front of the bot where we want it relative to
    // the centroid of the bot, which is where we track its position on the field
    final float ROBOT_LENGTH = 18.0f;
    final float MM_TO_INCH = 25.4f;

    // parameters needed to create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
    int countsPerRev = 753;		    // for GOBILDA - actually 753.2
    double wheelDiam = 4.7;		    // wheel diameter (in)

    final float SERVO_GRAB = 0.4f;                  // TBD ...
    final float SERVO_RELEASE = 0.7f;               // TBD ...
    final int WRIST_RAISED = countsPerRev/4;        // quarter turn of the wrist motor from initial UP position
    final int WRIST_LOWERED = countsPerRev/2;       // half turn of the wrist motor from initial UP position
    final int LIFT_ONE_INCH = (int)(countsPerRev*1.5f);   // or whatever ... measure this ... TBD
    final int LIFT_STONE_GRAB = LIFT_ONE_INCH*1;          // correct lift height to grab stone ... TBD


    /**
     * Constructor
     */
    public SkyStoneAutoBase() {
        // override default init timeout to prevent timeouts while starting Vuforia on slow phones.
        // need to do it here so it's in effect BEFORE init() is called.
        this.msStuckDetectInit = 10000;
        this.msStuckDetectStop = 10000;
    }

    @Override
    public void init() {

        // get hardware
        rh = new RobotHardware();
        rh.init(this);

        // post instructions to console
        telemetry.addData("Skystone auto base", "");
        telemetry.addData("requires Meccanum or X-drive", "");
        telemetry.addData("", "autonomous point to point");
        telemetry.addData("", "navigation using PositionIntegrator");
        telemetry.addData("", "driven by motor encoders with ");
        telemetry.addData("", "optional occasional Vuforia updates ");

        // create a PID controller for the sequence
        // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
        float Kp = 0.01f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.01f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // (option) Start up Vuforia
        final boolean bUseVuforia = false;
        if (bUseVuforia) {
            mVLib = new VuforiaLib_SkyStone();
            mVLib.init(this);     // pass it this OpMode (so it can do telemetry output)
        }

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // start out not-done
        bDone = false;

        // return to the derived opMode's init() function so it can fill in the auto-sequence
    }

    @Override public void start()
    {
        super.start();

        // Start tracking the data sets we care about.
        if (mVLib != null)
            mVLib.start();
    }

    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    public void stop()
    {
        super.stop();

        // Stop tracking the data sets we care about.
        if (mVLib != null)
            mVLib.stop();
    }
}

