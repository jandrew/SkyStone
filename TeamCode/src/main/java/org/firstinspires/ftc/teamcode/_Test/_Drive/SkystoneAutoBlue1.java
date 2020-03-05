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
 * a sample Skystone autonomous mode derived from the base SkyStoneAutoBase opmode that sets up
 * just about everything but the particular steps you need to do for this opmode.
 */

@Autonomous(name="Test: Skystone Auto Blue 1", group ="Test")
//@Disabled
public class SkystoneAutoBlue1 extends SkyStoneAutoBase {

    /**
     * Constructor
     */
    public SkystoneAutoBlue1() {
        super();
    }

    @Override
    public void init() {

        // create the base for this autonomous OpMode
        super.init();

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 0.4f;     // set this to go at whatever speed you want to move from point to point
        float tol = 1.0f;           // tolerance in inches

        // add a bunch of position integrator "legs" to the sequence -- uses absolute field coordinate system
        // corresponding to Vuforia convention of +X to the rear and +Y to the Blue side

        // get closer to the Skystones so Vuforia can see the Skystone image better
        Position lookLoc = new Position(DistanceUnit.INCH, -36, 40+ROBOT_LENGTH/2, 0., 0);
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid, lookLoc, -180, tol, true));

        // look for the Skystone with the image and update the target Position for the next move to go to it
        // default to the middle Stone if we don't see one ...
        Position skyLoc = new Position(DistanceUnit.INCH, -36, 24+ROBOT_LENGTH/2, 0., 0);
        AutoLib.ConcurrentSequence cs1 = new AutoLib.ConcurrentSequence();
        mSequence.add(cs1);

        cs1.add(new FindSkystoneStep(this, lookLoc, skyLoc, 3.0f));         // look for SkyStone ...
        cs1.add(new LogPosition(this, "skyLoc", skyLoc,0.0f));       // ... and report target position while searching

        // ... and lower the gripper from its initial UP position while we're looking ... leave power on to maintain position
        cs1. add(new AutoLib.EncoderMotorStepAbs(rh.mWrist, 0.5, WRIST_LOWERED, false));

        // drive to the SkyStone if we found it, otherwise to the default (middle) stone.
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid, skyLoc, -180, tol, true));

        // grab the Skystone
        mSequence.add(new AutoLib.ServoStep(rh.mServo, SERVO_GRAB));

        // raise wrist to lift the stone off the ground
        mSequence.add(new AutoLib.EncoderMotorStepAbs(rh.mWrist, 1.0, WRIST_RAISED, false));

        // back up a bit to pull the stone out of the line of stones
        Position pullLoc = new Position(DistanceUnit.INCH, 0, 0, 0., 0);
        mSequence.add(new ComputePositionStep(skyLoc, new Position(DistanceUnit.INCH, 0, 12, 0, 0), pullLoc));
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid, pullLoc, -180, tol, false));

        // go to the Blue Foundation
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 48, 32, 0., 0), -180, tol, true));

        // do the next Steps in parallel: raise the lift a little bit, lower the wrist
        AutoLib.ConcurrentSequence cs2 = new AutoLib.ConcurrentSequence();
        cs2.add(new AutoLib.EncoderMotorStepAbs(rh.mLift, 1.0, LIFT_ONE_INCH, false));
        cs2.add(new AutoLib.EncoderMotorStepAbs(rh.mWrist, 1.0, WRIST_LOWERED, false));
        mSequence.add(cs2);

        // drop the Skystone
        mSequence.add(new AutoLib.ServoStep(rh.mServo, SERVO_RELEASE));

        // next, grab the Foundation by lowering the lift ... might have to do more to actually grab okay ...
        mSequence.add(new AutoLib.EncoderMotorStepAbs(rh.mLift, 1.0, LIFT_ONE_INCH/2, false));

        // drag the Foundation to the Building Area
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 48, 62, 0., 0), -180, tol, true));

        // release the Foundation by raising the lift
        mSequence.add(new AutoLib.EncoderMotorStepAbs(rh.mLift, 1.0, LIFT_ONE_INCH*2, false));

        // slide out of the corridor left by positioning the Foundation
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 62, 0., 0), -180, tol, false));
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -180, tol, false));

        // lower the lift to get under the bridge
        mSequence.add(new AutoLib.EncoderMotorStepAbs(rh.mLift, 1.0, LIFT_ONE_INCH*0, false));

        // return to the quarry for a second SkyStone
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -60, 32, 0., 0), -180, tol, true));

        // grab the Skystone
        mSequence.add(new AutoLib.ServoStep(rh.mServo, SERVO_GRAB));

        // raise wrist to lift the stone off the ground
        mSequence.add(new AutoLib.EncoderMotorStepAbs(rh.mWrist, 1.0, WRIST_RAISED, false));

        // back up a bit to pull the stone out of the line of stones
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -60, 40, 0., 0), -180, tol, false));

        // bring it to the audience end of the Foundation via the Blue Skybridge
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -180, tol, false));
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 24, 60, 0., 0), -90, tol, true));

        // do the next Steps in parallel: raise the lift a little bit, lower the wrist
        AutoLib.ConcurrentSequence cs3 = new AutoLib.ConcurrentSequence();
        cs3.add(new AutoLib.EncoderMotorStepAbs(rh.mLift, 1.0, LIFT_ONE_INCH, false));
        cs3.add(new AutoLib.EncoderMotorStepAbs(rh.mWrist, 1.0, WRIST_LOWERED, false));
        mSequence.add(cs3);

        // drop the Skystone
        mSequence.add(new AutoLib.ServoStep(rh.mServo, SERVO_RELEASE));

        // park under the SkyBridge
        mSequence.add(new VfSqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -90, tol, true));

    }

}

