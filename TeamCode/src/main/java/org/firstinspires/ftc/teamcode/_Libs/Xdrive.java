/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode._Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * v
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Xdrive {
    /* Public OpMode members. */
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor BackLeft = null;
    public DcMotor BackRight = null;
    public DcMotor Shoulder = null;
    public DcMotor Waist = null;
    public DcMotor Intake1 = null;
    public DcMotor Intake2 = null;
    public Servo Xwrist = null;
    public Servo Ywrist = null;
    public Servo Grab = null;

    //bootleeeeeen
    public Boolean hasFrontLeft = Boolean.FALSE;
    public Boolean hasFrontRight = Boolean.FALSE;
    public Boolean hasBackLeft = Boolean.FALSE;
    public Boolean hasBackRight = Boolean.FALSE;
    public Boolean hasShoulder = Boolean.FALSE;
    public Boolean hasWaist = Boolean.FALSE;
    public Boolean hasIntake1 = Boolean.FALSE;
    public Boolean hasIntake2 = Boolean.FALSE;
    public Boolean hasXwrist = Boolean.FALSE;
    public Boolean hasYwrist = Boolean.FALSE;
    public Boolean hasGrab = Boolean.FALSE;

    public static final double FrontRightPower = 0;
    public static final double FrontLeftPower = 0;
    public static final double BackRightPower = 0;
    public static final double BackLeftPower = 0;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Xdrive() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


        hwMap = ahwMap;

        try {
            FrontLeft = hwMap.get(DcMotor.class, "FrontLeft");
            hasFrontLeft = Boolean.TRUE;
            FrontLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("FrontLeft", "Failed");
        }


        try {
            FrontRight = hwMap.get(DcMotor.class, "FrontRight");
            hasFrontRight = Boolean.TRUE;
            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            telemetry.addData("FrontRight", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("FrontRight", "Failed");
        }


        try {
            BackRight = hwMap.get(DcMotor.class, "BackRight");
            hasBackRight = Boolean.TRUE;
            BackRight.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            telemetry.addData("BackRight", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("BackRight", "Failed");
        }


        try {
            BackLeft = hwMap.get(DcMotor.class, "BackLeft");
            hasBackLeft = Boolean.TRUE;
            BackLeft.setDirection(DcMotor.Direction.FORWARD);
            BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            telemetry.addData("BackLeft", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("BackLeft", "Failed");
        }

        try {
            Shoulder = hwMap.get(DcMotor.class, "Shoulder");
            hasShoulder = Boolean.TRUE;
            Shoulder.setDirection(DcMotor.Direction.FORWARD);
            Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            telemetry.addData("Shoulder", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("Shoulder", "Failed");
        }

        try {
            Waist = hwMap.get(DcMotor.class, "Waist");
            hasWaist = Boolean.TRUE;
            Waist.setDirection(DcMotor.Direction.FORWARD);
            Waist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            telemetry.addData("Waist", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("Waist", "Failed");
        }

        try {
            Shoulder = hwMap.get(DcMotor.class, "Shoulder");
            hasShoulder = Boolean.TRUE;
            Shoulder.setDirection(DcMotor.Direction.FORWARD);
            Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            telemetry.addData("Shoulder", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("Shoulder", "Failed");
        }

        try {
            Xwrist = hwMap.get(Servo.class, "Xwrist");
            hasXwrist = Boolean.TRUE;
//            telemetry.addData("Xwrist", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("Xwrist", "Failed");
        }
        try {
            Ywrist = hwMap.get(Servo.class, "Ywrist");
            hasYwrist = Boolean.TRUE;
//            telemetry.addData("Ywrist", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("Ywrist", "Failed");
        }

        try {
            Grab = hwMap.get(Servo.class, "Grab");
            hasGrab = Boolean.TRUE;
//            telemetry.addData("Grab", "Initialized");
        } catch (IllegalArgumentException iax) {
//            telemetry.addData("Grab", "Failed");

//

        }
    }
}

