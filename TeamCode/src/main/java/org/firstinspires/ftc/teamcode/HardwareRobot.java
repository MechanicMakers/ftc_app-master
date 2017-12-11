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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class HardwareRobot
{
    // We declare 4 motors objects for the Mecanum
    public DcMotor  LFD   = null;
    public DcMotor  LRD  = null;
    public DcMotor  RFD     = null;
    public DcMotor  RRD     = null;

// We declare 2 motors objects for the lift
    public DcMotor  LL     = null;
    public DcMotor  LR     = null;

//  We declare 4 servoes objects for the Selfie_Stick
    private Servo AT = null;
    private Servo AA = null;
    private Servo AB = null;
    private Servo AC = null;

// We declare 1 servo objects for the Jewlin
    private Servo LJ = null;

// We declare 2 servoes objects for the Krabber
    private Servo LK = null;
    private Servo RK = null;

// We declare 2 servoes objects for the Grobber
    private Servo T = null;
    private Servo G = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors for the Mecanum
        LFD = ahwMap.get(DcMotor.class, "LFD");
        LRD = ahwMap.get(DcMotor.class, "LRD");
        RFD = ahwMap.get(DcMotor.class, "RFD");
        RRD = ahwMap.get(DcMotor.class, "RRD");

        LFD.setDirection(DcMotorSimple.Direction.FORWARD);
        LRD.setDirection(DcMotorSimple.Direction.FORWARD);
        RFD.setDirection(DcMotorSimple.Direction.REVERSE);
        RRD.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define and Initialize Motors for the Lift
        LL  = ahwMap.get(DcMotor.class, "LL");
        LR = ahwMap.get(DcMotor.class, "LR");

        LL.setDirection(DcMotorSimple.Direction.FORWARD);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power

        LFD.setPower(0.0);
        LRD.setPower(0.0);
        RFD.setPower(0.0);
        RRD.setPower(0.0);
        LL.setPower(0.0);
        LR.setPower(0.0);



        // Define and initialize ALL installed servos for the Selfie_Stick
        AT  = ahwMap.get(Servo.class, "AT");
        AA = ahwMap.get(Servo.class, "AA");
        AB = ahwMap.get(Servo.class, "AB");
        AC = ahwMap.get(Servo.class, "AC");

        // Define and initialize ALL installed servos for the Jewlin
        LJ = ahwMap.get(Servo.class,"lj");

        // Define and initialize ALL installed servos for the Krabber
        LK  = ahwMap.get(Servo.class, "LK");
        RK = ahwMap.get(Servo.class, "RK");


        // Define and initialize ALL installed servos for the Grobber
        T  = ahwMap.get(Servo.class, "T");
        G = ahwMap.get(Servo.class, "G");

        //set all the servoes to they start position

        AT.setPosition(0.0);
        AA.setPosition(0.0);
        AB.setPosition(0.0);
        AC.setPosition(0.0);

        LJ.setPosition(0.0);

        LK.setPosition(0.0);
        RK.setPosition(0.0);

        T.setPosition(0.0);
        G.setPosition(0.0);


    }
 }

