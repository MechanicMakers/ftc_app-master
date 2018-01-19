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

import android.view.ScaleGestureDetector;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *

 */
public class HardwareMapRobot
{

    /* Public OpMode members. */

    //Mecanum
    DcMotor LFD = null;
    DcMotor LRD = null;
    DcMotor RFD = null;
    DcMotor RRD = null;

    double thrshold = 0.1;
    double x1 = 0.0;
    double x2 = 0.0;
    double y1 = 0.0;

    //Jewlin
    Servo LJ = null;
    DistanceSensor LJDS = null;
    ColorSensor LJCS = null;

    Servo RJ = null;
    DistanceSensor RJDS = null;
    ColorSensor RJCS = null;

    //Elevator
    DcMotor EL = null;
    DcMotor ER = null;

    //Pizza
    Servo LP = null;
    Servo RP = null;

    //Collctor
    DcMotor CL = null;
    DcMotor CR = null;

    //Lifter
    Servo LL = null;
    Servo RL = null;

    //Relic
    Servo RT = null;
    Servo RG = null;
    Servo RAT = null;
    Servo RAA = null;
    Servo RAB = null;
    Servo RAC = null;

    //Camera
    VuforiaLocalizer vuforiaL;
    String pos;
    //sencors
    ColorSensor BC;
    SensorBNO055IMU imu = new SensorBNO055IMU();
    ModernRoboticsI2cRangeSensor FDS;

    /* HardwareMap */
    HardwareMap RobotHardwareMap =  null;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap RobotHardwareMap) {
        // Save reference to Hardware map
        //Mecanum
        LFD = RobotHardwareMap.get(DcMotor.class, "LFD");
        LRD = RobotHardwareMap.get(DcMotor.class, "LRD");
        RFD = RobotHardwareMap.get(DcMotor.class, "RFD");
        RRD = RobotHardwareMap.get(DcMotor.class, "RRD");

        LFD.setDirection(DcMotor.Direction.REVERSE);
        LRD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.FORWARD);
        RRD.setDirection(DcMotor.Direction.FORWARD);

        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Jewlin
        LJ  = RobotHardwareMap.get(Servo.class, "LJ");

        LJCS = RobotHardwareMap.get(ColorSensor.class, "LJCS");
        LJDS = RobotHardwareMap.get(DistanceSensor.class, "LJDS");

        LJ.setDirection(Servo.Direction.FORWARD);
        LJ.scaleRange(0, 1);

        RJ = RobotHardwareMap.get(Servo.class, "RJ");

        RJCS = RobotHardwareMap.get(ColorSensor.class, "RJCS");
        RJDS = RobotHardwareMap.get(DistanceSensor.class, "RJDS");

        RJ.setDirection(Servo.Direction.REVERSE);
        RJ.scaleRange(0, 1);

        //elevator
        EL = RobotHardwareMap.get(DcMotor.class, "EL");
        ER = RobotHardwareMap.get(DcMotor.class,"ER");

        EL.setDirection(DcMotor.Direction.REVERSE);
        ER.setDirection(DcMotor.Direction.FORWARD);

        EL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ER.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PIZZA :P
        LP = RobotHardwareMap.get(Servo.class, "PL");
        RP = RobotHardwareMap.get(Servo.class, "PR");

        RP.setDirection(Servo.Direction.REVERSE);
        LP.setDirection(Servo.Direction.FORWARD);

        //Collctor
        CL = RobotHardwareMap.get(DcMotor.class, "CL");
        CR = RobotHardwareMap.get(DcMotor.class, "CR");

        CR.setDirection(DcMotor.Direction.REVERSE);
        CL.setDirection(DcMotor.Direction.FORWARD);

        CR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lifter
        RL = RobotHardwareMap.get(Servo.class, "LR");
        LL = RobotHardwareMap.get(Servo.class, "LL");

        LL.setDirection(Servo.Direction.REVERSE);
        RL.setDirection(Servo.Direction.REVERSE);

        //Relic
        RG = RobotHardwareMap.get(Servo.class, "RG");
        RT = RobotHardwareMap.get(Servo.class, "RT");
        RAA = RobotHardwareMap.get(Servo.class, "RAA");
        RAB = RobotHardwareMap.get(Servo.class, "RAB");
        RAC = RobotHardwareMap.get(Servo.class, "RAC");
        RAT = RobotHardwareMap.get(Servo.class, "RAT"); //mouse!

        RG.setDirection(Servo.Direction.FORWARD);
        RT.setDirection(Servo.Direction.FORWARD);
        RAA.setDirection(Servo.Direction.FORWARD);
        RAB.setDirection(Servo.Direction.REVERSE);
        RAC.setDirection(Servo.Direction.FORWARD);
        RAT.setDirection(Servo.Direction.FORWARD);
        RAT.scaleRange(-1.0,1.0);
        //Sencor
        BC = RobotHardwareMap.get(ColorSensor.class, "BC");
        imu.init();



        //camera
        // Where the Display Will be Placed
        int cameraMonitorViewId= RobotHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", RobotHardwareMap.appContext.getPackageName());
        //Set up the monitor to the current Vuforia Object
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Set up the License Key to the current Vuforia Object
        parameters.vuforiaLicenseKey = "AR+/1Xj/////AAAAmZVeX1TclkDftf0EjQhj+idSNF6vpGAY0vWg1laTAVg+MedtLFjicjlrOlqdduoKW33j/ITg4YIsQ3OvH8wXq1Wp/6ojRAa9HkHIlpheD/WnAK86Vm4QgTL2uOnksDLu6/zFiVledm6pgfLTsq0Wu3ySl2uc3tHAP8MA4QuENWN1qjWheRAJrJsW03DlXucGxtoOvpz9zJWg9JlBPmS3cvNCMRR69h5lCYifhSdbsWkFvmETudUZqEb749+ePTwTVIjy4sJ4PJ3hf5VCbLF2pEuOrrCO2mC/luc8n93P95R607sRbIzAE9Z9701FD36ksH+RbZ9LJJAwATVYW1PZgTUDUFYLxcrOzSEo3m3VbNjq";
        //Set up the Camera to the current Vuforia Object
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Check Range and Convenient
        //Connect the parameters Mentioned above to the Current Vufuria
        this.vuforiaL= ClassFactory.createVuforiaLocalizer(parameters);
        //Set up the tracker to search for PictoGraph
        VuforiaTrackables relicTrackables=this.vuforiaL.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate= relicTrackables.get(0);
        //Degree Position
        OpenGLMatrix pose = null;
        //Set up the Search
        RelicRecoveryVuMark vuMark = null;

        FDS = RobotHardwareMap.get(ModernRoboticsI2cRangeSensor.class, "FDS");
    }

    //Autonomous
    //Mecanum
    //A function to move forward and backward
    public void autoMoveFB(double power)
    {
        LFD.setPower(power);
        LRD.setPower(power);
        RFD.setPower(power);
        RRD.setPower(power);
    }
    //A function to sidestep
    public void autoSideLR(double power)
    {
        LFD.setPower(power);
        LRD.setPower(-power);
        RFD.setPower(power);
        RRD.setPower(-power);
    }
    //A function to turn
    public void autoTiltLR(double power)
    {
        LFD.setPower(power);
        LRD.setPower(power);
        RFD.setPower(-power);
        RRD.setPower(-power);
    }

    //Camera

    public double PositionDegree()
    {
        int cameraMonitorViewId= RobotHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", RobotHardwareMap.appContext.getPackageName());
        //Set up the monitor to the current Vuforia Object
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Set up the License Key to the current Vuforia Object
        parameters.vuforiaLicenseKey = "AR+/1Xj/////AAAAmZVeX1TclkDftf0EjQhj+idSNF6vpGAY0vWg1laTAVg+MedtLFjicjlrOlqdduoKW33j/ITg4YIsQ3OvH8wXq1Wp/6ojRAa9HkHIlpheD/WnAK86Vm4QgTL2uOnksDLu6/zFiVledm6pgfLTsq0Wu3ySl2uc3tHAP8MA4QuENWN1qjWheRAJrJsW03DlXucGxtoOvpz9zJWg9JlBPmS3cvNCMRR69h5lCYifhSdbsWkFvmETudUZqEb749+ePTwTVIjy4sJ4PJ3hf5VCbLF2pEuOrrCO2mC/luc8n93P95R607sRbIzAE9Z9701FD36ksH+RbZ9LJJAwATVYW1PZgTUDUFYLxcrOzSEo3m3VbNjq";
        //Set up the Camera to the current Vuforia Object
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Check Range and Convenient
        //Connect the parameters Mentioned above to the Current Vufuria
        this.vuforiaL= ClassFactory.createVuforiaLocalizer(parameters);
        //Set up the tracker to search for PictoGraph
        VuforiaTrackables relicTrackables=this.vuforiaL.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate= relicTrackables.get(0);
        //Degree Position
        OpenGLMatrix pose=((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
        if (pose != null) {
            VectorF translation = pose.getTranslation();
            double degrees = Math.toDegrees(Math.atan2(translation.get(0), translation.get(1)));
            return degrees;
        }
        else {
            return 0;
        }
    }
    public String ObjectFinder()
    {
        int cameraMonitorViewId= RobotHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", RobotHardwareMap.appContext.getPackageName());
        //Set up the monitor to the current Vuforia Object
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Set up the License Key to the current Vuforia Object
        parameters.vuforiaLicenseKey = "AR+/1Xj/////AAAAmZVeX1TclkDftf0EjQhj+idSNF6vpGAY0vWg1laTAVg+MedtLFjicjlrOlqdduoKW33j/ITg4YIsQ3OvH8wXq1Wp/6ojRAa9HkHIlpheD/WnAK86Vm4QgTL2uOnksDLu6/zFiVledm6pgfLTsq0Wu3ySl2uc3tHAP8MA4QuENWN1qjWheRAJrJsW03DlXucGxtoOvpz9zJWg9JlBPmS3cvNCMRR69h5lCYifhSdbsWkFvmETudUZqEb749+ePTwTVIjy4sJ4PJ3hf5VCbLF2pEuOrrCO2mC/luc8n93P95R607sRbIzAE9Z9701FD36ksH+RbZ9LJJAwATVYW1PZgTUDUFYLxcrOzSEo3m3VbNjq";
        //Set up the Camera to the current Vuforia Object
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Check Range and Convenient
        //Connect the parameters Mentioned above to the Current Vufuria
        this.vuforiaL= ClassFactory.createVuforiaLocalizer(parameters);
        //Set up the tracker to search for PictoGraph
        VuforiaTrackables relicTrackables=this.vuforiaL.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate= relicTrackables.get(0);
        //Set up the Search
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark!=RelicRecoveryVuMark.UNKNOWN)
        {
            if (vuMark == RelicRecoveryVuMark.LEFT)
            {
                return "Left";
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER)
            {
                return "Center";
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT)
            {
                return "Right";
            }
        }
        return null;
    }
    public void MotorSet(DcMotor m, double Power){
        m.setPower(Power);
    }
    public void ServoSet(Servo s, double Position){
        s.setPosition(Position);
    }

    public double GetSmartDistance(){
        if(FDS.cmOptical() < 25){
            return FDS.cmOptical();
        }
        return FDS.cmUltrasonic();
    }

    public boolean CheckUnder5cm(){
        if (FDS.cmOptical() <= 5){
            return true;
        }
        return false;
    }
 }

