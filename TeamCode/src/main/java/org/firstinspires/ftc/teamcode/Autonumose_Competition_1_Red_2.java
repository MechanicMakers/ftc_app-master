package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="Autonumose_Competition_1_Red_2", group="Autonomous")

public class Autonumose_Competition_1_Red_2 extends OpMode {
    HardwareMapRobot robot = new HardwareMapRobot();
    private ElapsedTime time = new ElapsedTime();
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        //Jewlin
        robot.RJ.setPosition(0.2);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //Stage 0 - checking the picture
        while(time.milliseconds() >= 500){
            robot.pos = robot.ObjectFinder();
        }
        time.reset();

        // Stage 1 - Jewlin goes down
        while(time.milliseconds() >= 1500){
            robot.LJ.setPosition(0.83);
        }
        time.reset();

        // Stage 2 - Checking what is the color in front of us
        // if the the front is blue
        if (robot.RJCS.red() > robot.RJCS.blue())
        {

            //Stage 3 - Tilt to the left and Move the jewel
            while(time.milliseconds() >= 1000) {
                robot.autoTiltLR(-0.25);
            }
            time.reset();

            //Stage 4 - Tilt to the left and Move the jewel
            while(time.milliseconds() >= 1000) {
                robot.autoTiltLR(0.25);
            }
            time.reset();

            //Stage 5 - Return the Jewelin to his original position
            while(time.milliseconds() >= 1000) {
                robot.RJ.setPosition(0.2);
            }
            time.reset();
        }

        // Stage 2 - Checking what is the color in front of us
        // if the the front is blue
        else if(robot.LJCS.blue() > robot.LJCS.red()) {

            //Stage 3 - Tilt to the left and Move the jewel
            while (time.milliseconds() >= 1000) {
                robot.autoTiltLR(0.25);
            }
            time.reset();

            //Stage 4 - Tilt to the left and Move the jewel
            while(time.milliseconds() >= 1000) {
                robot.autoTiltLR(-0.25);
            }
            time.reset();

            //Stage 5 - Return the Jewelin to his original position
            while(time.milliseconds() >= 1000) {
                robot.LJ.setPosition(0.2);
            }
            time.reset();
        }

        // Stage 6 - Drive to the safe zone until the robot reach the blue line
        while(!(robot.BC.red() > robot.BC.blue() && robot.BC.red() > robot.BC.green())){
            robot.autoMoveFB(0.25);
        }
        //Stage 6.1
        while(time.milliseconds()>=500)
        {
            robot.autoMoveFB(-0.5);
        }
        time.reset();
        while(time.milliseconds()>=350)
        {
            robot.autoSideLR(0.5);
        }
        time.reset();
        if (robot.pos == "Center")
        {
            while (time.milliseconds() >= 1){
                robot.autoMoveFB(1);
            }
            robot.autoMoveFB(0);
        }
        if (robot.pos == "Left")
        {
            while(time.milliseconds()>= 200){
                robot.autoSideLR(-0.5);
            }
            time.reset();
            while (time.milliseconds() >= 1000){
                robot.autoMoveFB(1);
            }
            robot.autoMoveFB(0);
        }
        if (robot.pos == "Right")
        {
            while(time.milliseconds()>= 200){
                robot.autoSideLR(0.5);
            }
            time.reset();
            while (time.milliseconds() >= 1000){
                robot.autoMoveFB(1);
            }
            robot.autoMoveFB(0);
        }
        robot.autoMoveFB(0);
        time.reset();
        while(time.milliseconds() >= 1000){ //8 - Drive foreword
            robot.autoMoveFB(1);
        }
        robot.autoMoveFB(0);
        time.reset();

        robot.LP.setPosition(1); //9 - Release the glif
        robot.RP.setPosition(1);

        while(time.milliseconds() >= 1000)
        {
            robot.autoMoveFB(-0.25); //10 - Drive backwards
        }
        while(time.milliseconds() >= 1000)
        {
            robot.autoMoveFB(0.3); //11 - Push the glif forewords to make sure the glif is in place
        }

        time.reset();
    }
    @Override
    public void loop() {
    }
    @Override
    public void stop() {
    }
}
