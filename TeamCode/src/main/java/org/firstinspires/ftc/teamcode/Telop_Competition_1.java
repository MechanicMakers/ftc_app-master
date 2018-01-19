package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Telop_Competition_1", group="TeleOp")

public class Telop_Competition_1 extends OpMode {
    //Genreal init
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMapRobot robot = new HardwareMapRobot();
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop()
    {
        robot.init(hardwareMap);
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        //Mecanum
        if (Math.abs(gamepad1.left_stick_y) > robot.thrshold)
        {
            if (gamepad1.left_stick_y < 0){
                robot.y1 = -1*(gamepad1.left_stick_y * gamepad1.left_stick_y);
            }
            else if (gamepad1.left_stick_y > 0){
                robot.y1 = (gamepad1.left_stick_y * gamepad1.left_stick_y);
            }
        }
        else {
            robot.y1 = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) > robot.thrshold)
        {
            if (gamepad1.left_stick_x < 0){
                robot.x1 = -1*(gamepad1.left_stick_x * gamepad1.left_stick_x);
            }
            else if (gamepad1.left_stick_x > 0){
                robot.x1 = (gamepad1.left_stick_x * gamepad1.left_stick_x);
            }
        }
        else {
            robot.x1 = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) > robot.thrshold)
        {
            if (gamepad1.right_stick_x < 0){
                robot.x2 = -1*(gamepad1.right_stick_x * gamepad1.right_stick_x);
            }
            else if (gamepad1.right_stick_x > 0){
                robot.x2 = (gamepad1.right_stick_x * gamepad1.right_stick_x);
            }
        }
        else {
            robot.x2 = 0;
        }

        // our Mecanum formula
        robot.RFD.setPower(robot.y1 - robot.x1 + robot.x2); // = -1
        robot.RRD.setPower(robot.y1 + robot.x1 + robot.x2); // = -1
        robot.LFD.setPower(robot.y1 + robot.x1 - robot.x2); // = -1
        robot.LRD.setPower(robot.y1 - robot.x1 - robot.x2 ); // = 1

        // slow drive
        if (gamepad1.dpad_down)
        {
            robot.autoMoveFB(0.2);
        }
        if (gamepad1.dpad_up)
        {
            robot.autoMoveFB( -0.2);
        }
        if (gamepad1.dpad_left)
        {
        robot.autoTiltLR(0.75);
        }
        if (gamepad1.dpad_right)
        {
            robot.autoSideLR(- 0.75);
        }

        //Lifter
        // set the Position of the servo of the lift.
        if (gamepad1.right_trigger > 0.1)
        {
            robot.LL.setPosition(1.0);
            robot.RL.setPosition(1.0);
        }
        else if (gamepad1.left_trigger > 0.1 )
        {
            robot.LL.setPosition(0.0);
            robot.RL.setPosition(0.0);
        }

        //collector
        //set the power of the motors of the collector
        if (gamepad1.left_bumper || gamepad2.left_bumper)
        {
            robot.CL.setPower(1.0);
            robot.CR.setPower(1.0);
        }
        else if (gamepad1.right_bumper || gamepad2.right_bumper)
        {
            robot.CL.setPower(-1.0);
            robot.CR.setPower(-1.0);
        }
        else
        {
            robot.CL.setPower(0.0);
            robot.CR.setPower(0.0);
        }
        //pizza
        //set the Position of the pizza's servo
        if (gamepad1.x)
        {
            robot.LP.setPosition(1.0);
            robot.RP.setPosition(1.0);
        }
        else
        {
            robot.LP.setPosition(0.0);
            robot.RP.setPosition(0.0);
        }
        // elevator
        //set the power of the elevator's motors
        if (gamepad1.a || gamepad2.right_trigger > 0.1)
        {
            robot.EL.setPower(1.0);
            robot.ER.setPower(1.0);
        }
        else if (gamepad1.b || gamepad2.left_trigger > (0.1))
        {
            robot.EL.setPower(-1.0);
            robot.ER.setPower(-1.0);
        }
        else
        {
            robot.EL.setPower(0.0);
            robot.ER.setPower(0.0);
        }
        //grobber
        //the Position of the closing and opening of the grobber
        if (gamepad2.dpad_up)
        {
            robot.RG.setPosition(1.0);
        }
        else if (gamepad2.dpad_down)
        {
            robot.RG.setPosition(0.0);
        }
        //the Position of the twisting grobber
        if (gamepad2.dpad_right)
        {
            robot.RT.setPosition(robot.RT.getPosition()+0.2);
        }
        else if (gamepad2.dpad_left)
        {
            robot.RT.setPosition(robot.RT.getPosition()-0.2);
        }
        //opens to Position A in order to take the relick
        if (gamepad2.a)
        {
            robot.RAA.setPosition(1.0);
            robot.RAB.setPosition(0.0);
            robot.RAC.setPosition(1.0);
        }
        //opens the arm for its full size
         else if (gamepad2.b)
        {
            robot.RAA.setPosition(1.0);
            robot.RAB.setPosition(1.0);
            robot.RAC.setPosition(1.0);
        }
        //closing the arm
         else if (gamepad2.y)
        {
            robot.RAA.setPosition(0.0);
            robot.RAB.setPosition(0.0);
            robot.RAC.setPosition(0.0);
        }
        //twisting the whole arm
        if (gamepad2.left_stick_x < 0){
            robot.ServoSet(robot.RAT, robot.RAT.getPosition()-0.01);
        }
        else if (gamepad2.left_stick_x > 0){
            robot.ServoSet(robot.RAT, robot.RAT.getPosition()+0.01);
        }
        //controlling the all arm up and down
        if (-gamepad2.left_stick_y < 0){
            robot.ServoSet(robot.RAA, robot.RAA.getPosition()-0.01);
        }
        else if (-gamepad2.left_stick_y > 0){
            robot.ServoSet(robot.RAA, robot.RAA.getPosition()+0.01);
        }

    }
    @Override

    public void stop() {
        //reset all the robot
        // pizza stop
        robot.LP.setPosition(1);
        robot.RP.setPosition(1);
        //jewlin stop
        robot.LJ.setPosition(0.2);
        robot.RJ.setPosition(0.2);
        //lifter stop
        robot.RL.setPosition(0);
        robot.LL.setPosition(0);
        //Relic
        robot.RT.setPosition(0);
        robot.RG.setPosition(0);
        robot.RAT.setPosition(0);
        robot.RAA.setPosition(0);
        robot.RAB.setPosition(0);
        robot.RAC.setPosition(0);


    }
}
