package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Relic_Grobber", group="TeleOp")
//@Disabled
public class Relic_Grobber extends OpMode
{
    //counting time function
    private ElapsedTime runtime = new ElapsedTime();
    //grobber
    // We declare 2 servo objects for the servos
    private Servo T = null;
    private Servo G = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //grobber
        // Initialize grobber hardware, direction and scale range.
        T  = hardwareMap.get(Servo.class, "T");
        G = hardwareMap.get(Servo.class, "G");

        T.setDirection(Servo.Direction.FORWARD);
        G.setDirection(Servo.Direction.FORWARD);

        T.scaleRange(0.0, 1);
        G.scaleRange(0.0, 1);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        //grobber
        //Initialize the position of the servos
        T.setPosition(0);
        G.setPosition(0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Grobber
        //config the buttons
        if (gamepad2.right_bumper)
        {
            G.setPosition(1);
        }
        else if (gamepad2.left_bumper)
        {
            G.setPosition(0);
        }
        if (gamepad2.dpad_right){
            T.setPosition(T.getPosition() - 0.01);
        }
        if (gamepad2.dpad_left){
            T.setPosition(T.getPosition() + 0.01);
        }
        telemetry.addData("Relic_Grobber", "left (%.2f), right (%.2f)", T.getPosition(), G.getPosition());


        //ADD ROBOT TIME STATUS
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
