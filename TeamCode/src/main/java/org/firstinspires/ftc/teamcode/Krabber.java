package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Krabber", group="TeleOp")
//@Disabled
public class Krabber extends OpMode
{
    //counting time function
    private ElapsedTime runtime = new ElapsedTime();
    //KRABBER
    // We declare 2 servo objects and 2 variable for left krabb and right krabb and his scale Range
    private Servo LK = null;
    private Servo RK = null;
    private double minK = 0.2;
    private double maxK = 0.5;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //KRABBER
        // Initialize krabber hardware, direction and scale range.
        LK  = hardwareMap.get(Servo.class, "LK");
        RK = hardwareMap.get(Servo.class, "RK");

        LK.setDirection(Servo.Direction.FORWARD);
        RK.setDirection(Servo.Direction.REVERSE);

        LK.scaleRange(0.2, 0.5);
        RK.scaleRange(0.2, 0.5);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        //KRABER
        //Initialize the first position of krabber with GLYPH.
        LK.setPosition(1);
        RK.setPosition(1);
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
        //KRABBER
        //Drive control right and left servos of krabber.
        if (gamepad1.right_bumper)
        {
            LK.setPosition(1);
            RK.setPosition(1);
        }
        else if (gamepad1.left_bumper)
        {
            LK.setPosition(0);
            RK.setPosition(0);
        }
        telemetry.addData("KRABBER", "left (%.2f), right (%.2f)", LK.getPosition(), RK.getPosition());


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
