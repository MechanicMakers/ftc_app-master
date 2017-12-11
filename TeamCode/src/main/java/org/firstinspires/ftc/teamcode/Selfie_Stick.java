package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@TeleOp(name="Selfie_stick", group="TeleOp")
//@Disabled
public class Selfie_Stick extends OpMode
{
    //counting time function
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime autotime = new ElapsedTime();

    //Selfie_Stick
    // We declare 4 servo objects for the Selfie_Stick
    private Servo AT = null;
    private Servo AA = null;
    private Servo AB = null;
    private Servo AC = null;
    private boolean toggle = true;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //Selfie_Stick
        // Initialize Selfie_Stick hardware, direction and scale range.
        AT  = hardwareMap.get(Servo.class, "AT");
        AA = hardwareMap.get(Servo.class, "AA");
        AB = hardwareMap.get(Servo.class, "AB");
        AC = hardwareMap.get(Servo.class, "AC");


        AT.setDirection(Servo.Direction.FORWARD);
        AA.setDirection(Servo.Direction.FORWARD);
        AB.setDirection(Servo.Direction.REVERSE);
        AC.setDirection(Servo.Direction.FORWARD);

        AT.scaleRange(-1, 1);
        AA.scaleRange(0.0, 1);
        AB.scaleRange(0.0, 1);
        AC.scaleRange(0.0, 1);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        //Selfie_Stick
        //Initialize the position of the servos
        AT.setPosition(0);
        AA.setPosition(0);
        AB.setPosition(0);
        AC.setPosition(0);
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
        //Selfie_Stick
        //config the buttons.

        //control the arm twist servo by left stick x line
        if (gamepad2.left_stick_x != 0)
        {
            if (-gamepad2.left_stick_x > 0)
            {
                AT.setPosition(AT.getPosition()+ 0.0025);
            }
            if (-gamepad2.left_stick_x < 0)
            {
                AT.setPosition(AT.getPosition()- 0.0025);
            }
        }

        //checking if non of the button a, b and y pressed and checing if the toggle true or false and controling the servos by that
        if (!gamepad2.a && !gamepad2.b && !gamepad2.y)
        {
            if (toggle)
            {
                if (-gamepad2.right_stick_y > 0)
                {
                    AA.setPosition(AA.getPosition()+ 0.0025);
                }
                if (-gamepad2.right_stick_y < 0)
                {
                    AA.setPosition(AA.getPosition()- 0.0025);
                }
            }
            else if (!toggle)
            {
                if (-gamepad2.right_stick_y > 0)
                {
                    AB.setPosition(AB.getPosition()+ 0.0025);
                }
                if (-gamepad2.right_stick_y < 0)
                {
                    AB.setPosition(AB.getPosition()- 0.0025);
                }
            }
        }

        //A opens Arm A and Arm C in order to use the grabber for relic
        if (gamepad2.a)
        {
            autotime.reset();
            while (autotime.time() < 1)
            AA.setPosition(0.5);
            AC.setPosition(1.0);
        }

        //toggling the toggle variable and open the arm slowly
        if (gamepad2.b)
        {
            toggle = !toggle;

            autotime.reset();
            while (autotime.time() < 1) {
                AA.setPosition(1.0);
            }
            autotime.reset();
            while (autotime.time() < 1) {
                AB.setPosition(1.0);
                AC.setPosition(1.0);
            }

        }

        //reset the arm to the start point(closed)
        if (gamepad2.y)
        {
            autotime.reset();
            while (autotime.time() < 1) {
                AB.setPosition(0.0);
                AC.setPosition(0.0);
            }
            autotime.reset();
            while (autotime.time() < 1) {
                AA.setPosition(0.0);
            }
        }
        telemetry.addData("Selfie_Stick", "Arm Twist (%.2f), Arm A (%.2f)", AT.getPosition(),AA.getPosition());
        telemetry.addData("Selfie_Stick", "Arm B (%.2f), Arm C (%.2f)", AB.getPosition(), AC.getPosition());


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
