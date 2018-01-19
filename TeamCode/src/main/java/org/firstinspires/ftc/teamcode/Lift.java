package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Lift", group="TeleOp")
//@Disabled
public class Lift extends OpMode
{
    //counting time function
    private ElapsedTime runtime = new ElapsedTime();
    //Lift
    // We declare 2 motors objects for the lift
    private DcMotor LL = null;
    private DcMotor LR = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //Lift
        // Initialize lift hardware direction.
        LL = hardwareMap.get(DcMotor.class, "LL");
        LR = hardwareMap.get(DcMotor.class, "LR");

        LL.setDirection(DcMotor.Direction.FORWARD);
        LR.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
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
        //Lift
        //set the power of the motors of the lift.
        if (gamepad1.dpad_up)
        {
            LL.setPower(1.0);
            LR.setPower(1.0);
        }
        else if (gamepad1.dpad_down)
        {
            LL.setPower(-1.0);
            LR.setPower(-1.0);
        }
        else
        {
            LL.setPower(0.0);
            LR.setPower(0.0);;
        }
        telemetry.addData("Lift", "left (%.2f), right (%.2f)", LL.getPower(), LR.getPower() );


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
