package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


@Autonomous(name="Jewlin", group="Autonomous")
//@Disabled
public class Jewlin extends OpMode
{
    //counting time function
    private ElapsedTime runtime = new ElapsedTime();
    //KRABBER
    // We declare 2 servo objects and 2 variable for left krabb and right krabb and his scale Range
    private Servo LJ = null;
    DistanceSensor JDS;
    ColorSensor JCS;
    private double minK = 0.2;
    private double maxK = 0.5;
    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //KRABBER
        // Initialize krabber hardware, direction and scale range.
        LJ  = hardwareMap.get(Servo.class, "LK");
        JCS = hardwareMap.get(ColorSensor.class, "JCS");
        JDS = hardwareMap.get(DistanceSensor.class, "JCS");
        LJ.setDirection(Servo.Direction.FORWARD);



        LJ.scaleRange(0.2, 0.5);

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
        LJ.setPosition(1);
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
        Color.RGBToHSV((int) (JCS.red() * SCALE_FACTOR),
                (int) (JCS.green() * SCALE_FACTOR),
                (int) (JCS.blue() * SCALE_FACTOR),
                hsvValues);
        telemetry.addData("ColorSensorValues","Red " + JCS.red());
        telemetry.addData("ColorSensorValues","Blue " + JCS.blue());
        telemetry.addData("ColorSensorValues","Alpha " + JCS.alpha());

        //ADD ROBOT TIME STATUS
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
