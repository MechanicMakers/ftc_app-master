package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;



@Autonomous(name="JewlinRed", group="Autonomous")

public class Jewlin extends OpMode
{
    //counting time function
    private ElapsedTime runtime = new ElapsedTime();

    // We declare servo object and his scale Range
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



        LJ.scaleRange(0, 1);

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
        LJ.setPosition(0.2);
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

        //ADD ROBOT TIME STATUS
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        if(JCS.blue() >= JCS.red() +50){
            LJ.setPosition(0.1);
        }
        else if(JCS.red() >= JCS.blue()+50){
            LJ.setPosition(0.3);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
