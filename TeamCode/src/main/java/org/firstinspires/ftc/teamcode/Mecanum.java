package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum", group="TeleOp")
//@Disabled
public class Mecanum extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LFD = null;
    private DcMotor LRD = null;
    private DcMotor RFD = null;
    private DcMotor RRD = null;
    private double thrshold = 0.1;
    private double x1;
    private double x2;
    private double y1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        LFD = hardwareMap.get(DcMotor.class, "LFD");
        LRD = hardwareMap.get(DcMotor.class, "LRD");
        RFD = hardwareMap.get(DcMotor.class, "RFD");
        RRD = hardwareMap.get(DcMotor.class, "RRD");

        LFD.setDirection(DcMotor.Direction.FORWARD);
        LRD.setDirection(DcMotor.Direction.FORWARD);
        RFD.setDirection(DcMotor.Direction.REVERSE);
        RRD.setDirection(DcMotor.Direction.REVERSE);


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
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        if (Math.abs(gamepad1.left_stick_y) > thrshold) {
            y1 = gamepad1.left_stick_y * gamepad1.left_stick_y;
        } else {
            y1 = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) > thrshold) {
            x1 = gamepad1.left_stick_x * gamepad1.left_stick_x;
        } else {
            x1 = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) > thrshold) {
            x2 = gamepad1.right_stick_x * gamepad1.right_stick_x;
        } else {
            x2 = 0;
        }
        RFD.setPower(y1 - x2 - x1);
        RRD.setPower(y1 - x2 + x1);
        LFD.setPower(y1 + x2 + x1);
        LRD.setPower(y1 + x2 - x1);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors right side", "Forward" + RFD.getPower(), "Rear" + RRD.getPower());
        telemetry.addData("Motors left side", "Forward" + LFD.getPower(), "Rear" + LRD.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}


