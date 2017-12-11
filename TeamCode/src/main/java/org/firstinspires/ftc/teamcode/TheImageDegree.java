package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by ${alon} on 21/11/2017.
 */
@Autonomous(name = "TheImageDegree")
public class TheImageDegree extends LinearOpMode {

    VuforiaLocalizer vuforiaL;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId=hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters=new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey="AVmiHF3/////AAAAGTLb6D5vqUgdt1adb6Mf4sYb0tD0exNoGb4Af3FwgeOibcDTUsLJKtf9NcniZiHzmpuCChQnyFA1I/qwBlAnuy6jnujS0l0oVf8vRW5VMnMnnIRyisoKU8YuVDfI2aH5/zNibuTrUikvDuVGWH4DtnU1ZF+T/aG2p6lm6RWJCN1H0PVoMjkE2gPeqL6ShE6SvqNtYlByfFwlL613ptwTStn+VVAjCBkqiUtdAz0N3cbLR7aw7yAerGobh6X0vxvlADE9iDZLl+pJ725mEFJZHryiD3JF38AOESrY+5Fv49Wns+/VnXFqmZieQf7OnXjQX6Phc6Y0qkAHEYh2rz2Or7n6Ny19XCUdVWvfQTeOKkev";
        parameters.cameraDirection=VuforiaLocalizer.CameraDirection.BACK;
        this.vuforiaL= ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables=this.vuforiaL.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate= relicTrackables.get(0);

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {
            OpenGLMatrix pose=((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            if (pose != null) {
                VectorF translation = pose.getTranslation();
                double degrees = Math.toDegrees(Math.atan2(translation.get(0), translation.get(1)));
                telemetry.addData("degrees: ", degrees);
            }
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark!=RelicRecoveryVuMark.UNKNOWN)
                telemetry.addData("VuMark", "%s visible", vuMark);
            else
                telemetry.addData("VuMark", "not visible");
            telemetry.update();
        }

    }
}
