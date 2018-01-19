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

    public String pos;
    public void runOpMode(){

        int cameraMonitorViewId=hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AR+/1Xj/////AAAAmZVeX1TclkDftf0EjQhj+idSNF6vpGAY0vWg1laTAVg+MedtLFjicjlrOlqdduoKW33j/ITg4YIsQ3OvH8wXq1Wp/6ojRAa9HkHIlpheD/WnAK86Vm4QgTL2uOnksDLu6/zFiVledm6pgfLTsq0Wu3ySl2uc3tHAP8MA4QuENWN1qjWheRAJrJsW03DlXucGxtoOvpz9zJWg9JlBPmS3cvNCMRR69h5lCYifhSdbsWkFvmETudUZqEb749+ePTwTVIjy4sJ4PJ3hf5VCbLF2pEuOrrCO2mC/luc8n93P95R607sRbIzAE9Z9701FD36ksH+RbZ9LJJAwATVYW1PZgTUDUFYLxcrOzSEo3m3VbNjq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //Check Range and Convenient
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
                pos = vuMark.toString();
            else
                telemetry.addData("VuMark", "not visible");

            telemetry.update();
        }

    }
}
