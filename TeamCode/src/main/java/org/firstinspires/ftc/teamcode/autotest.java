package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.BlankDetector;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@Autonomous(name = "camera baby")
public class autotest extends LinearOpMode {
    OpenCvCamera camera;
    SkystoneDetector detector;



    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        camera.openCameraDevice();

        detector = new SkystoneDetector();
        detector.useDefaults();
        camera.setPipeline(detector);

        System.out.println("hello");


        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


        waitForStart();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Stone Position X", detector.getScreenPosition().x);
            telemetry.addData("Stone Position Y", detector.getScreenPosition().y);
            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", camera.getFps()));
            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
            telemetry.update();
        }


    }
}
