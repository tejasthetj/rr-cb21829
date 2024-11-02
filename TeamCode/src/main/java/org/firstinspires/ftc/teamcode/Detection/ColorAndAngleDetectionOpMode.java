package org.firstinspires.ftc.teamcode.Detection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.SampleDetectionPipelinePNP;

@Autonomous(name = "Color and Angle Detection OpMode")
public class ColorAndAngleDetectionOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    SampleDetectionPipelinePNP pipeline;

    @Override
    public void runOpMode() {
        // Get the webcam ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up the pipeline
        pipeline = new SampleDetectionPipelinePNP();
        webcam.setPipeline(pipeline);

        // Open the camera and start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Get the list of detected stones with color and angle
            ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> detectedStones = pipeline.getDetectedStones();

            // Display each stone's color and angle in telemetry
            if (!detectedStones.isEmpty()) {
                for (int i = 0; i < detectedStones.size(); i++) {
                    SampleDetectionPipelinePNP.AnalyzedStone stone = detectedStones.get(i);
                    telemetry.addData("Stone " + (i + 1), "Color: %s, Angle: %.2f", SampleDetectionPipelinePNP.AnalyzedStone.color, stone.angle);
                }
            } else {
                telemetry.addData("No stones detected", "Please adjust object position.");
            }
            telemetry.update();

            // Pause for a short time to avoid flooding telemetry
            sleep(100);
        }

        // Close the camera when the OpMode ends
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
