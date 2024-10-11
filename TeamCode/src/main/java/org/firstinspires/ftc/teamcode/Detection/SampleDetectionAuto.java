package org.firstinspires.ftc.teamcode.Detection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Detection.SampleDetectionPipelinePNP;

import java.util.ArrayList;

@Autonomous(name = "SampleDetectionAuto", group = "FTC")
public class SampleDetectionAuto extends LinearOpMode {

    private OpenCvCamera camera;
    private SampleDetectionPipelinePNP pipeline;

    // Robot hardware (example)
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, "left front");
        frontRightMotor = hardwareMap.get(DcMotor.class, "right front");
        backLeftMotor = hardwareMap.get(DcMotor.class, "left rear");
        backRightMotor = hardwareMap.get(DcMotor.class, "right rear");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        pipeline = new SampleDetectionPipelinePNP();
        camera.setPipeline(pipeline);

        // Start streaming from the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Continuously get data from the pipeline while the OpMode is running
            while (opModeIsActive()) {
                ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> detectedStones = pipeline.getDetectedStones();

                if (!detectedStones.isEmpty()) {
                    for (SampleDetectionPipelinePNP.AnalyzedStone stone : detectedStones) {
                        telemetry.addData("Detected", SampleDetectionPipelinePNP.AnalyzedStone.color);
                        telemetry.addData("Angle", SampleDetectionPipelinePNP.AnalyzedStone.angle);
                        telemetry.addData("Translation", "X: " + SampleDetectionPipelinePNP.AnalyzedStone.tvec.get(0, 0)[0] + " Y: " + SampleDetectionPipelinePNP.AnalyzedStone.tvec.get(1, 0)[0] + " Z: " + stone.tvec.get(2, 0)[0]);
                        telemetry.addData("Rotation", "Rvec: " + stone.rvec.dump());

                        // Example: Move robot based on detected stone's position (tvec)
//                        double distanceToStone = stone.tvec.get(2, 0)[0]; // Z value indicates forward distance
//                        if (distanceToStone > 20) {
//                            moveForward(0.3);  // Move robot forward if the stone is far away
//                        } else {
//                            stopMotors(); // Stop the robot when it's close enough
//                        }
                    }
                } else {
                    telemetry.addLine("No stones detected.");
                }

                telemetry.update();
            }
        }

        // Stop the camera streaming when OpMode ends
        camera.stopStreaming();
    }

    private void moveForward(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
