package org.firstinspires.ftc.teamcode.Detection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.SampleDetectionPipelinePNP; // Adjust import as necessary
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

@TeleOp(name = "Omni Drive to Red Stone", group = "Concept")
public class RobotAutoDriveToRedStoneOmni extends LinearOpMode {
    final double DESIRED_DISTANCE = 12.0; // Target distance from the stone in inches
    final double SPEED_GAIN = 0.02; // Forward Speed Control "Gain"
    final double STRAFE_GAIN = 0.015; // Strafe Speed Control "Gain"
    final double TURN_GAIN = 0.01; // Turn Control "Gain"

    final double MAX_AUTO_SPEED = 0.5; // Max approach speed
    final double MAX_AUTO_TURN = 0.3; // Max turn speed

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private VisionPortal visionPortal;
    private SampleDetectionPipelinePNP pipeline; // Your detection pipeline

    @Override
    public void runOpMode() {
        boolean targetFound = false; // Set to true when a red stone target is detected
        double drive = 0; // Desired forward power/speed (-1 to +1)
        double turn = 0; // Desired turning power/speed (-1 to +1)

        // Initialize the pipeline
        pipeline = new SampleDetectionPipelinePNP(); // Instantiate your pipeline here
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .build();

        // Initialize the hardware variables
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftback_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;

            // Get detected stones from the pipeline
            ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> stones = pipeline.getDetectedStones();

            // Check for a red stone
            for (SampleDetectionPipelinePNP.AnalyzedStone stone : stones) {
                if ("Red".equals(SampleDetectionPipelinePNP.AnalyzedStone.color)) {
                    targetFound = true;
                    double stoneX = SampleDetectionPipelinePNP.AnalyzedStone.angle; // Assuming angle determines horizontal position
                    double stoneDistance = SampleDetectionPipelinePNP.AnalyzedStone.tvec.get(0, 0)[0]; // Assuming tvec contains the distance info

                    // Telemetry updates
                    telemetry.addData("Found", "Red Stone at angle: %f, Distance: %f", stoneX, stoneDistance);

                    // Calculate errors to control the robot
                    double rangeError = (stoneDistance - DESIRED_DISTANCE);
                    double headingError = stoneX; // Assuming the angle determines turning

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                }
            }

            if (!targetFound) {
                telemetry.addData("Status", "Red Stone not detected");
            }

            telemetry.update();

            // Apply desired axes motions to the drivetrain
            moveRobot(drive, turn);
            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     */
    public void moveRobot(double drive, double turn) {
        // Calculate wheel powers
        double leftFrontPower = drive - turn;
        double rightFrontPower = drive + turn;
        double leftBackPower = drive + turn;
        double rightBackPower = drive - turn;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
