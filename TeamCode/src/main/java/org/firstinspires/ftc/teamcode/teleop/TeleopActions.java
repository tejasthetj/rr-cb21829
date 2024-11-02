package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Master;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop actions testing", group = "Exercise")
public class TeleopActions extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    public DcMotor Elevatorright, Elevatorleft, Horizontalright, Horizontalleft;

    IMU imu;

    public Servo leftClaw, rightClaw, wrist, axle, outAxle, outWrist, outRightClaw, outLeftClaw;

    Master robot = new Master();

    // Define actions for A and B buttons
    private SequentialAction Intake;
    private SequentialAction Outtake;
    private SequentialAction Reset;

    @Override
    public void init() {
        motorAndServoInit();  // Initializes motors and servos

        // Pre-build the actions for button A
        Intake = new SequentialAction(
                new InstantAction(() -> setServoPosition(wrist, WRIST_DOWN)),
                new InstantAction(() -> setServoPosition(axle, PIVOT_DOWN)),
                new SleepAction(0.5),
                new ParallelAction(
                        new InstantAction(() -> setServoPosition(leftClaw, LEFT_CLAW_CLOSE)),
                        new InstantAction(() -> setServoPosition(rightClaw, RIGHT_CLAW_CLOSE))
                ),
                new SleepAction(1.0),
                new ParallelAction(
                        new InstantAction(() -> setServoPosition(wrist, WRIST_UP)),
                        new InstantAction(() -> setServoPosition(axle, PIVOT_UP)),
                        new InstantAction(() -> setServoPosition(leftClaw, LEFT_CLAW_READJUST)),
                        new InstantAction(() -> setServoPosition(rightClaw, RIGHT_CLAW_READJUST))
                )
        );

        // Pre-build the actions for button X
        Outtake = new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> setServoPosition(outLeftClaw, OUT_LEFT_CLAW_OPEN)),
                        new InstantAction(() -> setServoPosition(outRightClaw, OUT_RIGHT_CLAW_OPEN))
                ),
                new SleepAction(0.5),
                new ParallelAction(
                        new InstantAction(() -> setServoPosition(leftClaw, RIGHT_CLAW_OPEN)),
                        new InstantAction(() -> setServoPosition(rightClaw, LEFT_CLAW_OPEN)),
                        new InstantAction(() -> setServoPosition(wrist, WRIST_READJUST)),
                        new InstantAction(() -> setServoPosition(axle, PIVOT_READJUST))
                ),
                new ParallelAction(
                        new InstantAction(() -> setServoPosition(outWrist, OUT_WRIST_DOWN)),
                        new InstantAction(() -> setServoPosition(outAxle, OUT_PIVOT_UP))
                )
        );

        Reset = new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> setServoPosition(leftClaw, RIGHT_CLAW_OPEN)),
                        new InstantAction(() -> setServoPosition(rightClaw, LEFT_CLAW_OPEN)),
                        new InstantAction(() -> setServoPosition(wrist, WRIST_READJUST)),
                        new InstantAction(() -> setServoPosition(axle, PIVOT_READJUST))
                ),
                new SleepAction(0.3),
                new ParallelAction(
                        new InstantAction(() -> setServoPosition(outLeftClaw, OUT_LEFT_CLAW_CLOSE)),
                        new InstantAction(() -> setServoPosition(outRightClaw, OUT_RIGHT_CLAW_CLOSE)),
                        new InstantAction(() -> setServoPosition(outWrist, OUT_WRIST_UP)),
                        new InstantAction(() -> setServoPosition(outAxle, OUT_PIVOT_DOWN))
                )
        );
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // Trigger actionA sequence if "A" is pressed
        if (gamepad1.a) {
            runningActions.clear(); // Clear any running actions
            runningActions.add(Intake);
        }
        // Trigger actionB sequence if "B" is pressed
        if (gamepad1.b) {
            runningActions.clear(); // Clear any running actions
            runningActions.add(Reset);
        }  if (gamepad1.x) {
            runningActions.clear(); // Clear any running actions
            runningActions.add(Outtake);
        }



        // Execute and filter out completed actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay()); // Update telemetry
            if (action.run(packet)) {
                newActions.add(action); // Keep action in list if not yet complete
            }
        }

        runningActions = newActions; // Update runningActions list
        dash.sendTelemetryPacket(packet); // Send telemetry packet to dashboard
    }

    public void motorAndServoInit() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "left front");
        frontRight = hardwareMap.get(DcMotor.class, "right front");
        rearLeft = hardwareMap.get(DcMotor.class, "left rear");
        rearRight = hardwareMap.get(DcMotor.class, "right rear");

        // Initialize control motors
        Elevatorright = hardwareMap.get(DcMotor.class, "vertical 1");
        Elevatorleft = hardwareMap.get(DcMotor.class, "vertical 2");
        Horizontalright = hardwareMap.get(DcMotor.class, "horizontal 1");
        Horizontalleft = hardwareMap.get(DcMotor.class, "horizontal 2");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        Horizontalleft.setDirection(DcMotor.Direction.REVERSE);
        Elevatorright.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos
        leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        axle = hardwareMap.get(Servo.class, "pivot servo");
        outAxle = hardwareMap.get(Servo.class, "outtake servo");
        outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        outWrist = hardwareMap.get(Servo.class, "out wrist servo");
    }

    // Helper function to set servo position safely
    private void setServoPosition(Servo servo, double position) {
        if (servo != null) {
            servo.setPosition(position);
        } else {
            telemetry.addData("Error", "Attempted to set position on a null servo");
            telemetry.update();
        }
    }
}
