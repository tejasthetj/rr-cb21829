package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_UP;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Master;

@TeleOp(name = "Servo Testing", group = "Exercises")
public class ServoTesting extends LinearOpMode {

    Master robot = new Master();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        robot.rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        robot.wrist = hardwareMap.get(Servo.class, "wrist servo");
        robot.axle = hardwareMap.get(Servo.class, "pivot servo");

        robot.outAxle = hardwareMap.get(Servo.class, "outtake servo");
        robot.outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        robot.outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        robot.outWrist = hardwareMap.get(Servo.class, "out wrist servo");

        waitForStart();

        robot.axle.setDirection(Servo.Direction.REVERSE);
//        robot.outAxle.setDirection(Servo.Direction.REVERSE);

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                robot.wrist.setPosition(WRIST_UP);
                robot.axle.setPosition(PIVOT_UP);
            } else if (gamepad1.dpad_down) {
                robot.wrist.setPosition(WRIST_DOWN);
                robot.axle.setPosition(PIVOT_DOWN);
            } else if (gamepad1.right_bumper) {
                robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                robot.leftClaw.setPosition(LEFT_CLAW_CLOSE);
            } else if (gamepad1.left_bumper) {
                robot.rightClaw.setPosition(RIGHT_CLAW_OPEN);
                robot.leftClaw.setPosition(LEFT_CLAW_OPEN);
            } else if (gamepad1.dpad_left) {
                robot.outRightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE);
                robot.outLeftClaw.setPosition(OUT_LEFT_CLAW_CLOSE);
            } else if (gamepad2.dpad_right) {
                robot.outLeftClaw.setPosition(OUT_LEFT_CLAW_OPEN);
                robot.outRightClaw.setPosition(OUT_RIGHT_CLAW_OPEN);
            } else if (gamepad1.y) {
                robot.outWrist.setPosition(OUT_WRIST_UP);
                robot.outAxle.setPosition(OUT_PIVOT_UP);
            } else if (gamepad1.a) {
                robot.outWrist.setPosition(OUT_WRIST_DOWN);
                robot.outAxle.setPosition(OUT_PIVOT_DOWN);
            }

            if (gamepad1.b) {
                robot.rightClaw.setPosition(RIGHT_CLAW_OPEN);
                robot.leftClaw.setPosition(LEFT_CLAW_OPEN);

                robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                robot.leftClaw.setPosition(LEFT_CLAW_CLOSE);

                robot.wrist.setPosition(WRIST_UP);
                robot.axle.setPosition(PIVOT_UP);

                robot.outWrist.setPosition(OUT_WRIST_DOWN);
                robot.outAxle.setPosition(OUT_PIVOT_DOWN);

                robot.outRightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE);
                robot.outLeftClaw.setPosition(OUT_LEFT_CLAW_CLOSE);

                robot.leftClaw.setPosition(LEFT_CLAW_OPEN);
                robot.rightClaw.setPosition(RIGHT_CLAW_OPEN);

                robot.outWrist.setPosition(OUT_WRIST_UP);
                robot.outAxle.setPosition(OUT_PIVOT_UP);
            }
        }

    }


}
