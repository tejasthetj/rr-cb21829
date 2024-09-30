package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.TeleOp.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.teleop.TeleOp.LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.teleop.TeleOp.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.teleop.TeleOp.RIGHT_CLAW_OPEN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTesting extends LinearOpMode {

    TeleOp servoConstants = new TeleOp();

    @Override
    public void runOpMode() throws InterruptedException {
        servoConstants.leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        servoConstants.rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        servoConstants.wrist = hardwareMap.get(Servo.class, "wrist servo");
        servoConstants.lebronJames = hardwareMap.get(Servo.class, "pivot servo");

        servoConstants.bethBurry = hardwareMap.get(Servo.class, "outtake servo");
        servoConstants.outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        servoConstants.outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        servoConstants.outWrist = hardwareMap.get(Servo.class, "out wrist servo");



        if (gamepad1.left_bumper) {
            servoConstants.leftClaw.setPosition(LEFT_CLAW_OPEN);
            servoConstants.rightClaw.setPosition(RIGHT_CLAW_OPEN);
        } else if (gamepad1.right_bumper) {
            servoConstants.leftClaw.setPosition(LEFT_CLAW_CLOSE);
            servoConstants.rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        }


    }
}
