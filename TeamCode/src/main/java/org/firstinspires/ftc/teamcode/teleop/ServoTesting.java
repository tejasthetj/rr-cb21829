package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.WRIST_UP;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.teleop.TeleOpProgram.PIVOT_DOWN;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo Testing", group = "exercises")
public class ServoTesting extends LinearOpMode {

    TeleOpProgram servoConstants = new TeleOpProgram();

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




        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {

                servoConstants.leftClaw.setPosition(LEFT_CLAW_OPEN);
                servoConstants.rightClaw.setPosition(RIGHT_CLAW_OPEN);

            } else if (gamepad1.right_bumper) {

                servoConstants.leftClaw.setPosition(LEFT_CLAW_CLOSE);
                servoConstants.rightClaw.setPosition(RIGHT_CLAW_CLOSE);

            } else if (gamepad1.dpad_up) {

                servoConstants.wrist.setPosition(WRIST_UP);
                servoConstants.lebronJames.setPosition(PIVOT_UP);

            } else if (gamepad1.dpad_down) {

                servoConstants.wrist.setPosition(WRIST_DOWN);
                servoConstants.lebronJames.setPosition(PIVOT_DOWN);

            }
        }



    }
}
