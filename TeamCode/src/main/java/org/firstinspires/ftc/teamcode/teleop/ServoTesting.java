package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Master;

public class ServoTesting extends LinearOpMode {

    Master robot = new Master();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.motorInit();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.leftClaw.setPosition(LEFT_CLAW_OPEN);
                robot.rightClaw.setPosition(RIGHT_CLAW_OPEN);
            } else if (gamepad1.right_bumper) {
                robot.leftClaw.setPosition(LEFT_CLAW_CLOSE);
                robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            }

        }



    }


}
