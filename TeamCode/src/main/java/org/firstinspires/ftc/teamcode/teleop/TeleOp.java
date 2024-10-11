package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Master;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleop testing", group = "exercises")
public class TeleOp extends LinearOpMode {

    Master robot = new Master();
    Boolean fCentric = Boolean.FALSE;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        while (opModeInInit()) {
            if (gamepad1.a) {
                fCentric = Boolean.FALSE;
            } else if (gamepad1.b) {
                fCentric = Boolean.TRUE;
            }
        }

        while (opModeIsActive()) {

            robot.centricMovement(fCentric);
            robot.servoMovements();


        }

    }




}
