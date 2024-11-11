package org.firstinspires.ftc.teamcode.teleop.actionsTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder Count Tester", group = "exercises")
public class EncoderCount extends LinearOpMode {

    DcMotor elevatorRight, elevatorLeft;


    @Override
    public void runOpMode() throws InterruptedException {

        elevatorLeft = hardwareMap.get(DcMotor.class, "vertical 2");
        elevatorRight = hardwareMap.get(DcMotor.class, "vertical 1");
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Reset Values", "Done");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Left Elevator Encoder Count", elevatorLeft.getCurrentPosition());
            telemetry.addData("Right Elevator Encoder Count", elevatorRight.getCurrentPosition());

            telemetry.update();
        }
    }
}
