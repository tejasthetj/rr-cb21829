package org.firstinspires.ftc.teamcode.teleop.actionsTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder Count Tester", group = "exercises")
public class EncoderCount extends LinearOpMode {

    private DcMotor elevatorRight, elevatorLeft, horizontalRight, horizontalLeft;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        elevatorLeft = hardwareMap.get(DcMotor.class, "vertical 2");
        elevatorRight = hardwareMap.get(DcMotor.class, "vertical 1");
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontalRight = hardwareMap.get(DcMotor.class, "horizontal 1");
        horizontalLeft = hardwareMap.get(DcMotor.class, "horizontal 2");
        horizontalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Reset Values", "Done");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Left Horizontal Encoder Count", horizontalLeft.getCurrentPosition());
            telemetry.addData("Right Horizontal Encoder Count", horizontalRight.getCurrentPosition());
            telemetry.addData("Left Elevator Encoder Count", elevatorLeft.getCurrentPosition());
            telemetry.addData("Right Elevator Encoder Count", elevatorRight.getCurrentPosition());


            telemetry.update();
        }
    }
}
