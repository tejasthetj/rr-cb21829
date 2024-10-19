package org.firstinspires.ftc.teamcode.outReach;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Outreach Teleop", group = "exercises")
public class OutReachMain extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;

//    DcMotor elevator;

    Servo drone;

    double DRONE_LAUNCH = 1;
    double DRONE_RETRACT = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "left_Motor1");
        frontRight = hardwareMap.get(DcMotor.class, "right_Motor1");
        rearLeft = hardwareMap.get(DcMotor.class, "left_Motor2");
        rearRight = hardwareMap.get(DcMotor.class, "right_Motor2");

//        elevator = hardwareMap.get(DcMotor.class, "Elevator");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        drone = hardwareMap.get(Servo.class, "drone");

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Clipping the power to make sure it doesn't exceed the maximum value
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.45);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(backRightPower);


            if (gamepad1.b) {
                drone.setPosition(DRONE_LAUNCH);
            } else if (gamepad1.a) {
                drone.setPosition(DRONE_RETRACT);
            }

//            if (gamepad1.right_trigger > 0) {
//                elevator.setPower(-.4);
//            } else if (gamepad1.left_trigger > 0)  {
//                elevator.setPower(.4);
//            }


        }
    }
}
