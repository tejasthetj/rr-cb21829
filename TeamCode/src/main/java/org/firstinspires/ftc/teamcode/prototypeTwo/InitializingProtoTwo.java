package org.firstinspires.ftc.teamcode.prototypeTwo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Testing motors", group = "exercises")
public class InitializingProtoTwo extends LinearOpMode {
    public DcMotor leftFront, rightFront, rightRear, leftRear;
    public DcMotor linkage, elevator;

    public Servo claw, wrist, leftElbow, rightElbow, leftMain, rightMain;

    

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware maps for all the motors.
        leftFront = hardwareMap.get(DcMotor.class, "left front motor");
        rightFront = hardwareMap.get(DcMotor.class, "right front motor");
        rightRear = hardwareMap.get(DcMotor.class, "right rear motor");
        leftRear = hardwareMap.get(DcMotor.class, "left rear motor");

        linkage = hardwareMap.get(DcMotor.class, "linkage");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftElbow = hardwareMap.get(Servo.class, "left elbow");
        rightElbow = hardwareMap.get(Servo.class, "right elbow");
        leftMain = hardwareMap.get(Servo.class, "left main");
        rightMain = hardwareMap.get(Servo.class, "right main");


        //Set reverse directions if necessary.
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0) {
                elevator.setPower(0.4);
            }

            if (gamepad1.left_trigger > 0) {
                elevator.setPower(-0.4);
            }

            if (gamepad1.dpad_up) {
                linkage.setPower(-1);
            }

            if (gamepad1.dpad_down) {
                linkage.setPower(1);
            }
        }


    }
}
