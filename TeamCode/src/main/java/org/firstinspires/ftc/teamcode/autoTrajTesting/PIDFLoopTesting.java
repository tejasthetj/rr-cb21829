package org.firstinspires.ftc.teamcode.autoTrajTesting;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;


@Config
@Autonomous(name = "testing pid loop", group = "exercise")
public class PIDFLoopTesting extends OpMode {
    private PIDFController controllerLeft;
    private PIDFController controllerRight;

    public static double p = 0, i = 0, d = 0, f = 0;
    public static double p1 = 0, i1 = 0, d1 = 0, f1 = 0;

    public static int target = 0;

    private final double ticks_in_degree = 537.6 / 180.0;

    public DcMotorEx elevatorLeft, elevatorRight;


    @Override
    public void init() {
        controllerLeft = new PIDFController(p, i, d, f);
        controllerRight = new PIDFController(p1, i1, d1, f1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        elevatorLeft = hardwareMap.get(DcMotorEx.class, "vertical 2");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "vertical 1");
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void loop() {
        controllerLeft.setPIDF(p, i, d, f);
        controllerRight.setPIDF(p, i, d, f);
        int elevatorRightPos = elevatorRight.getCurrentPosition();
        int elevatorLeftPos = elevatorLeft.getCurrentPosition();

        double pid = controllerLeft.calculate(elevatorLeftPos, target);
        double pid2 = controllerRight.calculate(elevatorRightPos, target);

        double ff = Math.cos(Math.toRadians(target)) * f;
        double ff2 = Math.cos(Math.toRadians(target)) * f1;

        double powerLeft = pid + ff;
        double powerRight = pid2 + ff2;

        elevatorRight.setPower(powerRight);
        elevatorLeft.setPower(powerLeft);

        telemetry.addData("left pos:", elevatorLeftPos);
        telemetry.addData("right pos", elevatorRightPos);
        telemetry.addData("target:", target);
        telemetry.update();

    }
}
