package org.firstinspires.ftc.teamcode.autoTrajTesting;

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
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_READJUST;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.xml.transform.Source;

public class AllMechForRR {
    public static DcMotorEx elevatorLeft, elevatorRight, horizontalRight, horizontalLeft;
    public Servo leftClaw, rightClaw, wrist, axle, outAxle, outWrist, outRightClaw, outLeftClaw;

    PIDController rightVertController;
    PIDController leftVertController;
    PIDController rightHorController;
    PIDController leftHorController;
    public static double pv = 0.007, iv = 0, dv = 0.0002;
    public static double ph = 0, ih = 0, dh = 0;
    public static double fv = 0.05, fh = 0;

    public volatile int vertTarget = 0;
    public static int horTarget = 0;


    private final double ticks_in_degrees = 576.7/180;



    public AllMechForRR(HardwareMap hardwareMap) {
        horizontalRight = hardwareMap.get(DcMotorEx.class, "horizontal 1");
        horizontalLeft = hardwareMap.get(DcMotorEx.class, "horizontal 2");

        horizontalRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        axle = hardwareMap.get(Servo.class, "pivot servo");

        outAxle = hardwareMap.get(Servo.class, "outtake servo");
        outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        outWrist = hardwareMap.get(Servo.class, "out wrist servo");

        elevatorLeft = hardwareMap.get(DcMotorEx.class, "vertical 2");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "vertical 1");
        // elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        rightVertController = new PIDController(pv,iv,dv);
        leftVertController = new PIDController(pv, iv, dv);
        rightHorController = new PIDController(ph,ih,dh);
        leftHorController = new PIDController(ph,ih,dh);


    }

    public Action elevatorUp(int target) {
        return new InstantAction(() -> vertTarget = target);
    }


    public class UpdatePID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            System.out.println("vert target position" + vertTarget);
            rightVertController.setPID(pv,iv,dv);
            leftVertController.setPID(pv,iv,dv);
            rightHorController.setPID(ph,ih,dh);
            leftHorController.setPID(ph,ih,dh);

            int rightVertPos = elevatorRight.getCurrentPosition();
            int leftVertPos = elevatorLeft.getCurrentPosition();
            int rightHorPos = horizontalLeft.getCurrentPosition();
            int leftHorPos = horizontalRight.getCurrentPosition();

            double rightVertPid = rightVertController.calculate(rightVertPos,vertTarget);
            double leftVertPid = leftVertController.calculate(leftVertPos,vertTarget);
            double rightHorPid = rightHorController.calculate(rightHorPos, horTarget);
            double leftHorPid = leftHorController.calculate(leftHorPos, horTarget);

            double vertFF = Math.cos(Math.toRadians(vertTarget / ticks_in_degrees)) * fv;
            double hotFF = Math.cos(Math.toRadians(horTarget / ticks_in_degrees)) * fh;

            double rightVertPower = rightVertPid + vertFF;
            double leftVertPower = leftVertPid + vertFF;
            double rightHorPower = rightHorPid + hotFF;
            double leftHorPower = leftHorPid + hotFF;

            elevatorRight.setPower(rightVertPower);
            elevatorLeft.setPower(leftVertPower);
            horizontalRight.setPower(rightHorPower);
            horizontalLeft.setPower(leftHorPower);
            System.out.println(elevatorRight);
            System.out.println(elevatorLeft);
            System.out.println("elevator right power: " + rightVertPower);
            System.out.println("elevator left power: " + leftVertPower);
            System.out.println("Current Elevator right mdoe: " + elevatorRight.getMode());
            System.out.println("Curent Elevator left mode: " + elevatorLeft.getMode());

            return true;
        }
    }
    public Action updatePID() {
        return new UpdatePID();
    }

    public class IntakeClawAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(WRIST_DOWN);
            axle.setPosition(PIVOT_DOWN);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            try {
                sleep(700);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            wrist.setPosition(WRIST_UP);
            axle.setPosition(PIVOT_UP);
            rightClaw.setPosition(RIGHT_CLAW_READJUST);
            leftClaw.setPosition(LEFT_CLAW_READJUST);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }
    public Action intakeClawAction() {
        return new IntakeClawAction();
    }

    public class OuttakeClawAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outLeftClaw.setPosition(OUT_LEFT_CLAW_OPEN);
            outRightClaw.setPosition(OUT_RIGHT_CLAW_OPEN);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            wrist.setPosition(WRIST_READJUST);
            axle.setPosition(PIVOT_READJUST);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            outWrist.setPosition(OUT_WRIST_DOWN);
            outAxle.setPosition(OUT_PIVOT_UP);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            return false;
        }
    }
    public Action outtakeClawAction() {
        return new OuttakeClawAction();
    }

    public class ResetClassAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            outLeftClaw.setPosition(OUT_LEFT_CLAW_CLOSE);
            outRightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            outWrist.setPosition(OUT_WRIST_UP);
            outAxle.setPosition(OUT_PIVOT_DOWN);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            wrist.setPosition(WRIST_READJUST);
            axle.setPosition(PIVOT_READJUST);
            return false;
        }
    }
    public Action resetClassAction() {
        return new ResetClassAction();
    }
}
