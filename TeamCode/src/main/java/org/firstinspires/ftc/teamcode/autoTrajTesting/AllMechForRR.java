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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AllMechForRR {
    public DcMotorEx elevatorLeft, elevatorRight, Horizontalright, Horizontalleft;
    public Servo leftClaw, rightClaw, wrist, axle, outAxle, outWrist, outRightClaw, outLeftClaw;



    public AllMechForRR(HardwareMap hardwareMap) {
        Horizontalright = hardwareMap.get(DcMotorEx.class, "horizontal 1");
        Horizontalleft = hardwareMap.get(DcMotorEx.class, "horizontal 2");

        Horizontalleft.setDirection(DcMotorSimple.Direction.REVERSE);

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
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public class ElevatorUp implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                elevatorLeft.setPower(0.4);
                elevatorRight.setPower(0.4);
                initialized = true;
            }

            double posLeft = elevatorLeft.getCurrentPosition();
            double posRight = elevatorRight.getCurrentPosition();
            packet.put("leftPos", posLeft);
            packet.put("rightPos", posRight);

            if (posLeft < 3100.0 && posRight < 3100.0) {
                return true;
            } else {
                rightClaw.setPosition(OUT_RIGHT_CLAW_OPEN);
                leftClaw.setPosition(OUT_LEFT_CLAW_OPEN);
                elevatorRight.setPower(0);
                elevatorLeft.setPower(0);
                return false;
            }

        }
    }
    public Action elevatorUp() {
        return new ElevatorUp();
    }

    public class ElevatorDown implements Action {
        private boolean initialized = false;



        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                elevatorLeft.setPower(-0.4);
                elevatorRight.setPower(-0.4);
                initialized = true;
            }

            double posLeft = elevatorLeft.getCurrentPosition();
            double posRight = elevatorRight.getCurrentPosition();
            packet.put("leftPos", posLeft);
            packet.put("rightPos", posRight);

            if (posLeft > 10.0 && posRight > 10.0) {
                return true;
            } else {
                elevatorLeft.setPower(0);
                elevatorRight.setPower(0);
                return false;
            }

        }
    }
    public Action elevatorDown() {
        return new ElevatorDown();
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
