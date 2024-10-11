package org.firstinspires.ftc.teamcode.teleop;

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
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_UP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Master;

import java.util.ArrayList;
import java.util.List;

public class TeleopActions extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    public static int vertright = 4000;

    Master robot = new Master();


    @Override
    public void init() {
        robot.motorInit();
    }


    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();


        if (gamepad1.a) {
            runningActions.add(new SequentialAction(
                    new SleepAction(1.0),
                    new InstantAction(() -> robot.wrist.setPosition(WRIST_DOWN)),
                    new InstantAction(() -> robot.axle.setPosition(PIVOT_DOWN)),
                    new SleepAction(0.5),
                    new ParallelAction(
                            new InstantAction(() -> robot.leftClaw.setPosition(LEFT_CLAW_CLOSE)),
                            new InstantAction(() -> robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE))
                    ),
                    new SleepAction(1.0),
                    new InstantAction(() -> robot.wrist.setPosition(WRIST_UP)),
                    new InstantAction(() -> robot.axle.setPosition(PIVOT_UP))
            ));
        }  else if (gamepad1.b) {
            runningActions.add(new SequentialAction(
                    new SleepAction(1.0),
                    new InstantAction(() -> robot.wrist.setPosition(OUT_WRIST_UP)),
                    new InstantAction(() -> robot.axle.setPosition(OUT_PIVOT_UP)),
                    new SleepAction(0.5),
                    new ParallelAction(
                            new InstantAction(() -> robot.leftClaw.setPosition(OUT_LEFT_CLAW_CLOSE)),
                            new InstantAction(() -> robot.rightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE))
                    ),
                    new SleepAction(1.0),
                    new ParallelAction(
                    new InstantAction(() -> robot.wrist.setPosition(OUT_WRIST_DOWN)),
                    new InstantAction(() -> robot.axle.setPosition(OUT_PIVOT_DOWN)),
                    new InstantAction(() -> robot.Elevatorleft.setTargetPosition(vertright)))


            ));
        }


        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }

        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }


    }
