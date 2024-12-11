package org.firstinspires.ftc.teamcode.teleop.actionsTesting;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
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

import org.firstinspires.ftc.teamcode.Master;
import org.firstinspires.ftc.teamcode.autoTrajTesting.AllMechForRR;

import java.util.ArrayList;
import java.util.List;

public class TeleopActionTesting extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    AllMechForRR robot = new AllMechForRR(hardwareMap);

    @Override
    public void init() {
        SequentialAction intakeClawAction;

        intakeClawAction = new SequentialAction(
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
        );
    }
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();



        runningActions.add(new ParallelAction(
                robot.robotCentric(),
                robot.teleopIntakeClaw(),
                robot.teleopOuttakeClaw(),
                robot.updatePID(),
                robot.teleopElevatorSetting()
                )
        );

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
