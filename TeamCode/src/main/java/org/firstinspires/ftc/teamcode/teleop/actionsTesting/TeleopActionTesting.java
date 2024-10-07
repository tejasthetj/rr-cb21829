package org.firstinspires.ftc.teamcode.teleop.actionsTesting;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Master;

import java.util.ArrayList;
import java.util.List;

public class TeleopActionTesting extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    Master robot = new Master();

    @Override
    public void init() {
        robot.motorInit();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        robot.wrist.setPosition(WRIST_DOWN);
        robot.axle.setPosition(PIVOT_DOWN);

        if (gamepad1.a) {
            runningActions.add(new SequentialAction(
                    new SleepAction(1.0),
                    new ParallelAction(
                            new InstantAction(() -> robot.wrist.setPosition(WRIST_DOWN)),
                            new InstantAction(() -> robot.axle.setPosition(PIVOT_DOWN))
                    )
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
