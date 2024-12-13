package org.firstinspires.ftc.teamcode.teleop.actionsTesting;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_READJUST;
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

    SequentialAction intakeClawAction = new SequentialAction(
            new InstantAction(() -> robot.wrist.setPosition(WRIST_DOWN)),
            new InstantAction(() -> robot.axle.setPosition(PIVOT_DOWN)),
            new ParallelAction(
                    new InstantAction(() -> robot.leftClaw.setPosition(LEFT_CLAW_CLOSE)),
                    new InstantAction(() -> robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE))
            ),
            new InstantAction(() -> robot.wrist.setPosition(WRIST_UP)),
            new InstantAction(() -> robot.axle.setPosition(PIVOT_UP)),
            new ParallelAction(
                    new InstantAction(() -> robot.rightClaw.setPosition(RIGHT_CLAW_READJUST)),
                    new InstantAction(() -> robot.leftClaw.setPosition(LEFT_CLAW_READJUST))
            )
    );

    SequentialAction outtakeClawAction = new SequentialAction(
            new ParallelAction(
                    new InstantAction(() -> robot.outLeftClaw.setPosition(OUT_LEFT_CLAW_OPEN)),
                    new InstantAction(() -> robot.outRightClaw.setPosition(OUT_RIGHT_CLAW_OPEN))
            ),
            new ParallelAction(
                    new InstantAction(() -> robot.rightClaw.setPosition(RIGHT_CLAW_OPEN)),
                    new InstantAction(() -> robot.leftClaw.setPosition(LEFT_CLAW_OPEN))
            ),
            new InstantAction(() -> robot.wrist.setPosition(WRIST_READJUST)),
            new InstantAction(() -> robot.axle.setPosition(PIVOT_READJUST)),
            new InstantAction(() -> robot.wrist.setPosition(WRIST_DOWN)),
            new InstantAction(() -> robot.axle.setPosition(PIVOT_DOWN))
    );

    SequentialAction resetClawAction = new SequentialAction(
            new InstantAction(() -> robot.wrist.setPosition(WRIST_READJUST)),
            new InstantAction(() -> robot.axle.setPosition(PIVOT_READJUST)),
            new ParallelAction(
                    new InstantAction(() -> robot.leftClaw.setPosition(LEFT_CLAW_OPEN)),
                    new InstantAction(() -> robot.rightClaw.setPosition(LEFT_CLAW_OPEN))
            ),
            new InstantAction(() -> robot.outWrist.setPosition(OUT_WRIST_UP)),
            new InstantAction(() -> robot.outAxle.setPosition(OUT_PIVOT_UP)),
            new ParallelAction(
                    new InstantAction(() -> robot.rightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE)),
                    new InstantAction(() -> robot.leftClaw.setPosition(OUT_LEFT_CLAW_CLOSE))
            ),
            new ParallelAction(
                    new InstantAction(() -> robot.setHorizontalTarget(20)),
                    robot.updatePID()
            )
    );


    @Override
    public void init() {



    }
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        AllMechForRR.frontLeft.setPower(frontLeftPower);
        AllMechForRR.rearLeft.setPower(backLeftPower);
        AllMechForRR.frontRight.setPower(frontRightPower);
        AllMechForRR.rearLeft.setPower(backRightPower);

        // add whatever gamepads you need.(claw actions)

        //intake
        if (gamepad2.a) {
            runningActions.add(
                    intakeClawAction
            );
        }

        //reset
        if (gamepad2.b) {
            runningActions.add(
                    resetClawAction
            );
        }

        //outtake
        if (gamepad2.x) {
            runningActions.add(
              outtakeClawAction
            );
        }

        // basket drop
        if (gamepad2.dpad_up) {
            runningActions.add(
                    new ParallelAction(
                            robot.updatePID(),
                            robot.setElevatorTarget(3400)
                    )
            );
        }

        // reset down
        if (gamepad2.dpad_down) {
            runningActions.add(
                    new ParallelAction(
                            robot.updatePID(),
                            robot.setElevatorTarget(20)
                    )
            );
        }

        // specimen up
        if (gamepad2.dpad_right) {
            runningActions.add(
              new ParallelAction(
                      robot.updatePID(),
                      robot.setElevatorTarget(1500)
              )
            );
        }

        // specimen left
        if (gamepad2.dpad_left) {
            runningActions.add(
                    new ParallelAction(
                            robot.updatePID(),
                            robot.setElevatorTarget(600)
                    )
            );
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
