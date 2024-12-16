package org.firstinspires.ftc.teamcode.autoTrajTesting;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class HorizontalPIDTesting extends OpMode {
    private PIDController leftHorController;
    private PIDController rightHorController;

    public static double ph = 0, ih = 0, dh = 0;
    public static double fh = 0;

    public static int horTarget = 0;

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
