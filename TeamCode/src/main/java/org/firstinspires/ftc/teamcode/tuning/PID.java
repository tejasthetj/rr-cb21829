package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class PID extends OpMode {
private PIDController rightVertController;
private PIDController leftVertController;
private PIDController rightHorController;
private PIDController leftHorController;
public static double pvr = 0, ivr = 0, dvr = 0;
    public static double pvl = 0, ivl = 0, dvl = 0;
    public static double phr = 0, ihr = 0, dhr = 0;
    public static double phl = 0, ihl = 0, dhl = 0;
public static final double f = 0;

public static int vertTarget = 0;
public static int horTarget = 0;


private final double ticks_in_inches = 576.7/11.3;
    private DcMotor Elevatorright, Elevatorleft, Horizontalright, Horizontalleft;
    @Override
    public void init() {
        rightVertController = new PIDController(pvr,ivr,dvr);
        leftVertController = new PIDController(pvl, ivl, dvl);
        rightHorController = new PIDController(phr,ihr,dhr);
        leftHorController = new PIDController(phl,ihl,dhl);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Elevatorright = hardwareMap.get(DcMotor.class, "vertical 1");
        Elevatorleft = hardwareMap.get(DcMotor.class, "vertical 2");
        Horizontalright = hardwareMap.get(DcMotor.class, "horizontal 1");
        Horizontalleft = hardwareMap.get(DcMotor.class, "horizontal 2");
        Horizontalleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Elevatorright.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    @Override
    public void loop() {

        rightVertController.setPID(pvr,ivr,dvr);
        leftVertController.setPID(pvl,ivl,dvl);
        rightHorController.setPID(phr,ihr,dhr);
        leftHorController.setPID(phl,ihl,dhl);
        int rightVertPos = Elevatorright.getCurrentPosition();
        int leftVertPos = Elevatorleft.getCurrentPosition();
        int rightHorPos = Horizontalleft.getCurrentPosition();
        int leftHorPos = Horizontalright.getCurrentPosition();
        double rightVertPid = rightVertController.calculate(rightVertPos,vertTarget);
        double leftVertPid = leftVertController.calculate(leftVertPos,vertTarget);
        double rightHorPid = rightHorController.calculate(rightHorPos,horTarget);
        double leftHorPid = leftHorController.calculate(leftHorPos,horTarget);
        double vertff = Math.cos(Math.toRadians(vertTarget / ticks_in_inches)) * f;
        double horff = Math.cos(Math.toRadians(horTarget / ticks_in_inches)) * f;

        double rightVertPower = rightVertPid + vertff;
        double leftVertPower = leftVertPid + vertff;
        double rightHorPower = rightHorPid + horff;
        double leftHorPower = leftHorPid + horff;

        Elevatorright.setPower(rightVertPower);
        Elevatorleft.setPower(leftVertPower);
        Horizontalright.setPower(rightHorPower);
        Horizontalleft.setPower(leftHorPower);

        telemetry.addData("Right vertical pos",rightVertPos );
        telemetry.addData("Left vertical pos",leftVertPos );
        telemetry.addData("Right horizontal pos",rightHorPos );
        telemetry.addData("Left horizontal pos",leftHorPos );
        telemetry.addData("Vetical Target", vertTarget);
        telemetry.addData("Horizontal Target", horTarget);
        telemetry.update();










    }
}
