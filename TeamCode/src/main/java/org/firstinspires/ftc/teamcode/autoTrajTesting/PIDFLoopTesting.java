package org.firstinspires.ftc.teamcode.autoTrajTesting;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Config
@Autonomous(name = "testing pid loop", group = "exercise")
public class PIDFLoopTesting extends OpMode {
        private PIDController rightVertController;
        private PIDController leftVertController;
        private PIDController rightHorController;
        private PIDController leftHorController;
        public static double pv = 0.007, iv = 0, dv = 0.0002;
        public static double ph = 0.006, ih = 0, dh = 0.002;
        public static double fv = 0.05, fh = 0.001;

        public static int vertTarget = 0;
        public static int horTarget = 0;


        private final double ticks_in_degrees = 576.7/180;
        private DcMotor elevatorRight, elevatorLeft, horizontalRight, horizontalLeft;



        @Override
        public void init() {
            rightVertController = new PIDController(pv,iv,dv);
            leftVertController = new PIDController(pv, iv, dv);
            rightHorController = new PIDController(ph,ih,dh);
            leftHorController = new PIDController(ph,ih,dh);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


            elevatorRight = hardwareMap.get(DcMotor.class, "vertical 1");
            elevatorLeft = hardwareMap.get(DcMotor.class, "vertical 2");
            horizontalRight = hardwareMap.get(DcMotor.class, "horizontal 1");
            horizontalLeft = hardwareMap.get(DcMotor.class, "horizontal 2");
//            elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            horizontalRight.setDirection(DcMotorSimple.Direction.REVERSE);
            elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        }

        @Override
        public void loop() {

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

            double vertff = Math.cos(Math.toRadians(vertTarget / ticks_in_degrees)) * fv;
            double horff = Math.cos(Math.toRadians(horTarget / ticks_in_degrees)) * fh;

            double rightVertPower = rightVertPid + vertff;
            double leftVertPower = leftVertPid + vertff;
            double rightHorPower = rightHorPid + horff;
            double leftHorPower = leftHorPid + horff;

            elevatorRight.setPower(rightVertPower);
            elevatorLeft.setPower(leftVertPower);
            horizontalRight.setPower(rightHorPower);
            horizontalLeft.setPower(leftHorPower);
//            telemetry.addData("Right vertical pos", rightVertPos);
//            telemetry.addData("Left vertical pos", leftVertPos);
            telemetry.addData("Right horizontal pos", rightHorPos);
            telemetry.addData("Left horizontal pos", leftHorPos);
            telemetry.addData("Left horizontal power", leftVertPower);
            telemetry.addData("Right horizontal power", rightVertPower);
//            telemetry.addData("Vetical Target", vertTarget);
            telemetry.addData("Horizontal Target", horTarget);
            telemetry.update();










        }
}
