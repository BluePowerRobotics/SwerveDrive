package org.firstinspires.ftc.teamcode.VoltageOutTester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.VoltageOut;
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;

@TeleOp
@Config
public class VoltageOutTester extends LinearOpMode {
    public static  int voltageFilterWindow = 10;
    @Override
    public void runOpMode() throws InterruptedException {

        MeanFilter voltageFilter = new MeanFilter(voltageFilterWindow);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VoltageOut voltageOut = new VoltageOut(hardwareMap);

        DcMotorEx V_Motor = hardwareMap.get(DcMotorEx.class, "VMotor");
        DcMotorEx D_Motor = hardwareMap.get(DcMotorEx.class, "DMotor");
        DcMotorEx CurrentEater = hardwareMap.get(DcMotorEx.class, "CurrentEater");

        V_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        D_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        CurrentEater.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        V_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        D_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CurrentEater.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Voltage", voltageOut.getVoltage());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            double targetVoltage = 8; // Example target voltage
            double power = targetVoltage / 12;
            double V_power = voltageOut.getVoltageOutPower(targetVoltage);
            if(gamepad1.a){
                V_Motor.setPower(V_power);
            }
            if(gamepad1.b){
                D_Motor.setPower(power);
            }

            if(gamepad1.x) {
                CurrentEater.setPower(1);
                V_Motor.setPower(V_power);
            }
            if(gamepad1.y) {
                CurrentEater.setPower(1);
                D_Motor.setPower(power);
            }

            if(gamepad1.back){
                CurrentEater.setPower(0);
                V_Motor.setPower(0);
                D_Motor.setPower(0);
            }

            telemetry.addData("Current Voltage", voltageOut.getVoltage());
            telemetry.addData("Filtered Voltage", voltageFilter.filter(voltageOut.getVoltage()));
            telemetry.addData("Target Voltage", targetVoltage);
            telemetry.addData("Output Power", V_power);
            telemetry.addData("power", power);
            telemetry.addData("V_vel", V_Motor.getVelocity());
            telemetry.addData("D_vel", D_Motor.getVelocity());
            telemetry.update();
        }
    }
}
