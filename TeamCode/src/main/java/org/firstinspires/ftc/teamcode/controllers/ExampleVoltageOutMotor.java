package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utility.PIDSVAController;
import org.firstinspires.ftc.teamcode.utility.SlotConfig;
import org.firstinspires.ftc.teamcode.utility.VoltageOut;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ExampleVoltageOutMotor {
    // Dashboard 热调参参数
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double maxI = 1.0;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double outputMin = -14.0;
    public static double outputMax = 14.0;

    private final DcMotorEx motor;
    private final VoltageOut voltageOut;
    private final PIDSVAController controller;
    private final SlotConfig config;
    private final Telemetry telemetry;
    private double targetVelocity = 0;
    private long lastUpdateTime = 0;

    public ExampleVoltageOutMotor(HardwareMap hardwareMap, String motorName, Telemetry telemetry) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.voltageOut = new VoltageOut(hardwareMap);
        this.config = new SlotConfig()
                .withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        this.controller = PIDSVAController.create().withSlot0(config);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.telemetry = telemetry;
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void update() {
        // 动态同步 SlotConfig 参数，支持 Dashboard 热调参
        config.withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
              .withKS(kS).withKV(kV).withKA(kA)
              .withOutputLimits(outputMin, outputMax);
        long now = System.currentTimeMillis();
        double dt = lastUpdateTime == 0 ? 0.02 : (now - lastUpdateTime) / 1000.0;
        lastUpdateTime = now;
        double currentVelocity = motor.getVelocity();
        double outputVoltage = controller.calculate(targetVelocity, currentVelocity, targetVelocity, 0.0, dt);
        double power = voltageOut.getVoltageOutPower(outputVoltage);
        motor.setPower(power);
        telemetry.addData("TargetVelocity", targetVelocity);
        telemetry.addData("CurrentVelocity", currentVelocity);
        telemetry.addData("OutputVoltage", outputVoltage);
        telemetry.addData("Power", power);
    }

    public void stop() {
        motor.setPower(0);
    }
}
