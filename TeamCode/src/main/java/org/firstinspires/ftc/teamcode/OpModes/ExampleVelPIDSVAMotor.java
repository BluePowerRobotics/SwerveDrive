package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.controllers.ExampleVoltageOutMotor;

@Config
@TeleOp(name = "Example VoltageOut Motor")
public class ExampleVelPIDSVAMotor extends LinearOpMode {
    public static double targetVelocity = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExampleVoltageOutMotor flywheel = new ExampleVoltageOutMotor(hardwareMap, "SVA", telemetry);
        waitForStart();
        while (opModeIsActive()) {
            // 用左摇杆Y轴控制目标转速（-1~1 -> -max~max）
            targetVelocity = -gamepad1.left_stick_y * 2000; // 2000为示例最大速度
            flywheel.setTargetVelocity(targetVelocity);
            flywheel.update();
            if (gamepad1.a) {
                flywheel.setTargetVelocity(0);
            }
            telemetry.addData("Setpoint", targetVelocity);
            telemetry.update();
        }
        flywheel.stop();
    }
}
