package org.firstinspires.ftc.teamcode.Swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.WheelUnit;
import org.firstinspires.ftc.teamcode.utility.Point2D;

import java.util.List;

@TeleOp
public class SVATuning extends LinearOpMode {
    SwerveDrive swerveDrive;
    List<List<Point2D>> point2DSs;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = InstanceTelemetry.init(telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        swerveDrive = new SwerveDrive(hardwareMap);
        waitForStart();
        swerveDrive.swerveController.gamepadInput(0,0,0.1);
        boolean inited=false;
        while (!inited) {
            boolean moved = true;
            for (WheelUnit wheelUnit : swerveDrive.swerveController.wheelUnits) {
                if (Math.abs(Point2D.dot(Point2D.fromPolar(wheelUnit.getHeading(), 1), wheelUnit.getPosition())) < 0.01) {
                    moved = false;
                }
            }
            if (moved) {
                inited = true;
            }
            swerveDrive.swerveController.gamepadInput(0,0,0.1);
        }
        DcMotorEx[] dcMotorExes = new DcMotorEx[SwerveDrive.PARAMS.unitNames.length];
        double[] power=new double[SwerveDrive.PARAMS.unitNames.length];
        double[] velocity=new double[SwerveDrive.PARAMS.unitNames.length];
        for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
            dcMotorExes[i] = hardwareMap.get(DcMotorEx.class, SwerveDrive.PARAMS.unitNames[i]);
            power[i]=0;
            velocity[i]=0;
        }
        while(opModeIsActive()){
            for(int i=0;i<SwerveDrive.PARAMS.unitNames.length;i++) {
                telemetry.addData("Motor"+i+"Power", power[i]);
                telemetry.addData("Motor Velocity", velocity[i]);
            }
            telemetry.update();
        }
    }
}
