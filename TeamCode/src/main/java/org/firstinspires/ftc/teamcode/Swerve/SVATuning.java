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
    enum TuningMode{
        ROTATION, TRANSLATION;
        TuningMode next(){
            return this==ROTATION?TRANSLATION:ROTATION;
        }
    }
    TuningMode tuningMode = TuningMode.ROTATION;
    List<List<Point2D>> point2DSs;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = InstanceTelemetry.init(telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        swerveDrive = new SwerveDrive(hardwareMap);
        while (opModeInInit()) {
            if (tuningMode == TuningMode.ROTATION) {
                telemetry.addLine("Tuning Mode: Rotation");
            } else {
                telemetry.addLine("Tuning Mode: Translation");
            }
            if (gamepad1.aWasReleased()) {
                tuningMode = tuningMode.next();
            }
            telemetry.update();
        }
        switch (tuningMode) {
            case ROTATION:
            waitForStart();
            swerveDrive.swerveController.gamepadInput(0, 0, 0.1);
            boolean initiated = false;
            while (!initiated) {
                boolean moved = true;
                for (WheelUnit wheelUnit : swerveDrive.swerveController.wheelUnits) {
                    if (Math.abs(Point2D.dot(Point2D.fromPolar(wheelUnit.getHeading(), 1), wheelUnit.getPosition())) < 0.01) {
                        moved = false;
                    }
                }
                if (moved) {
                    initiated = true;
                }
                swerveDrive.swerveController.gamepadInput(0, 0, 0.1);
            }
            double[] power = new double[SwerveDrive.PARAMS.unitNames.length];
            double[] velocity = new double[SwerveDrive.PARAMS.unitNames.length];
            for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                power[i] = 0;
                velocity[i] = 0;
            }
            while (opModeIsActive()) {
                for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                    swerveDrive.swerveController.wheelUnits[i].setPower(power[i]);
                    swerveDrive.swerveController.wheelUnits[i].update();
                    telemetry.addData("Motor" + SwerveDrive.PARAMS.unitNames[i] + "Power", power[i]);
                    telemetry.addData("Motor" +SwerveDrive.PARAMS.unitNames[i]+"Velocity", velocity[i]);
                }
                telemetry.update();
            }
            break;
            case TRANSLATION:
        }
    }
}
