package org.firstinspires.ftc.teamcode.Swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.WheelUnit;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
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
    enum RotationState {
        kS_ASSESSING,
        kS_kV_FITTING,
        kJ_ASSESSING
    }
    RotationState rotationState = RotationState.kS_ASSESSING;
    TuningMode tuningMode = TuningMode.ROTATION;
    List<List<Point2D>> point2DSs;
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;
    public double kM = 0;//等效质量
    public double kJ = 0;//等效旋转惯量
    public static int kS_kV_TestPoints = 20;
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
                double power = 0;
                double[] velocity = new double[SwerveDrive.PARAMS.unitNames.length];
                for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                    velocity[i] = 0;
                }

                boolean kS_KV_Started = false;
                double UsableTestPower = 0;
                double testPowerIncrement = 0;
                int currentTestPoint = 0;
                while (opModeIsActive()) {
                    switch(rotationState) {
                        case kS_ASSESSING:
                            power += 0.01;
                            boolean allMoving = true;
                            for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                                double v = swerveDrive.swerveController.wheelUnits[i].getVelocityInTPS();
                                if (Math.abs(v) < MathSolver.toInch(200)) {
                                    allMoving = false;
                                }
                                velocity[i] = v;
                            }
                            if (allMoving) {
                                rotationState = RotationState.kS_kV_FITTING;
                                telemetry.addData("kS Assessed", kS = power*swerveDrive.swerveController.voltageSensor.getVoltage());
                                telemetry.update();
                            }
                            break;
                        case kS_kV_FITTING:


                            //devide (power - kS) to 20 target power
                            if (!kS_KV_Started) {
                                UsableTestPower = swerveDrive.swerveController.voltageSensor.getVoltage() - kS;
                                testPowerIncrement = UsableTestPower / kS_kV_TestPoints;
                                kS_KV_Started = true;
                            }
                            power = kS + testPowerIncrement * currentTestPoint;
                            break;
                        case kJ_ASSESSING:
                            break;
                    }
                    for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                        swerveDrive.swerveController.wheelUnits[i].setPower(power/swerveDrive.swerveController.voltageSensor.getVoltage());
                        swerveDrive.swerveController.wheelUnits[i].update();
                        velocity[i]= swerveDrive.swerveController.wheelUnits[i].getVelocityInTPS();
                        telemetry.addData("Motor" + SwerveDrive.PARAMS.unitNames[i] + "Power", power);
                        telemetry.addData("Motor" +SwerveDrive.PARAMS.unitNames[i]+"Velocity", velocity[i]);
                    }
                    telemetry.update();
                }
                break;
            case TRANSLATION:
        }
    }
}
