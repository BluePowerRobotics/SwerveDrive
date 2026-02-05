package org.firstinspires.ftc.teamcode.Swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.WheelUnit;
import org.firstinspires.ftc.teamcode.utility.Line;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;

import java.util.ArrayList;
import java.util.List;

@Config
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
        kJ_WAITING,
        kJ_ASSESSING,
        FINISHED
    }
    RotationState rotationState = RotationState.kS_ASSESSING;
    TuningMode tuningMode = TuningMode.ROTATION;
    List<Point2D> point2Ds_SV = new ArrayList<>();
    List<Point2D> point2Ds_kJ = new ArrayList<>();
    List<Double> testAccelerations = new ArrayList<>();
    MeanFilter meanFilter_SV = new MeanFilter(20);
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;
    public double kM = 0;//等效质量
    public double kJ = 0;//等效旋转惯量

    //static参数可调
    public static int kS_kV_TestPoints = 20;
    public static double kS_kV_TestMaxVariance = 1;
    public static double[] kJ_TestUseVoltage = new double[]{1,2,3,4,5,6};
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
                double outputVoltage = 0;
                double[] velocity = new double[SwerveDrive.PARAMS.unitNames.length];
                for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                    velocity[i] = 0;
                }

                boolean kS_KV_Started = false;
                boolean kJ_Started = false;
                double UsableTestVoltage = 0;
                double testVoltageIncrement = 0;
                int currentTestPoint = 0;
                int kJ_TestIndex = 0;
                long kJ_lastTime = 0;//nano sec
                double lastVelocityTPS=0;
                while (opModeIsActive()) {
                    double batteryVoltage = swerveDrive.swerveController.voltageSensor.getVoltage();
                    switch(rotationState) {
                        //初始评估kS，防止拟合直线时有部分点在x轴上，形成折线，影响拟合效果
                        case kS_ASSESSING:
                            sleep(5);
                            outputVoltage += 0.005;
                            boolean allMoving = true;
                            for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                                if (Math.abs(swerveDrive.swerveController.wheelUnits[i].getSpeed()) < MathSolver.toInch(50)) {
                                    allMoving = false;
                                }
                            }
                            if (allMoving) {
                                rotationState = RotationState.kS_kV_FITTING;
                                kS = outputVoltage;
                            }
                            break;
                        //x轴截距为kS，斜率倒数为kV
                        case kS_kV_FITTING:
                            //divide (power - kS) to several target power
                            if (!kS_KV_Started) {
                                UsableTestVoltage = batteryVoltage - kS;
                                testVoltageIncrement = UsableTestVoltage / kS_kV_TestPoints;
                                kS_KV_Started = true;
                            }
                            meanFilter_SV.filter(MathSolver.avg(velocity));
                            boolean dataUsable = true;
                            if (meanFilter_SV.getCount()< meanFilter_SV.getWindowSize()){
                                dataUsable = false;
                            }
                            if (meanFilter_SV.getVariance()>kS_kV_TestMaxVariance){
                                dataUsable = false;
                            }
                            if(dataUsable){
                                point2Ds_SV.add(new Point2D(Math.min(outputVoltage,batteryVoltage), meanFilter_SV.getMean()));
                                currentTestPoint++;
                                meanFilter_SV.reset();
                            }
                            if(currentTestPoint > kS_kV_TestPoints){
                                //fit line
                                Line line = MathSolver.fitLine(point2Ds_SV);
                                kV = 1/line.getSlope();
                                kS = line.getXIntercept();
                                rotationState = RotationState.kJ_WAITING;
                            }
                            outputVoltage = kS + testVoltageIncrement * currentTestPoint;
                            break;
                        case kJ_WAITING:
                            //wait for wheel to stop
                            boolean needWaiting = true;
                            while (needWaiting) {
                                boolean allStopped = true;
                                telemetry.addLine("Waiting for wheels to stop...");
                                telemetry.update();
                                for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                                    swerveDrive.swerveController.wheelUnits[i].stop();
                                    if (Math.abs(swerveDrive.swerveController.wheelUnits[i].getSpeed()) > MathSolver.toInch(50)) {
                                        allStopped = false;
                                    }
                                    velocity[i] = swerveDrive.swerveController.wheelUnits[i].getVelocityInTPS();
                                }
                                if (allStopped) {
                                    needWaiting = false;
                                    rotationState = RotationState.kJ_ASSESSING;
                                }
                            }
                            break;
                        case kJ_ASSESSING:
                            if(!kJ_Started){
                                kJ_lastTime = System.nanoTime();
                                kJ_Started = true;
                            }
                            long nowTime = System.nanoTime();
                            double deltaTime = (nowTime - kJ_lastTime)/1e9;//sec
                            double deltaVelocityTPS = MathSolver.avg(velocity)-lastVelocityTPS;
                            lastVelocityTPS = MathSolver.avg(velocity);
                            double acceleration = deltaVelocityTPS/deltaTime;//T P S^2
                            kJ_lastTime = nowTime;
                            testAccelerations.add(acceleration);
                            double speedUpVoltage = kJ_TestUseVoltage[kJ_TestIndex];
                            outputVoltage = kS + kV*MathSolver.avg(velocity) + speedUpVoltage;
                            if(outputVoltage>batteryVoltage){
                                point2Ds_kJ.add(new Point2D(MathSolver.avg(testAccelerations.toArray(new Double[0])),speedUpVoltage));
                                kJ_TestIndex++;
                                kJ_Started = false;
                                rotationState = RotationState.kJ_WAITING;
                                if(kJ_TestIndex>= kJ_TestUseVoltage.length) {
                                    //fit line
                                    Line line = MathSolver.fitLine(point2Ds_kJ);
                                    kJ = 1 / line.getSlope();
                                    rotationState = RotationState.FINISHED;
                                }
                            }
                            break;
                        case FINISHED:
                            outputVoltage = 0;
                            break;
                    }
                    for (int i = 0; i < SwerveDrive.PARAMS.unitNames.length; i++) {
                        swerveDrive.swerveController.wheelUnits[i].setPower(outputVoltage/batteryVoltage);
                        swerveDrive.swerveController.wheelUnits[i].update();
                        velocity[i]= swerveDrive.swerveController.wheelUnits[i].getVelocityInTPS();
                        telemetry.addData("Motor" + SwerveDrive.PARAMS.unitNames[i] + "Voltage", outputVoltage);
                        telemetry.addData("Motor" +SwerveDrive.PARAMS.unitNames[i]+"Velocity", velocity[i]);
                        telemetry.addData("kS", kS);
                        telemetry.addData("kV", kV);
                        telemetry.addData("kJ", kJ);
                    }
                    telemetry.update();
                }
                break;
            case TRANSLATION:
        }
    }
}
