package org.firstinspires.ftc.teamcode.controllers.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.locate.RobotPosition;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.WheelUnit;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.filter.AngleMeanFilter;

public class SwerveController {
    boolean firstRun = true;
    public SwerveController(Localizer localizer,VoltageSensor voltageSensor, WheelUnit... wheelUnits) {
        robotPosition= RobotPosition.refresh(localizer);
        this.voltageSensor = voltageSensor;
        this.wheelUnits = wheelUnits;
    }


    public static class Params {
        //todo 调整参数
        public double maxV = 0.5; // 最大线速度 (m/s)
        public double maxA = 0.5; // 最大加速度 (m/s²)
        public double maxOmega = Math.PI * 1 / 2; // 最大角速度 (rad/s)
        public double zeroThresholdV = 0.05; // 速度零点阈值 (m/s)
        public double zeroThresholdOmega = Math.toRadians(0.5); // 角速度零点阈值 (rad/s)
    }

    public static Params PARAMS = new Params();
    public RobotPosition robotPosition;
    boolean fullyAutoMode = false;
    boolean useNoHeadMode = false;
    public boolean runningToPoint = false;
    boolean autoLockHeading = true;
    boolean HeadingLockRadianReset = true;
    double HeadingLockRadian;
    public WheelUnit[] wheelUnits;
    private VoltageSensor voltageSensor;
    static double voltage = 12.0;
    public static double getVoltage() {
        return voltage;
    }

    public double getHeadingLockRadian() {
        return HeadingLockRadian;
    }

    public double noHeadModeStartError;
    public static ChassisCalculator chassisCalculator = new ChassisCalculator();

    public void exchangeNoHeadMode() {
        useNoHeadMode = !useNoHeadMode;
    }

    public boolean getUseNoHeadMode() {
        return useNoHeadMode;
    }

    public void setAutoLockHeading(boolean autoLockHeading) {
        this.autoLockHeading = autoLockHeading;
    }

    public void resetNoHeadModeStartError(double Radian) {
        noHeadModeStartError = Radian;
    }

    public void resetNoHeadModeStartError() {
        resetNoHeadModeStartError(0);
    }

    public void setHeadingLockRadian(double headingLockRadian) {
        this.HeadingLockRadian = MathSolver.normalizeAngle(headingLockRadian);
    }

    public void setTargetPoint(Pose2d pose2d) {
        runningToPoint = true;
        HeadingLockRadian = pose2d.heading.log();
        //todo autorun code
    }

    public void resetPosition(Pose2d pose2d) {
        RobotPosition.getInstance().localizer.setPose(pose2d);
    }

    Point2D targetPoint = new Point2D(0, 0);
    double targetRadian = 0;

    public void gamepadInput(double vx, double vy, double omega) {
        voltage = voltageSensor.getVoltage();
        if(firstRun){
            HeadingLockRadian = robotPosition.getData().headingRadian;
            noHeadModeStartError=robotPosition.getData().headingRadian;
            firstRun = false;
        }
        vx = vx * PARAMS.maxV;
        vy = vy * PARAMS.maxV;
        omega = omega * PARAMS.maxOmega;
        if (!fullyAutoMode) {
            if (runningToPoint) {
                if (Math.abs(Math.hypot(vx, vy)) >= PARAMS.zeroThresholdV || Math.abs(omega) >= PARAMS.zeroThresholdOmega) {
                    runningToPoint = false;//打断自动驾驶
                } else {
                    //todo autorun code
//                    if(targetPoint==null){
//                        targetPoint=robotPosition.getData().getPosition(DistanceUnit.MM);
//                    }
//                    if(Double.isNaN(targetRadian)){
//                        if(HeadingLockRadianReset) {
//                            targetRadian = robotPosition.getData().headingRadian;
//                        }else{
//                            targetRadian=HeadingLockRadian;
//                        }
//                    }
//                    wheelSpeeds = chassisCalculator.solveGround(chassisCalculator.calculatePIDXY(targetPoint, robotPosition.getData().getPosition(DistanceUnit.MM)),
//                            chassisCalculator.calculatePIDRadian(targetRadian,robotPosition.getData().headingRadian),robotPosition.getData().headingRadian );
                }
            }
            if (!runningToPoint) {
                if (autoLockHeading) {
                    if (omega != 0) {
                        HeadingLockRadianReset = true;
                    } else {
                        if (HeadingLockRadianReset) {
                            HeadingLockRadianReset = false;
                            chassisCalculator.firstRunRadian = true;
                            HeadingLockRadian = robotPosition.getData().headingRadian;
                        }
                        if (Math.abs(robotPosition.getData().headingRadian - HeadingLockRadian) <= PARAMS.zeroThresholdOmega) {
                            chassisCalculator.pidRadian.reset();
                        }
                        omega = chassisCalculator.calculatePIDRadian(HeadingLockRadian, robotPosition.getData().headingRadian);
                    }
                }
                for (int i = 0, wheelUnitsLength = wheelUnits.length; i < wheelUnitsLength; i++) {
                    WheelUnit wheelUnit = wheelUnits[i];
                    if (useNoHeadMode)
                        chassisCalculator.solveGround(wheelUnit, vx, vy, omega, robotPosition.getData().headingRadian - noHeadModeStartError,i);
                    else
                        chassisCalculator.solveChassis(wheelUnit, vx, vy, omega,i);
                    wheelUnit.update();
                }
            }
        }
    }
}

@Config
class ChassisCalculator {
    public static class Params {
        public double skP = 0.002;//speed k
        public double skI = 0;
        public double skD = 0.00025;
        public double rkP = 0.7;//radian k
        public double rkI = 1.2;
        public double rkD = 0.1;
        public double hkP = 0.7;//speedHeading k
        public double hkI = 0;
        public double hkD = 0.1;
    }

    public static Params PARAMS = new Params();

    PIDController pidSpeed;
    PIDController pidSpeedHeading;
    PIDController pidRadian;

    ChassisCalculator() {
        // 私有构造函数，防止外部实例化
        pidSpeed = new PIDController(PARAMS.skP, PARAMS.skI, PARAMS.skD);
        pidSpeedHeading = new PIDController(PARAMS.hkP, PARAMS.hkI, PARAMS.hkD);
        pidRadian = new PIDController(PARAMS.rkP, PARAMS.rkI, PARAMS.rkD);
    }

    /**
     * 逆运动学公式
     *
     * @param vx    机器人相对于自身的横移速度 (m/s) —— +右
     * @param vy    机器人相对于自身的前进速度 (m/s) —— +前
     * @param omega 机器人旋转角速度 (rad/s) —— +逆时针
     */
    public void solveChassis(WheelUnit wheelUnit,double vx, double vy, double omega,int index) {
        Point2D translation = new Point2D(vx,vy);
        Point2D rotation = Point2D.fromPolar(Math.atan2(wheelUnit.getPosition().getY(),wheelUnit.getPosition().getX())+Math.PI/2,omega*wheelUnit.getPosition().getDistance());
        double heading = Math.atan2(vy-omega*wheelUnit.getPosition().getX(), vx+omega*wheelUnit.getPosition().getY());
        double speed = Math.hypot(vx + omega * wheelUnit.getPosition().getY(), vy - omega * wheelUnit.getPosition().getX());
        wheelUnit.setHeading(heading);
        wheelUnit.setSpeed(speed);
        InstanceTelemetry.getTelemetry().addData(index+"targetHeading",heading);
        InstanceTelemetry.getTelemetry().addData(index+"targetSpeed",speed);
    }

    /**
     * 逆运动学公式（地面坐标系）
     *
     * @param vx            机器人相对于地面的横移速度 (m/s) —— +右
     * @param vy            机器人相对于地面的前进速度 (m/s) —— +前
     * @param omega         机器人旋转角速度 (rad/s) —— +逆时针
     * @param headingRadian 机器人朝向，弧度制
     */
    public void solveGround(WheelUnit wheelUnit,double vx, double vy, double omega, double headingRadian,int index) {

        double vxPro = vx * Math.cos(headingRadian) + vy * Math.sin(headingRadian);
        double vyPro = -vx * Math.sin(headingRadian) + vy * Math.cos(headingRadian);
        solveChassis(wheelUnit,vxPro, vyPro, omega,index);
    }

    public void solveGround(WheelUnit wheelUnit,double[] vxy, double vOmega, double headingRadian,int index) {
        solveGround(wheelUnit,vxy[0], vxy[1], vOmega, headingRadian,index);
    }

    long lastTimeXY = 0;
    boolean firstRunXY = true;
    double thisTimeHeadingRadian = 0;
    AngleMeanFilter meanFilter = new AngleMeanFilter(10);
    Point2D lastCurrent = new Point2D(0, 0);

    public double[] calculatePIDXY(Point2D target, Point2D current) {
        double errorX = target.getX() - current.getX();
        double errorY = target.getY() - current.getY();
        double distance = Math.hypot(errorX, errorY);
        double angleToTarget = Math.atan2(errorY, errorX);
        if (firstRunXY) {
            firstRunXY = false;
            lastTimeXY = System.nanoTime();
            pidSpeed.reset();
            pidSpeedHeading.reset();
            lastCurrent = current;
            meanFilter.reset();
        }
        thisTimeHeadingRadian = meanFilter.filter(Point2D.translate(current, Point2D.rotate(lastCurrent, Math.PI)).getRadian());
        lastCurrent = current;
        pidSpeed.setPID(PARAMS.skP, PARAMS.skI, PARAMS.skD);
        pidSpeedHeading.setPID(PARAMS.hkP, PARAMS.hkI, PARAMS.hkD);
        double headingError = MathSolver.normalizeAngle(angleToTarget - thisTimeHeadingRadian);
        double v = pidSpeed.calculate(distance, 0, (System.nanoTime() - lastTimeXY) / 1e9);
        double heading = pidSpeedHeading.calculate(0, headingError, (System.nanoTime() - lastTimeXY) / 1e9);
        lastTimeXY = System.nanoTime();
        double vx = v * Math.cos(angleToTarget - heading);
        double vy = v * Math.sin(angleToTarget - heading);
        return new double[]{vx, vy};
    }

    long lastTimeRadian = 0;
    boolean firstRunRadian = true;

    public double calculatePIDRadian(double targetRadian, double currentRadian) {
        if (firstRunRadian) {
            firstRunRadian = false;
            lastTimeRadian = System.nanoTime();
            pidRadian.reset();
        }
        double errorRadian = targetRadian - currentRadian;
        // 归一化到[-π, π]
        errorRadian = MathSolver.normalizeAngle(errorRadian);
        pidRadian.setPID(PARAMS.rkP, PARAMS.rkI, PARAMS.rkD);
        double output = pidRadian.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
        lastTimeRadian = System.nanoTime();
        return output;
    }
}