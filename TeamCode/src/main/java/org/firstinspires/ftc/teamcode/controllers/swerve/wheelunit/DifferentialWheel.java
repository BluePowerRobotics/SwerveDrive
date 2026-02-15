package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.controllers.AngleSensor;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveController;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.SVAController;

public class DifferentialWheel implements WheelUnit{
    public static class Params {
        public double sp=1;
        public double si=0;
        public double sd=0;
        public double mp=0.0;
        public double mi=0;
        public double md=0.0;
        public double kS=0;
        public double kV=0;
        public double kA=0;
        public double kM=0;//等效质量
        public double kJ=0;//等效旋转惯量
        public Params() {}
        public Params(double sp, double si, double sd, double mp, double mi, double md, double kS, double kV, double kA,double kM,double kJ) {
            this.sp = sp;
            this.si = si;
            this.sd = sd;
            this.mp = mp;
            this.mi = mi;
            this.md = md;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kM = kM;
            this.kJ = kJ;
        }
    }
    Params PARAMS = new Params();
    DifferentialWheelConfig config;
    DcMotorEx motor1;
    DcMotorEx motor2;
    AngleSensor angleSensor;
    PIDController anguPID;
    PIDController motor1PID;
    SVAController motorSVA;
    public DifferentialWheel (DifferentialWheelConfig config, DcMotorEx motor1, DcMotorEx motor2, AngleSensor angleSensor){
        this.motor1=motor1;
        this.motor2=motor2;
        this.angleSensor=angleSensor;
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.config=config;
        anguPID = new PIDController(PARAMS.sp, PARAMS.si, PARAMS.sd);
        motor1PID = new PIDController(PARAMS.mp, PARAMS.mi, PARAMS.md);
        motorSVA = new SVAController(PARAMS.kS,PARAMS.kV,PARAMS.kA);
    }
    public DifferentialWheel setPARAMS(Params params){
        PARAMS = params;
        return this;
    }
    enum InputMethod{
        SPEED_HEADING,
        POWER_HEADING,
        VECTOR,
        TRANSLATION_ROTATION
    }
    InputMethod inputMethod = InputMethod.SPEED_HEADING;
    private double targetSpeed =0;
    private double targetPower = 0;
    private Point2D targetHeading = new Point2D(0,0);
    private Point2D targetVector = new Point2D(0,0);
    private Point2D targetRotation = new Point2D(0,0), lastRotation = new Point2D(0,0);
    private Point2D targetTranslation = new Point2D(0,0), lastTranslation = new Point2D(0,0);
    @Override
    public Point2D getPosition() {
        return new Point2D(config.wheelPosition);
    }

    @Override
    public void setSpeed(double speed) {
        inputMethod = InputMethod.SPEED_HEADING;
        this.targetSpeed = speed;
    }

    @Override
    public void setHeading(double heading) {
        this.targetHeading = Point2D.fromPolar(heading,1);
    }

    @Override
    public void setPower(double power) {
        inputMethod = InputMethod.POWER_HEADING;
        this.targetPower = power;
    }

    @Override
    public void setVector(Point2D vector) {
        inputMethod = InputMethod.VECTOR;
        this.targetVector = vector;
    }

    @Override
    public void setVector(Point2D translation, Point2D rotation) {
        inputMethod = InputMethod.TRANSLATION_ROTATION;
        this.lastRotation = new Point2D(this.targetRotation);
        this.lastTranslation = new Point2D(this.targetTranslation);
        this.targetRotation = rotation;
        this.targetTranslation = translation;
    }

    @Override
    public double getSpeed() {
        return ((motor1.getVelocity()/28*2*Math.PI)/config.motor1GearRatio/config.motor1ToTurntableTimes-(motor2.getVelocity()/28*2*Math.PI)/config.motor2GearRatio/config.motor2ToTurntableTimes)/config.turntableToWheelTimes*config.wheelDiameter/2;
    }

    @Override
    public double getVelocityInTPS() {
        return Math.abs((motor1.getVelocity()-motor2.getVelocity()/2.0));
    }

    @Override
    public double getHeading() {
        switch (config.angleSenSorDirection) {
            case FORWARD:
                return MathSolver.normalizeAngle(angleSensor.getRadian()-config.zeroDegreeSensorValue);
            case REVERSE:
                return MathSolver.normalizeAngle(-angleSensor.getRadian()+config.zeroDegreeSensorValue);
        }
        return 0;
    }

    public double getAngularVelocity(){
        return ((motor1.getVelocity()/28*2*Math.PI)/config.motor1GearRatio/config.motor1ToTurntableTimes+(motor2.getVelocity()/28*2*Math.PI)/config.motor2GearRatio/config.motor2ToTurntableTimes)/2;
    }
    private long lastUpdateTime = System.nanoTime();
    @Override
    public void update() {
        double K_kA=1,K_kM=0,K_kJ=0;
        switch (inputMethod) {
            case POWER_HEADING:
                anguPID.setPID(PARAMS.sp, PARAMS.si, PARAMS.sd);
                Point2D calculatedTargetHeading_PH;
                double calculatedPower;
                if (targetPower != 0) {
                    Point2D now = Point2D.fromPolar(getHeading(), targetPower);
                    if (Point2D.dot(now, targetHeading) < 0) {
                        calculatedTargetHeading_PH = Point2D.scale(targetHeading, -1);
                    } else {
                        calculatedTargetHeading_PH = new Point2D(targetHeading);
                    }
                    calculatedPower = Point2D.dot(now, targetHeading);
                } else {
                    Point2D now = Point2D.fromPolar(getHeading(), targetPower);
                    if (Point2D.dot(now, targetHeading) < 0) {
                        calculatedTargetHeading_PH = Point2D.scale(Point2D.fromPolar(getPosition().getRadian(), 1), -1);
                    } else {
                        calculatedTargetHeading_PH = Point2D.fromPolar(getPosition().getRadian(), 1);
                    }
                    calculatedPower = targetPower;
                }
                double omega = anguPID.calculate(0, MathSolver.normalizeAngle(calculatedTargetHeading_PH.getRadian() - getHeading()), (System.nanoTime() - lastUpdateTime) / 1e9);
                double power1 = + calculatedPower + omega;
                double power2 = - calculatedPower + omega;
                motor1.setPower(power1);
                motor2.setPower(power2);
                lastUpdateTime=System.nanoTime();
                return;
            case TRANSLATION_ROTATION:
                targetVector = Point2D.translate(targetTranslation,targetRotation);
                K_kA = 0;
                K_kM = 1;
                K_kJ = 1;
            case VECTOR:
                setSpeed(targetVector.getDistance());
                setHeading(targetVector.getRadian());
            case SPEED_HEADING:
                break;
        }
        anguPID.setPID(PARAMS.sp, PARAMS.si, PARAMS.sd);
        motor1PID.setPID(PARAMS.mp, PARAMS.mi, PARAMS.md);
        motorSVA.setSVA(PARAMS.kS, PARAMS.kV, PARAMS.kA);
        double calculatedSpeed;
        Point2D calculatedTargetHeading;
        if (targetSpeed != 0) {
            Point2D now = Point2D.fromPolar(getHeading(), targetSpeed);
            if (Point2D.dot(now, targetHeading) < 0) {
                calculatedTargetHeading = Point2D.scale(targetHeading, -1);
            } else {
                calculatedTargetHeading = new Point2D(targetHeading);
            }
            calculatedSpeed = Point2D.dot(now, targetHeading);
        } else {
            Point2D now = Point2D.fromPolar(getHeading(), targetSpeed);
            if (Point2D.dot(now, targetHeading) < 0) {
                calculatedTargetHeading = Point2D.scale(Point2D.fromPolar(getPosition().getRadian(), 1), -1);
            } else {
                calculatedTargetHeading = Point2D.fromPolar(getPosition().getRadian(), 1);
            }
            calculatedSpeed = 0;
        }

        double targetAngularVelocity = anguPID.calculate(0, MathSolver.normalizeAngle(calculatedTargetHeading.getRadian() - getHeading()), (System.nanoTime() - lastUpdateTime) / 1e9);
        double targetVelocity1 = (targetAngularVelocity + calculatedSpeed * config.turntableToWheelTimes / config.wheelDiameter) * config.motor1GearRatio * config.motor1ToTurntableTimes * 14 / Math.PI;
        double targetVelocity2 = (targetAngularVelocity - calculatedSpeed * config.turntableToWheelTimes / config.wheelDiameter) * config.motor2GearRatio * config.motor2ToTurntableTimes * 14 / Math.PI;
    }
    @Override
    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
    }
}
