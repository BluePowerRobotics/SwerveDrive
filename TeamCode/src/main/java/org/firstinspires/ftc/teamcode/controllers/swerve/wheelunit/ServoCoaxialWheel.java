package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.AngleSensor;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveController;
import org.firstinspires.ftc.teamcode.utility.SVAController;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;
@Config
public class ServoCoaxialWheel implements WheelUnit{
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
        public Params(){}
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
    ServoCoaxialWheelConfig config;
    DcMotorEx motor;
    Servo servo;
    AngleSensor angleSensor;
    MeanFilter angularVelocityFilter = new MeanFilter(5);
    PIDController servoPID;
    PIDController motorPID;
    SVAController motorSVA;
    enum InputMethod{
        SPEED_HEADING,
        POWER_HEADING,
        VECTOR,
        TRANSLATION_ROTATION
    }
    InputMethod inputMethod = InputMethod.SPEED_HEADING;
    public ServoCoaxialWheel setPARAMS(Params params){
        PARAMS = params;
        return this;
    }

    @Override
    public Point2D getPosition() {
        return config.wheelPosition;
    }

    public ServoCoaxialWheel(ServoCoaxialWheelConfig servoCoaxialWheelConfig, DcMotorEx dcMotor, Servo servo, AngleSensor angleSensor){
        this.config = servoCoaxialWheelConfig;
        this.motor = dcMotor;
        this.servo = servo;
        this.angleSensor=angleSensor;
        this.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.servo.setDirection(config.servoDirection);
        lastRadian = config.zeroDegreeSensorValue;
        lastTime = System.nanoTime();
        targetHeading = Point2D.fromPolar(config.wheelPosition.getRadian(), 1);
        servoPID = new PIDController(PARAMS.sp, PARAMS.si, PARAMS.sd);
        motorPID = new PIDController(PARAMS.mp, PARAMS.mi, PARAMS.md);
        motorSVA = new SVAController(PARAMS.kS,PARAMS.kV,PARAMS.kA);
    }
    private double targetSpeed=0;
    @Override
    public void setSpeed(double speed) {
        inputMethod = InputMethod.SPEED_HEADING;
        targetSpeed = speed;
    }
    private Point2D targetHeading;

    @Override
    public void setHeading(double heading) {
        targetHeading = Point2D.fromPolar(heading,1);
    }
    private double targetPower=0;
    @Override
    public void setPower(double power) {
        inputMethod = InputMethod.POWER_HEADING;
        targetPower = power;
    }
    private Point2D targetVector = new Point2D(0,0);
    @Override
    public void setVector(Point2D vector) {
        inputMethod = InputMethod.VECTOR;
        targetVector = vector;
    }
    private Point2D targetTranslation = new Point2D(0,0),targetRotation = new Point2D(0,0);
    @Override
    public void setVector(Point2D translation, Point2D rotation) {
        inputMethod = InputMethod.TRANSLATION_ROTATION;
        targetTranslation = translation;
        targetRotation = rotation;
    }

    private double lastRadian;
    private long lastTime;

    public double getAngularVelocity(){
        double nowRadian = angleSensor.getRadian();
        long nowTime = System.nanoTime();
        double angularVelocity = MathSolver.normalizeAngle(nowRadian-lastRadian)/((nowTime-lastTime)/1e9);
        lastTime = nowTime;
        lastRadian = nowRadian;
        return angularVelocityFilter.filter(angularVelocity);
    }

    @Override
    public double getSpeed() {
        return (motor.getVelocity()/(28.0/* tick / cycle *//(2*Math.PI))/config.motorGearRatio/config.motorToTurntableTimes-getAngularVelocity())/config.turntableToWheelTimes*config.wheelDiameter/2;
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
    private long lastUpdateTime = System.nanoTime();
    @Override
    public void update() {
        double K_kA=1,K_kM=0,K_kJ=0;
        switch (inputMethod) {
            case POWER_HEADING:
                servoPID.setPID(PARAMS.sp, PARAMS.si, PARAMS.sd);
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
                servo.setPosition(0.5 + servoPID.calculate(0, MathSolver.normalizeAngle(calculatedTargetHeading_PH.getRadian() - getHeading()), (System.nanoTime() - lastUpdateTime) / 1e9));
                motor.setPower(calculatedPower);
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
        servoPID.setPID(PARAMS.sp, PARAMS.si, PARAMS.sd);
        motorPID.setPID(PARAMS.mp, PARAMS.mi, PARAMS.md);
        motorSVA.setSVA(PARAMS.kS, PARAMS.kV, PARAMS.kA);
        double motorVelocity;
        Point2D calculatedTargetHeading;
        if (targetSpeed != 0) {
            Point2D now = Point2D.fromPolar(getHeading(), targetSpeed);
            if (Point2D.dot(now, targetHeading) < 0) {
                calculatedTargetHeading = Point2D.scale(targetHeading, -1);
            } else {
                calculatedTargetHeading = new Point2D(targetHeading);
            }
            motorVelocity = (Point2D.dot(now, targetHeading) / config.wheelDiameter * 2 * config.turntableToWheelTimes + getAngularVelocity()) * config.motorToTurntableTimes * config.motorGearRatio * (28.0/* tick / cycle */ / (2 * Math.PI));
        } else {
            Point2D now = Point2D.fromPolar(getHeading(), targetSpeed);
            if (Point2D.dot(now, targetHeading) < 0) {
                calculatedTargetHeading = Point2D.scale(Point2D.fromPolar(getPosition().getRadian(), 1), -1);
            } else {
                calculatedTargetHeading = Point2D.fromPolar(getPosition().getRadian(), 1);
            }
            motorVelocity = getAngularVelocity() * config.motorToTurntableTimes * config.motorGearRatio * (28.0/* tick / cycle */ / (2 * Math.PI));
        }
        servo.setPosition(0.5 + servoPID.calculate(0, MathSolver.normalizeAngle(calculatedTargetHeading.getRadian() - getHeading()), (System.nanoTime() - lastUpdateTime) / 1e9));
        double pidPower = motorPID.calculate(motorVelocity, motor.getVelocity(), (System.nanoTime() - lastUpdateTime) / 1e9);
        double svaPower = motorSVA.calculate(motorVelocity, K_kA*(motorVelocity - motor.getVelocity()) / ((System.nanoTime() - lastUpdateTime) / 1e9));
        double rotationVelocity = Point2D.dot(Point2D.fromPolar(getHeading(), motor.getVelocity()), Point2D.fromPolar(getPosition().getRadian() + Math.PI / 2, 1))/* /1 */;
        double rotationAcceleration = (targetRotation.getDistance()-rotationVelocity) / ((System.nanoTime() - lastUpdateTime) / 1e9);
        svaPower += K_kJ * PARAMS.kJ * rotationAcceleration;
        Point2D rotationVector = Point2D.fromPolar(getHeading() + Math.PI / 2, rotationVelocity);
        double translationVelocity = Point2D.translate(Point2D.fromPolar(getHeading(), motor.getVelocity()), Point2D.rotate(rotationVector, Math.PI)).getDistance();
        double translationAcceleration = (targetTranslation.getDistance()-translationVelocity) / ((System.nanoTime() - lastUpdateTime) / 1e9);
        svaPower += K_kM * PARAMS.kM * translationAcceleration;
        motor.setPower((svaPower + pidPower) / SwerveController.getVoltage());
        lastUpdateTime=System.nanoTime();
    }
}
