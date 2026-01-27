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
    }
    public static Params PARAMS = new Params();
    ServoCoaxialWheelConfig config;
    DcMotorEx motor;
    Servo servo;
    AngleSensor angleSensor;
    MeanFilter angularVelocityFilter = new MeanFilter(5);
    PIDController servoPID;
    PIDController motorPID;
    SVAController motorSVA;

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
        targetSpeed = speed;
    }
    private Point2D targetHeading;

    @Override
    public void setHeading(double heading) {
        targetHeading = Point2D.fromPolar(heading,1);
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
        servoPID.setPID(PARAMS.sp, PARAMS.si, PARAMS.sd);
        motorPID.setPID(PARAMS.mp, PARAMS.mi, PARAMS.md);
        motorSVA.setSVA(PARAMS.kS, PARAMS.kV, PARAMS.kA);
        double motorVelocity;
        Point2D calculatedTargetHeading;
        if(targetSpeed!=0) {
            Point2D now = Point2D.fromPolar(getHeading(), targetSpeed);
            if(Point2D.dot(now,targetHeading)<0){
                calculatedTargetHeading=Point2D.scale(targetHeading,-1);
            }else{
                calculatedTargetHeading = new Point2D(targetHeading);
            }
            motorVelocity = (Point2D.dot(now, targetHeading)/config.wheelDiameter*2*config.turntableToWheelTimes+getAngularVelocity())*config.motorToTurntableTimes*config.motorGearRatio*(28.0/* tick / cycle *//(2*Math.PI));
        }else{
            Point2D now = Point2D.fromPolar(getHeading(), targetSpeed);
            if(Point2D.dot(now,targetHeading)<0){
                calculatedTargetHeading = Point2D.scale(Point2D.fromPolar(getPosition().getRadian(),1),-1);
            }else{
                calculatedTargetHeading = Point2D.fromPolar(getPosition().getRadian(),1);
            }
            motorVelocity = getAngularVelocity()*config.motorToTurntableTimes*config.motorGearRatio*(28.0/* tick / cycle *//(2*Math.PI));
        }
        servo.setPosition(0.5+servoPID.calculate   (0,MathSolver.normalizeAngle(calculatedTargetHeading.getRadian()-getHeading()), (System.nanoTime() - lastUpdateTime)/1e9));
        double pidPower = motorPID.calculate(motorVelocity, motor.getVelocity(), (System.nanoTime() - lastUpdateTime)/1e9);
        double svaPower = motorSVA.calculate(motorVelocity, (motorVelocity-motor.getVelocity())/((System.nanoTime() - lastUpdateTime)/1e9));
        motor.setPower((svaPower + pidPower)/ SwerveController.getVoltage());
        lastUpdateTime=System.nanoTime();
    }
}
