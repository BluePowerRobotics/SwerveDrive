package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.AngleSensor;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;
@Config
public class ServoCoaxialWheel implements WheelUnit{
    public static class Params {
        public double sp=1;
        public double si=0;
        public double sd=0.1;
    }
    public static Params PARAMS = new Params();
    ServoCoaxialWheelConfig config;
    DcMotorEx motor;
    Servo servo;
    AngleSensor angleSensor;
    MeanFilter angularVelocityFilter = new MeanFilter(5);
    PIDController servoPID;

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
        double motorVelocity;
        if(targetSpeed!=0) {
            Point2D now = Point2D.fromPolar(getHeading(), targetSpeed);
            if(Point2D.dot(now,targetHeading)<0){
                targetHeading=Point2D.scale(targetHeading,-1);
            }
            Point2D target = Point2D.scale(targetHeading, Point2D.dot(now, targetHeading));
            motorVelocity = (target.getDistance()/config.wheelDiameter*2*config.turntableToWheelTimes+getAngularVelocity())*config.motorToTurntableTimes*config.motorGearRatio*(28.0/* tick / cycle *//(2*Math.PI));
        }else{
            motorVelocity = getAngularVelocity()*config.motorToTurntableTimes*config.motorGearRatio*(28.0/* tick / cycle *//(2*Math.PI));
        }
        servo.setPosition(servoPID.calculate(0,MathSolver.normalizeAngle(targetHeading.getRadian()-getHeading()), (System.nanoTime() - lastUpdateTime)/1e9));
        motor.setVelocity(motorVelocity);
        lastUpdateTime=System.nanoTime();
    }
}
