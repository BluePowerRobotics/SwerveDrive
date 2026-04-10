package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public class ServoCoaxialWheelConfig {
    public double motorGearRatio;
    public double motorToTurntableTimes;
    public double turntableToWheelTimes;
    public double wheelDiameter;
    public double zeroDegreeSensorRadian;
    public Servo.Direction servoDirection;
    public enum AngleSenSorDirection{FORWARD, REVERSE}
    public AngleSenSorDirection angleSenSorDirection;
    public Point2D wheelPosition;
    public ServoCoaxialWheelConfig(Point2D wheelPosition, double zeroDegreeSensorRadian, Servo.Direction servoDirection, AngleSenSorDirection angleSenSorDirection,
                                   double motorGearRatio, double motorToTurntableTimes, double turntableToWheelTimes, double wheelDiameter){
        this.zeroDegreeSensorRadian = zeroDegreeSensorRadian;
        this.wheelPosition=wheelPosition;
        this.servoDirection = servoDirection;
        this.angleSenSorDirection = angleSenSorDirection;
        this.motorGearRatio=motorGearRatio;
        this.motorToTurntableTimes=motorToTurntableTimes;
        this.turntableToWheelTimes=turntableToWheelTimes;
        this.wheelDiameter=wheelDiameter;
    }
}
