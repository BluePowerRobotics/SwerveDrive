package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public class DifferentialWheelConfig {
    public double motor1GearRatio;
    public double motor2GearRatio;
    public double motor1ToTurntableTimes;
    public double motor2ToTurntableTimes;
    public double turntableToWheelTimes;
    public double wheelDiameter;
    public double zeroDegreeSensorValue;
    public enum AngleSenSorDirection{FORWARD, REVERSE}
    public AngleSenSorDirection angleSenSorDirection;
    public Point2D wheelPosition;
    public DifferentialWheelConfig(Point2D wheelPosition, double zeroDegreeSensorValue, AngleSenSorDirection angleSenSorDirection,
                                   double motor1GearRatio, double motor2GearRatio,
                                   double motor1ToTurntableTimes, double motor2ToTurntableTimes,
                                   double turntableToWheelTimes, double wheelDiameter){
        this.zeroDegreeSensorValue = zeroDegreeSensorValue;
        this.wheelPosition=wheelPosition;
        this.angleSenSorDirection = angleSenSorDirection;
        this.motor1GearRatio=motor1GearRatio;
        this.motor2GearRatio=motor2GearRatio;
        this.motor1ToTurntableTimes=motor1ToTurntableTimes;
        this.motor2ToTurntableTimes=motor2ToTurntableTimes;
        this.turntableToWheelTimes=turntableToWheelTimes;
        this.wheelDiameter=wheelDiameter;
    }
}
