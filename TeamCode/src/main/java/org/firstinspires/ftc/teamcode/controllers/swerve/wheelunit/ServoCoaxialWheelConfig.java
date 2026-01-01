package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public class ServoCoaxialWheelConfig {
    public double zeroDegreeServoPosition;
    public double motorToWheelTimes;
    Point2D wheelPosition;
    public ServoCoaxialWheelConfig(Point2D wheelPosition, double motorToWheelTimes, double zeroDegreeServoPosition){
        this.zeroDegreeServoPosition=zeroDegreeServoPosition;
        this.motorToWheelTimes=motorToWheelTimes;
        this.wheelPosition=wheelPosition;
    }
}
