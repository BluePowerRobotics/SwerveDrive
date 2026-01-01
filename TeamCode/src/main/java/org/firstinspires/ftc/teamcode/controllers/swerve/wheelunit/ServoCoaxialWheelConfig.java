package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public class ServoCoaxialWheelConfig {
    public double zeroDegreeServoPosition;
    public double motorToWheelTimes;
    public Servo.Direction servoDirection;
    Point2D wheelPosition;
    public ServoCoaxialWheelConfig(Point2D wheelPosition, double motorToWheelTimes, double zeroDegreeServoPosition ,Servo.Direction servoDirection){
        this.zeroDegreeServoPosition=zeroDegreeServoPosition;
        this.motorToWheelTimes=motorToWheelTimes;
        this.wheelPosition=wheelPosition;
        this.servoDirection = servoDirection;
    }
}
