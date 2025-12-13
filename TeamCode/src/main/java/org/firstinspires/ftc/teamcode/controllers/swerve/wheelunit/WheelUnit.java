package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public interface WheelUnit {
    public Point2D getPosition();
    public void setSpeed(double speed);
    public void setHeading(double heading);
    public double getSpeed();
    public double getHeading();
    public void update();
}
