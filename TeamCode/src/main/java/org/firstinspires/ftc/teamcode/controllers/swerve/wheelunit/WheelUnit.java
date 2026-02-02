package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public interface WheelUnit {
    public Point2D getPosition();
    public void setSpeed(double speed);
    public void setHeading(double heading);
    public void setPower(double power);
    public void setVector(Point2D vector);
    public void setVector(Point2D translation, Point2D rotation);
    public double getSpeed();
    public double getVelocityInTPS();
    public double getHeading();
    public void update();
    public void stop();
}
