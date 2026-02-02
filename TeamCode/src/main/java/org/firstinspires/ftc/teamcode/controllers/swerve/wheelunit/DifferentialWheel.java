package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public class DifferentialWheel implements WheelUnit{
    DifferentialWheelConfig config;
    DcMotorEx motor1;
    DcMotorEx motor2;
    public DifferentialWheel (DifferentialWheelConfig config,DcMotorEx motor1, DcMotorEx motor2){
        this.motor1=motor1;
        this.motor2=motor2;
        this.config=config;
    }
    @Override
    public Point2D getPosition() {
        return null;
    }

    @Override
    public void setSpeed(double speed) {

    }

    @Override
    public void setHeading(double heading) {

    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public void setVector(Point2D vector) {

    }

    @Override
    public void setVector(Point2D translation, Point2D rotation) {

    }

    @Override
    public double getSpeed() {
        return 0;
    }

    @Override
    public double getVelocityInTPS() {
        return 0;
    }

    @Override
    public double getHeading() {
        return 0;
    }

    @Override
    public void update() {

    }
    @Override
    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
    }
}
