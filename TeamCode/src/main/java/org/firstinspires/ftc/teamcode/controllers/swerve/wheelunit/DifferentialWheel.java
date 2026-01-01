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
    public double getSpeed() {
        return 0;
    }

    @Override
    public double getHeading() {
        return 0;
    }

    @Override
    public void update() {

    }
}
