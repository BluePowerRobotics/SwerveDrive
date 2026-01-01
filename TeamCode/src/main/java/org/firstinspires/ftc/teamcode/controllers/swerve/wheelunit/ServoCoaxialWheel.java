package org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Point2D;

public class ServoCoaxialWheel implements WheelUnit{
    ServoCoaxialWheelConfig config;
    DcMotorEx motor;
    Servo servo;

    @Override
    public Point2D getPosition() {
        return null;
    }

    public ServoCoaxialWheel(ServoCoaxialWheelConfig servoCoaxialWheelConfig, DcMotorEx dcMotor, Servo servo){
        this.config = servoCoaxialWheelConfig;
        this.motor = dcMotor;
        this.servo = servo;
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
