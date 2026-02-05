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
    enum InputMethod{
        SPEED_HEADING,
        POWER_HEADING,
        VECTOR,
        TRANSLATION_ROTATION
    }
    InputMethod inputMethod = InputMethod.SPEED_HEADING;
    private double targetSpeed =0;
    private double targetPower = 0;
    private Point2D targetHeading = new Point2D(0,0);
    private Point2D targetVector = new Point2D(0,0);
    private Point2D targetRotation = new Point2D(0,0), lastRotation = new Point2D(0,0);
    private Point2D targetTranslation = new Point2D(0,0), lastTranslation = new Point2D(0,0);
    @Override
    public Point2D getPosition() {
        return new Point2D(config.wheelPosition);
    }

    @Override
    public void setSpeed(double speed) {
        inputMethod = InputMethod.SPEED_HEADING;
        this.targetSpeed = speed;
    }

    @Override
    public void setHeading(double heading) {
        this.targetHeading = Point2D.fromPolar(heading,1);
    }

    @Override
    public void setPower(double power) {
        inputMethod = InputMethod.POWER_HEADING;
        this.targetPower = power;
    }

    @Override
    public void setVector(Point2D vector) {
        inputMethod = InputMethod.VECTOR;
        this.targetVector = vector;
    }

    @Override
    public void setVector(Point2D translation, Point2D rotation) {
        inputMethod = InputMethod.TRANSLATION_ROTATION;
        this.lastRotation = new Point2D(this.targetRotation);
        this.lastTranslation = new Point2D(this.targetTranslation);
        this.targetRotation = rotation;
        this.targetTranslation = translation;
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
