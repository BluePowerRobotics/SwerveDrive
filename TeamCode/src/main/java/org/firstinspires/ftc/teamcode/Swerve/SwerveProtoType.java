package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SwerveProtoType extends LinearOpMode {
    Servo servo;
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.FORWARD);
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        double power = 0;
        while(opModeIsActive()){
            telemetry.addData("power", power);
            motor.setPower(power);
            telemetry.addData("degree", servo.getPosition());
            servo.setPosition((gamepad1.right_stick_x + 1) / 2);
            telemetry.update();
            if(Math.abs(gamepad1.left_stick_x) > 0.1){
                power = gamepad1.left_stick_x;
            }
            if(gamepad1.aWasPressed()){
                power += 0.1;
            }
            if(gamepad1.bWasPressed()){
                power -= 0.1;
            }
        }
    }
}
