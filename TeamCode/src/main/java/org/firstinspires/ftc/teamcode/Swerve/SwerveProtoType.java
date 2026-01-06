package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;

@TeleOp
public class SwerveProtoType extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = InstanceTelemetry.init(telemetry);
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            swerveDrive.swerveController.gamepadInput(40*gamepad1.left_stick_x,-40*gamepad1.left_stick_y,-20*gamepad1.right_stick_x);
            for(int index = 0; index<swerveDrive.swerveController.wheelUnits.length;index++){
                telemetry.addData(index+"Heading",swerveDrive.swerveController.wheelUnits[index].getHeading());
                telemetry.addData(index+"Speed",swerveDrive.swerveController.wheelUnits[index].getSpeed());
            }
            telemetry.update();
        }
    }
}
