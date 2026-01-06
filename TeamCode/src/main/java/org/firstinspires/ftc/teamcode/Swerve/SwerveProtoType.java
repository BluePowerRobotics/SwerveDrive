package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;

@TeleOp
public class SwerveProtoType extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            swerveDrive.swerveController.gamepadInput(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
            for(int index = 0; index<swerveDrive.swerveController.wheelUnits.length;index++){
                telemetry.addData(index+"Heading",swerveDrive.swerveController.wheelUnits[index].getHeading());
                telemetry.addData(index+"Speed",swerveDrive.swerveController.wheelUnits[index].getSpeed());
            }
            telemetry.update();
        }
    }
}
