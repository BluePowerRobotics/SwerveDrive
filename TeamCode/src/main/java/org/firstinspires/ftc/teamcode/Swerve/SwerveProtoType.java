package org.firstinspires.ftc.teamcode.Swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.controllers.swerve.locate.RobotPosition;

@TeleOp
public class SwerveProtoType extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            swerveDrive.swerveController.gamepadInput(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
            for(int index = 0; index<swerveDrive.swerveController.wheelUnits.length;index++){
                telemetry.addData(index+"Heading",swerveDrive.swerveController.wheelUnits[index].getHeading());
                telemetry.addData(index+"Speed",swerveDrive.swerveController.wheelUnits[index].getSpeed());
            }
            telemetry.addData("x,y", RobotPosition.getInstance().getData().getPosition(DistanceUnit.INCH).toString());
            telemetry.addData("heading", RobotPosition.getInstance().getData().headingRadian);
            telemetry.update();
        }
    }
}
