package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive.PARAMS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.AngleSensor;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.MathSolver;

@TeleOp
public class AngleSensorTester extends LinearOpMode {
    boolean useGamepad = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        AngleSensor leftFront = new AngleSensor(
                hardwareMap,PARAMS.unitNames[0]+"Analog",0);
//        ),leftBack = new AngleSensor(
//                hardwareMap,PARAMS.unitNames[1]+"Analog",0
//        ),rightFront = new AngleSensor(
//                hardwareMap,PARAMS.unitNames[2]+"Analog",0
//        ),rightBack = new AngleSensor(
//                hardwareMap,PARAMS.unitNames[3]+"Analog",0
//        );

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.aWasReleased())
                useGamepad = !useGamepad;
            if(useGamepad)
                swerveDrive.swerveController.gamepadInput(40*gamepad1.left_stick_x,-40*gamepad1.left_stick_y,-20*gamepad1.right_stick_x);
            else {
                //if(!gamepad1.atRest())
                for (int index = 0; index < swerveDrive.swerveController.wheelUnits.length; index++) {
                    double setHeading = MathSolver.normalizeAngle(swerveDrive.swerveController.wheelUnits[index].getHeading() + gamepad1.left_stick_x);
                    double setSpeed = gamepad1.right_stick_y+0.0001;
                    swerveDrive.swerveController.wheelUnits[index].setHeading(setHeading);
                    swerveDrive.swerveController.wheelUnits[index].setSpeed(setSpeed);
                    swerveDrive.swerveController.wheelUnits[index].update();
                    telemetry.addData(index + "SetHeading", setHeading);
                    telemetry.addData(index + "SetSpeed", setSpeed);
                }
            }
            for(int index = 0; index<swerveDrive.swerveController.wheelUnits.length;index++){
                telemetry.addData(index+"Heading",swerveDrive.swerveController.wheelUnits[index].getHeading());
                telemetry.addData(index+"Speed",swerveDrive.swerveController.wheelUnits[index].getSpeed());
            }
            telemetry.addData("LF Angle", leftFront.getRadian());
//            telemetry.addData("LB Angle", leftBack.getRadian());
//            telemetry.addData("RF Angle", rightFront.getRadian());
//            telemetry.addData("RB Angle", rightBack.getRadian());
            telemetry.addData("LF Voltage", leftFront.getVoltage());
//            telemetry.addData("LB Voltage", leftBack.getVoltage());
//            telemetry.addData("RF Voltage", rightFront.getVoltage());
//            telemetry.addData("RB Voltage", rightBack.getVoltage());
            telemetry.update();
        }
    }
}
