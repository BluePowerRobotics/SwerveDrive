package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.controllers.swerve.locate.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
@TeleOp(name = "IMU_Tester",group = "Test")
public class IMUTester extends LinearOpMode {
    public static String deviceName = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    public static TestMode testMode = TestMode.IMU;

    LazyHardwareMapImu lazyImu;
    IMU imu;
    Localizer localizer;
    public enum TestMode{
        IMU,
        Localizer,
        RobotPosition;
        public TestMode next(){
            return values()[(this.ordinal()+1)%values().length];
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = InstanceTelemetry.init(telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lazyImu = new LazyHardwareMapImu(hardwareMap,deviceName,new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));
        imu = lazyImu.get();
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);
        swerveDrive.swerveController.setAutoLockHeading(false);
        localizer = RobotPosition.getInstance().localizer;
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.aWasReleased()) testMode = testMode.next();
            telemetry.addData("TestMode",testMode);
            telemetry.addLine();
            TelemetryPacket packet = new TelemetryPacket();
            switch (testMode) {
                case IMU:
                    YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                    telemetry.addData("heading", angles.getYaw());
                    telemetry.addData("pitch", angles.getPitch());
                    telemetry.addData("roll", angles.getRoll());
                    if(gamepad1.bWasReleased()) imu.resetYaw();

                    packet.fieldOverlay().setStroke("#3F51B5");
                    Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(0,0, angles.getYaw()));
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    break;
                case Localizer:
                    localizer.update();
                    Pose2d pose = localizer.getPose();
                    telemetry.addData("heading",pose.heading.log());
                    telemetry.addData("x",pose.position.x);
                    telemetry.addData("y",pose.position.y);
                    if(gamepad1.bWasReleased()) localizer.setPose(new Pose2d(0,0,0));

                    packet.fieldOverlay().setStroke("#3F51B5");
                    Drawing.drawRobot(packet.fieldOverlay(), pose);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    break;
                case RobotPosition:
                    Point2D point2D = RobotPosition.getInstance().getData().getPosition(DistanceUnit.INCH);
                    double heading = RobotPosition.getInstance().getData().headingRadian;
                    telemetry.addData("heading",heading);
                    telemetry.addData("x",point2D.getX());
                    telemetry.addData("y",point2D.getY());

                    packet.fieldOverlay().setStroke("#3F51B5");
                    Drawing.drawRobot(packet.fieldOverlay(), MathSolver.toPose2d(point2D, heading));
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    break;
            }
            telemetry.update();
        }
    }
}
