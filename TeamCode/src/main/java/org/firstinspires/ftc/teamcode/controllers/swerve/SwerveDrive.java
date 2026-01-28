package org.firstinspires.ftc.teamcode.controllers.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.controllers.AngleSensor;
import org.firstinspires.ftc.teamcode.controllers.swerve.locate.Data;
import org.firstinspires.ftc.teamcode.controllers.swerve.locate.RobotPosition;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.ServoCoaxialWheel;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.ServoCoaxialWheelConfig;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.WheelUnit;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;

import java.util.LinkedList;

@Config
public class SwerveDrive {
    public static class Params{
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        public String[] unitNames = {"leftFront","rightFront","leftBack","rightBack"};
    }
    public static Params PARAMS = new Params();

    public static ServoCoaxialWheelConfig leftFront = new ServoCoaxialWheelConfig(new Point2D(-1,1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.REVERSE,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,2.5);
    public static ServoCoaxialWheelConfig rightFront = new ServoCoaxialWheelConfig(new Point2D(1,1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.REVERSE,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,2.5);
    public static ServoCoaxialWheelConfig leftBack = new ServoCoaxialWheelConfig(new Point2D(-1,-1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.REVERSE,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,2.5);
    public static ServoCoaxialWheelConfig rightBack = new ServoCoaxialWheelConfig(new Point2D(1,-1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.REVERSE,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,2.5);
    public static ServoCoaxialWheel.Params leftFrontParams = new ServoCoaxialWheel.Params();
    public static ServoCoaxialWheel.Params rightFrontParams = new ServoCoaxialWheel.Params();
    public static ServoCoaxialWheel.Params leftBackParams = new ServoCoaxialWheel.Params();
    public static ServoCoaxialWheel.Params rightBackParams = new ServoCoaxialWheel.Params();
    public SwerveController swerveController;
    public final LazyImu lazyImu;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    public SwerveDrive(HardwareMap hardwareMap){
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        lazyImu =  new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        swerveController = new SwerveController(
                new DriveLocalizer(Data.getInstance().getPose2d()),
                hardwareMap.voltageSensor.iterator().next(),
                new ServoCoaxialWheel(leftFront,
                        hardwareMap.get(DcMotorEx.class,PARAMS.unitNames[0]),
                        hardwareMap.get(Servo.class,PARAMS.unitNames[0]+"Servo"),
                        new AngleSensor(
                                hardwareMap,PARAMS.unitNames[0]+"Analog",0
                        )).setPARAMS(leftFrontParams),
                new ServoCoaxialWheel(rightFront,
                        hardwareMap.get(DcMotorEx.class,PARAMS.unitNames[1]),
                        hardwareMap.get(Servo.class,PARAMS.unitNames[1]+"Servo"),
                        new AngleSensor(
                                hardwareMap,PARAMS.unitNames[1]+"Analog",0
                        )).setPARAMS(rightFrontParams),
                new ServoCoaxialWheel(leftBack,
                        hardwareMap.get(DcMotorEx.class,PARAMS.unitNames[2]),
                        hardwareMap.get(Servo.class,PARAMS.unitNames[2]+"Servo"),
                        new AngleSensor(
                                hardwareMap,PARAMS.unitNames[2]+"Analog",0
                        )).setPARAMS(leftBackParams),
                new ServoCoaxialWheel(rightBack,
                        hardwareMap.get(DcMotorEx.class,PARAMS.unitNames[3]),
                        hardwareMap.get(Servo.class,PARAMS.unitNames[3]+"Servo"),
                        new AngleSensor(
                                hardwareMap,PARAMS.unitNames[3]+"Analog",0
                        )).setPARAMS(rightBackParams)
        );
    }
    public void setDrivePowers(PoseVelocity2d powers) {
        swerveController.gamepadInput(-powers.linearVel.y, powers.linearVel.x, powers.angVel);
    }
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = RobotPosition.getInstance().localizer.update();
        poseHistory.add(RobotPosition.getInstance().getData().getPose2d());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(RobotPosition.getInstance().getData().getPose2d()));


        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }
    public class DriveLocalizer implements Localizer{
        private Point2D position;
        private double headingRadian;
        private double startRadian;
        public final IMU imu;
        public DriveLocalizer(Pose2d initialPose){
            imu = lazyImu.get();
            position = MathSolver.toPoint2D(initialPose);
            startRadian = initialPose.heading.log();
            headingRadian = startRadian;
            lastUpdateTime = System.nanoTime();
        }
        @Override
        public void setPose(Pose2d pose) {
            position = MathSolver.toPoint2D(pose);
            startRadian = MathSolver.normalizeAngle(pose.heading.log()-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            headingRadian = pose.heading.log();
        }

        @Override
        public Pose2d getPose() {
            return MathSolver.toPose2d(position,headingRadian);
        }
        private long lastUpdateTime;
        @Override
        public PoseVelocity2d update() {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double imuRadian = angles.getYaw(AngleUnit.RADIANS);
            headingRadian = MathSolver.normalizeAngle(imuRadian+startRadian);
            double radian = MathSolver.normalizeAngle(headingRadian+Math.PI/2);
            Point2D total = Point2D.ZERO;
            for(WheelUnit wheelUnit:swerveController.wheelUnits){
                total = Point2D.translate(total,Point2D.fromPolar(wheelUnit.getHeading(),wheelUnit.getSpeed()));
            }
            Point2D avgSpeed = Point2D.scale(total,1.0/swerveController.wheelUnits.length);
            long nowTime = System.nanoTime();
            Point2D avgMove = Point2D.scale(avgSpeed,(nowTime-lastUpdateTime)/1e9);
            lastUpdateTime = nowTime;
            position = Point2D.translate(position, Point2D.rotate(avgMove,radian));
            return new PoseVelocity2d(new Vector2d(avgSpeed.getY(),-avgSpeed.getX()),imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);
        }
    }

}
