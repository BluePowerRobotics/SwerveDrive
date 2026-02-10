package org.firstinspires.ftc.teamcode.controllers.swerve;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
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
import org.firstinspires.ftc.teamcode.utility.SwerveWheelVelConstraint;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class SwerveDrive {
    public static class Params{
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        public String[] unitNames = {"leftFront","rightFront","leftBack","rightBack"};

        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn

    }
    public static Params PARAMS = new Params();


    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    new SwerveWheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

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
    private VoltageSensor voltageSensor;
    public SwerveDrive(HardwareMap hardwareMap, Pose2d initialPose){
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        lazyImu =  new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        swerveController = new SwerveController(
                this,
                new DriveLocalizer(initialPose),
                () -> voltageSensor.getVoltage(),
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
    public SwerveDrive(HardwareMap hardwareMap){
        this(hardwareMap,Data.getInstance().getPose2d());
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
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                swerveController.autoInput(0,0,0);
                boolean allStopped = true;
                for (WheelUnit wheelUnit : swerveController.wheelUnits) {
                    if (Math.abs(wheelUnit.getHeading()-wheelUnit.getPosition().getRadian()) > 0.05) {
                        allStopped = false;
                    }
                }
                if(allStopped){
                    for (WheelUnit wheelUnit : swerveController.wheelUnits) {
                        wheelUnit.stop();
                    }
                }
                return !allStopped;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            //targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, RobotPosition.getInstance().getData().getPose2d(), robotVelRobot);
            //driveCommandWriter.write(new DriveCommandMessage(command));




            swerveController.autoInput(-command.linearVel.y.get(0), command.linearVel.x.get(0), command.angVel.get(0));

            p.put("x", RobotPosition.getInstance().getData().getPose2d().position.x);
            p.put("y", RobotPosition.getInstance().getData().getPose2d().position.y);
            p.put("heading (deg)", Math.toDegrees(RobotPosition.getInstance().getData().getPose2d().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(RobotPosition.getInstance().getData().getPose2d());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, RobotPosition.getInstance().getData().getPose2d());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                swerveController.autoInput(0,0,0);
                boolean allStopped = true;
                for (WheelUnit wheelUnit : swerveController.wheelUnits) {
                    if (Math.abs(wheelUnit.getHeading()-wheelUnit.getPosition().getRadian()) > 0.05) {
                        allStopped = false;
                    }
                }
                if(allStopped){
                    for (WheelUnit wheelUnit : swerveController.wheelUnits) {
                        wheelUnit.stop();
                    }
                }
                return !allStopped;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, RobotPosition.getInstance().getData().getPose2d(), robotVelRobot);

            swerveController.autoInput(-command.linearVel.y.get(0), command.linearVel.x.get(0), command.angVel.get(0));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, RobotPosition.getInstance().getData().getPose2d());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}
