package org.firstinspires.ftc.teamcode.controllers.swerve.locate;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.controllers.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
@Config
public class RobotPosition {
    public static class Params {
        static double inPerTick = 0.001999;
        static int minUpdateIntervalMs = 1; // 最小更新时间间隔，单位毫秒

    }
    public void setMinUpdateIntervalMs(@NonNull int interval){
        Params.minUpdateIntervalMs=interval;
    }

    private static RobotPosition instance;
    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }
    /**
    * 初始化位置
    * @param hardwareMap 硬件映射
     * @param localizer 定位器
    * @return RobotPosition实例
     */
    public static RobotPosition refresh(HardwareMap hardwareMap,Localizer localizer){
        Point2D initialPosition=MathSolver.toPoint2D(localizer.getPose());
        double initialHeadingRadian=localizer.getPose().heading.toDouble();
        instance=new RobotPosition(hardwareMap);
        instance.initialPosition=initialPosition;
        instance.initialHeadingRadian=initialHeadingRadian;
        Data.instance.setPosition(initialPosition);
        Data.instance.headingRadian=initialHeadingRadian;
        instance.localizer = localizer;
        return instance;
    }

    private RobotPosition(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
        }

    private HardwareMap hardwareMap;
    public Localizer localizer;
    public Point2D initialPosition=new Point2D(0,0);
    public double initialHeadingRadian=0;

    public long lastUpdateTime=0;

    public void update(){
        if(System.currentTimeMillis()-lastUpdateTime<Params.minUpdateIntervalMs){
            return;
        }
        PoseVelocity2d poseVelocity2d = localizer.update();
        Pose2d pose = localizer.getPose();
        Data.instance.headingSpeedRadianPerSec=poseVelocity2d.angVel;
        Data.instance.setSpeed(new Vector2d(-poseVelocity2d.linearVel.y,+poseVelocity2d.linearVel.x));
        Data.instance.setPosition(new Point2D(-pose.position.y,+pose.position.x));
        Data.instance.headingRadian=pose.heading.log();
        lastUpdateTime=System.currentTimeMillis();
    }
    public Data getData(){
        update();
        return Data.instance;
    }
}
