package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;


public class SwerveWheelVelConstraint implements VelConstraint {
    public double maxWheelVel;
    public SwerveWheelVelConstraint(double maxWheelVel){
        this.maxWheelVel=maxWheelVel;
    }
    @Override
    public double maxRobotVel(Pose2dDual<Arclength> robotPose,PosePath path, double s) {
        double robotVel = Math.hypot(robotPose.velocity().value().linearVel.x, robotPose.velocity().value().linearVel.y);
        return maxWheelVel/robotVel;
    }
}
