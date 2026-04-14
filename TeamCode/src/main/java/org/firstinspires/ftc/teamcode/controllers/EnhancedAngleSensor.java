package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.LinearInterpolationTable;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class EnhancedAngleSensor extends AngleSensor{
    Point2D[] point2DS;
    LinearInterpolationTable interpolationTable;
    public EnhancedAngleSensor(HardwareMap hardwareMap, String deviceName, double ZeroDegreeVoltage, Point2D[] point2DS) {
        super(hardwareMap, deviceName, ZeroDegreeVoltage);
        this.point2DS = point2DS;
        interpolationTable = new LinearInterpolationTable(this.point2DS);
    }
    public EnhancedAngleSensor(HardwareMap hardwareMap, String deviceName, double ZeroDegreeVoltage) {
        this(hardwareMap, deviceName, ZeroDegreeVoltage, new Point2D[]{
                new Point2D(0, 0),
                new Point2D(2 * Math.PI, 2 * Math.PI)
        });
    }
    @Override
    public double getRadian() {
        double angle = super.getRadian();
        angle = interpolationTable.getOutput(angle);
        return angle;
    }
}
