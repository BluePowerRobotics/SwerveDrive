package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.LinearInterpolationTable;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class EnhancedAngleSensor extends SimpleAngleSensor implements AngleSensor{
    Point2D[] point2DS;
    LinearInterpolationTable interpolationTable;
    double max = Double.NEGATIVE_INFINITY;
    double min = Double.POSITIVE_INFINITY;
    public EnhancedAngleSensor(HardwareMap hardwareMap, String deviceName, double ZeroDegreeVoltage, Point2D[] point2DS) {
        super(hardwareMap, deviceName, ZeroDegreeVoltage);
        this.point2DS = point2DS;
        interpolationTable = new LinearInterpolationTable(this.point2DS);
        max = MathSolver.max(interpolationTable.getY());
        min = MathSolver.min(interpolationTable.getY());
    }
    public EnhancedAngleSensor(HardwareMap hardwareMap, String deviceName, double ZeroDegreeVoltage) {
        this(hardwareMap, deviceName, ZeroDegreeVoltage, new Point2D[]{
                new Point2D(0,-Math.PI),
                new Point2D(3.3,Math.PI)
        });
    }
    @Override
    public double getRadian() {
        double voltage = super.getVoltage();
        return interpolationTable.getOutput(voltage);
    }
    public double getMaxOutputRadian(){
        return max;
    }
    public double getMinOutputRadian(){
        return min;
    }
}
