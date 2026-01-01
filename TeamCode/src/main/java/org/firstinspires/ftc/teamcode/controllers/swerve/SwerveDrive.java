package org.firstinspires.ftc.teamcode.controllers.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.ServoCoaxialWheel;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.ServoCoaxialWheelConfig;
import org.firstinspires.ftc.teamcode.utility.Point2D;
@Config
public class SwerveDrive {
    public static ServoCoaxialWheelConfig frontLeft = new ServoCoaxialWheelConfig(new Point2D(-1,1),1,0);
    public static ServoCoaxialWheelConfig frontRight = new ServoCoaxialWheelConfig(new Point2D(1,1),1,0);
    public static ServoCoaxialWheelConfig backLeft = new ServoCoaxialWheelConfig(new Point2D(-1,-1),1,0);
    public static ServoCoaxialWheelConfig backRight = new ServoCoaxialWheelConfig(new Point2D(1,-1),1,0);
    public SwerveController swerveController;
    public SwerveDrive(HardwareMap hardwareMap){
        swerveController = new SwerveController(hardwareMap,
                new ServoCoaxialWheel(frontLeft,
                        hardwareMap.get(DcMotorEx.class,"frontLeft"),
                        hardwareMap.get(Servo.class,"frontLeft")),
                new ServoCoaxialWheel(frontRight,
                        hardwareMap.get(DcMotorEx.class,"frontRight"),
                        hardwareMap.get(Servo.class,"frontRight")),
                new ServoCoaxialWheel(backLeft,
                        hardwareMap.get(DcMotorEx.class,"backLeft"),
                        hardwareMap.get(Servo.class,"backLeft")),
                new ServoCoaxialWheel(backRight,
                        hardwareMap.get(DcMotorEx.class,"backRight"),
                        hardwareMap.get(Servo.class,"backRight"))
        );
    }

}
