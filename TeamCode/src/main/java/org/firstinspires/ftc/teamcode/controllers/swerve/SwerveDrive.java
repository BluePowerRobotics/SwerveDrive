package org.firstinspires.ftc.teamcode.controllers.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.AngleSensor;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.ServoCoaxialWheel;
import org.firstinspires.ftc.teamcode.controllers.swerve.wheelunit.ServoCoaxialWheelConfig;
import org.firstinspires.ftc.teamcode.utility.Point2D;
@Config
public class SwerveDrive {
    public static ServoCoaxialWheelConfig leftFront = new ServoCoaxialWheelConfig(new Point2D(-1,1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.FORWARD,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,63.5);
    public static ServoCoaxialWheelConfig rightFront = new ServoCoaxialWheelConfig(new Point2D(1,1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.FORWARD,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,63.5);
    public static ServoCoaxialWheelConfig leftBack = new ServoCoaxialWheelConfig(new Point2D(-1,-1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.FORWARD,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,63.5);
    public static ServoCoaxialWheelConfig rightBack = new ServoCoaxialWheelConfig(new Point2D(1,-1),
            0, Servo.Direction.FORWARD, ServoCoaxialWheelConfig.AngleSenSorDirection.FORWARD,
            2.89*3.61/* *5.23 */,80.0/72.0,60.0/18.0,63.5);
    public SwerveController swerveController;
    public SwerveDrive(HardwareMap hardwareMap){
        swerveController = new SwerveController(hardwareMap,
                new ServoCoaxialWheel(leftFront,
                        hardwareMap.get(DcMotorEx.class,"leftFront"),
                        hardwareMap.get(Servo.class,"leftFront"),
                        new AngleSensor(
                                hardwareMap,"leftFront",0
                        )),
                new ServoCoaxialWheel(rightFront,
                        hardwareMap.get(DcMotorEx.class,"rightFront"),
                        hardwareMap.get(Servo.class,"rightFront"),
                        new AngleSensor(
                                hardwareMap,"rightFront",0
                        )),
                new ServoCoaxialWheel(leftBack,
                        hardwareMap.get(DcMotorEx.class,"leftBack"),
                        hardwareMap.get(Servo.class,"leftBack"),
                        new AngleSensor(
                                hardwareMap,"leftBack",0
                        )),
                new ServoCoaxialWheel(rightBack,
                        hardwareMap.get(DcMotorEx.class,"rightBack"),
                        hardwareMap.get(Servo.class,"rightBack"),
                        new AngleSensor(
                                hardwareMap,"rightBack",0
                        ))
        );
    }

}
