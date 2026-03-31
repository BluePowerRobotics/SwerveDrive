package org.firstinspires.ftc.teamcode.VoltageOutTester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.utility.Line;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.VoltageOut;
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import android.os.Environment;

@Config
@TeleOp(name = "Motor SVA Tuning")
public class MotorSVATuning extends LinearOpMode {
    private DcMotorEx motor;
    private FileWriter fileWriter;
    private String logFileName;
    private VoltageOut voltageOut;
    private boolean lastA = false;

    public enum TuningMode {
        AUTOMATIC, MANUAL;
        TuningMode next() {
            return this == AUTOMATIC ? MANUAL : AUTOMATIC;
        }
    }

    enum State {
        kS_ASSESSING,
        kS_kV_FITTING,
        WAITING,
        kA_ASSESSING,
        FINISHED
    }

    State state = State.kS_ASSESSING;
    List<Point2D> points_kS_kV = new ArrayList<>();
    List<Point2D> points_kA = new ArrayList<>();
    //todo para
    MeanFilter meanFilter = new MeanFilter(20);

    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    // Configurable parameters
    public static TuningMode tuningMode = TuningMode.MANUAL;
    public static double KS_VOLTAGE_INCREMENT = 0.001; // Voltage increment for kS assessment
    public static double KS_VELOCITY_THRESHOLD = 20; // Velocity threshold to determine if motor is moving
    public static int testPoints = 5;
    public static double maxVariance = 1;
    public static double[] kA_TestVoltages = new double[]{1, 2, 3, 4, 5, 6};
    public static double voltageIncrement = 0.01; // For manual mode
    public static String motorName = "SVA"; // Configurable motor name

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = InstanceTelemetry.init(telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageOut = new VoltageOut(hardwareMap);

        // Initialize log file
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.CHINA);
        logFileName = Environment.getExternalStorageDirectory().getPath() + "/" + sdf.format(new Date()) + "_MotorSVA.csv";
        try {
            fileWriter = new FileWriter(logFileName);
            fileWriter.write("Time,OutputVoltage,Velocity\n");
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to create log file: " + e.getMessage());
            telemetry.update();
            return;
        }

        while (opModeInInit()) {
            telemetry.addLine("Tuning Mode: " + tuningMode);
            telemetry.addLine("Press A to switch mode");
            if (gamepad1.aWasPressed()) { // Assuming aWasReleased logic, but simplified
                tuningMode = tuningMode.next();
            }
            telemetry.update();
        }

        waitForStart();

        double outputVoltage = 0;
        int currentTestPoint = 0;
        int kA_TestIndex = 0;
        boolean kA_Started = false;
        long lastTime = System.nanoTime();
        double lastVelocity = 0;
        List<Double> accelerations = new ArrayList<>();

        while (opModeIsActive()) {
            double velocity = motor.getVelocity(); // Assuming getVelocity() returns TPS or similar
            long nowTime = System.nanoTime();
            double deltaTime = (nowTime - lastTime) / 1e9;
            double acceleration = (velocity - lastVelocity) / deltaTime;
            lastVelocity = velocity;
            lastTime = nowTime;

            switch (state) {
                case kS_ASSESSING:
                    outputVoltage += KS_VOLTAGE_INCREMENT / 100;//延缓增加，给电流作用的时间
                    if (Math.abs(velocity) > KS_VELOCITY_THRESHOLD) { // Threshold to start moving
                        kS = outputVoltage;
                        state = State.kS_kV_FITTING;
                    }
                    break;
                case kS_kV_FITTING:
                    if (tuningMode == TuningMode.AUTOMATIC) {
                        if (currentTestPoint < testPoints) {
                            meanFilter.filter(velocity);
                            if (meanFilter.getCount() >= meanFilter.getWindowSize() && meanFilter.getVariance() <= maxVariance) {
                                points_kS_kV.add(new Point2D(outputVoltage, meanFilter.getMean()));
                                currentTestPoint++;
                                meanFilter.reset();
                                outputVoltage += (voltageOut.getVoltage() - kS) / testPoints;
                            }
                        } else {
                            Line line = MathSolver.fitLine(points_kS_kV);
                            kV = 1 / line.getSlope();
                            kS = line.getXIntercept();
                            state = State.WAITING;
                        }
                    } else { // MANUAL
                        if (gamepad1.dpadUpWasPressed()) {
                            outputVoltage += voltageIncrement;
                        } else if (gamepad1.dpadDownWasPressed()) {
                            outputVoltage -= voltageIncrement;
                        }
                        if(gamepad1.dpadLeftWasPressed()){
                            outputVoltage -= KS_VOLTAGE_INCREMENT;
                        }
                        if(gamepad1.dpadRightWasPressed()){
                            outputVoltage += KS_VOLTAGE_INCREMENT;
                        }
                        if (gamepad1.a) { // Record point
                            points_kS_kV.add(new Point2D(outputVoltage, velocity));
                            currentTestPoint++;
                            if (currentTestPoint >= testPoints) {
                                Line line = MathSolver.fitLine(points_kS_kV);
                                kV = 1 / line.getSlope();
                                kS = line.getXIntercept();
                                //state = State.WAITING;
                            }
                        }
                        if(gamepad1.start){
                            state = State.WAITING;
                        }
                    }
                    break;
                case WAITING:
                    motor.setPower(0);
                    if (Math.abs(velocity) < 10) {
                        state = State.kA_ASSESSING;
                    }
                    break;
                case kA_ASSESSING:
                    if (!kA_Started) {
                        accelerations.clear();
                        lastTime = System.nanoTime();
                        lastVelocity = velocity;
                        kA_Started = true;
                    }
                    accelerations.add(acceleration);
                    double speedUpVoltage = kA_TestVoltages[kA_TestIndex];
                    outputVoltage = kS + kV * velocity + speedUpVoltage;
                    if (outputVoltage > voltageOut.getVoltage()) {
                        double avgAccel = MathSolver.avg(accelerations.toArray(new Double[0]));
                        points_kA.add(new Point2D(avgAccel, speedUpVoltage));
                        kA_TestIndex++;
                        kA_Started = false;
                        if (kA_TestIndex >= kA_TestVoltages.length) {
                            Line line = MathSolver.fitLine(points_kA);
                            kA = 1 / line.getSlope();
                            state = State.FINISHED;
                        }
                    }
                    break;
                case FINISHED:
                    outputVoltage = 0;
                    break;
            }
            motor.setPower(voltageOut.getVoltageOutPower(outputVoltage));

            // Telemetry
            telemetry.addData("State", state);
            telemetry.addData("Output Voltage", outputVoltage);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("kS", kS);
            telemetry.addData("kV", kV);
            telemetry.addData("kA", kA);
            telemetry.update();

            // Log to file
            try {
                fileWriter.write(System.currentTimeMillis() + "," + outputVoltage + "," + velocity + "\n");
            } catch (IOException e) {
                telemetry.addData("Log Error", e.getMessage());
            }

            sleep(10); // Small delay
        }

        // Close file
        try {
            fileWriter.close();
        } catch (IOException e) {
            telemetry.addData("Close Error", e.getMessage());
        }
    }
}

// Note: Point2D class assumed to be available from utility package

