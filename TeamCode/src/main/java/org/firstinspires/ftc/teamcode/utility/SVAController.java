package org.firstinspires.ftc.teamcode.utility;

public class SVAController {
    public SVAController(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
    double kS;
    double kV;
    double kA;
    public double calculate(double velocity, double acceleration){
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }
    public void setSVA(double kS, double kV, double kA){
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
}
