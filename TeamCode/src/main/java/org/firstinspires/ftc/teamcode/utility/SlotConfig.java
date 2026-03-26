package org.firstinspires.ftc.teamcode.utility;

public class  SlotConfig {
    public double kP = 0, kI = 0, kD = 0, maxI = 1;
    public double kS = 0, kV = 0, kA = 0;
    public double outputMin = -1, outputMax = 1;

    public SlotConfig withKP(double kP) { this.kP = kP; return this; }
    public SlotConfig withKI(double kI) { this.kI = kI; return this; }
    public SlotConfig withKD(double kD) { this.kD = kD; return this; }
    public SlotConfig withMaxI(double maxI) { this.maxI = maxI; return this; }
    public SlotConfig withKS(double kS) { this.kS = kS; return this; }
    public SlotConfig withKV(double kV) { this.kV = kV; return this; }
    public SlotConfig withKA(double kA) { this.kA = kA; return this; }
    public SlotConfig withOutputLimits(double min, double max) { this.outputMin = min; this.outputMax = max; return this; }
}

