package org.firstinspires.ftc.teamcode.utility;

import java.util.HashMap;
import java.util.Map;

public class PIDSVAController {
    private final Map<Integer, SlotConfig> slots = new HashMap<>();
    private int currentSlot = 0;
    private double integral = 0, previousError = 0;

    public static PIDSVAController create() {
        return new PIDSVAController();
    }

    /**
     * 快速设置默认slot(0号slot)的PID和SVA参数
     */
    public PIDSVAController withSlot0(SlotConfig config) {
        return withSlot(0, config);
    }

    /**
     * 设置第n个slot的PID和SVA参数
     * @param slot
     * @param config
     * @return
     */
    public PIDSVAController withSlot(int slot, SlotConfig config) {
        slots.put(slot, config);
        return this;
    }

    /**
     * 切换到第n个slot，重置积分和微分状态
     * @param slot
     */
    public void setSlot(int slot) {
        if (!slots.containsKey(slot)) throw new IllegalArgumentException("Slot not configured");
        currentSlot = slot;
        integral = 0;
        previousError = 0;
    }

    /**
     * 快捷方法：简单速度闭环
     * setpoint 作为前馈速度项，忽略加速度项
     */
    public double calculate(double setpoint, double measurement, double dt) {
        return calculate(setpoint, measurement, setpoint, 0.0, dt);
    }

    /**
     * 快捷方法：简单速度闭环 / 简单位置闭环
     * VelCycle 为真：setpoint 作为前馈速度项，忽略加速度项
     * VelCycle 为假：前馈速度和加速度项均为0
     */
    public double calculate(double setpoint, double measurement, double dt, boolean VelCycle) {
        if(VelCycle){
            return calculate(setpoint, measurement, setpoint, 0.0, dt);
        }
        else{
            return calculate(setpoint, measurement, 0.0, 0.0, dt);
        }
    }


    /**
     * 完整PIDSVA闭环，计算输出
     * @param setpoint 目标值（位置或速度）
     * @param measurement 当前值（位置或速度）
     * @param velocity 当前速度（用于SVA前馈）
     * @param acceleration 当前加速度（用于SVA前馈）
     * @param dt 时间间隔
     * @return 输出
     */
    public double calculate(double setpoint, double measurement, double velocity, double acceleration, double dt) {
        SlotConfig cfg = slots.get(currentSlot);
        double error = setpoint - measurement;
        integral += error * dt;
        if (cfg != null && integral > cfg.maxI) integral = cfg.maxI;
        if (cfg != null && integral < -cfg.maxI) integral = -cfg.maxI;
        double derivative = (error - previousError) / dt;
        previousError = error;
        double pid = cfg.kP * error + cfg.kI * integral + cfg.kD * derivative;
        double sva = cfg.kS * Math.signum(velocity) + cfg.kV * velocity + cfg.kA * acceleration;
        double output = pid + sva;
        if (output > cfg.outputMax) output = cfg.outputMax;
        if (output < cfg.outputMin) output = cfg.outputMin;
        return output;
    }

    /**
     * 重置积分和微分状态（不切换slot）
     */
    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
