package org.firstinspires.ftc.teamcode.utility;

/**
 * 直线类，表示一条直线 y = slope * x + intercept
 */
public class Line {
    private final double slope;      // 斜率
    private final double intercept;  // 截距
    private final double rSquared;   // 拟合优度 R²

    public Line(double slope, double intercept, double rSquared) {
        this.slope = slope;
        this.intercept = intercept;
        this.rSquared = rSquared;
    }

    /**
     * 根据x值预测y值
     * @param x 自变量x
     * @return 预测的y值
     */
    public double predict(double x) {
        return slope * x + intercept;
    }

    /**
     * 获取直线的参数方程
     * @return 直线方程字符串
     */
    public String getEquation() {
        if (Double.isNaN(slope) || Double.isNaN(intercept)) {
            return "无法拟合直线";
        }

        String sign = intercept >= 0 ? " + " : " - ";
        return String.format("y = %.4fx%s%.4f", slope, sign, Math.abs(intercept));
    }

    /**
     * 获取与x轴的交点（当y=0时）
     * @return x轴交点
     */
    public Double getXIntercept() {
        if (Math.abs(slope) < 1e-10) return null;
        return -intercept / slope;
    }

    /**
     * 获取与y轴的交点（当x=0时）
     * @return y轴交点
     */
    public double getYIntercept() {
        return intercept;
    }

    /**
     * 获取点到直线的距离
     * @param x 点的x坐标
     * @param y 点的y坐标
     * @return 点到直线的垂直距离
     */
    public double distanceToPoint(double x, double y) {
        return Math.abs(slope * x - y + intercept) / Math.sqrt(slope * slope + 1);
    }

    // Getters
    public double getSlope() { return slope; }
    public double getIntercept() { return intercept; }
    public double getRSquared() { return rSquared; }

    @Override
    public String toString() {
        return String.format("Line{%s, R²=%.4f}",
                getEquation(), rSquared);
    }
}
