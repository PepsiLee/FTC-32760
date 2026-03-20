package org.firstinspires.ftc.teamcode.util;

public class SimpleFilter {
    private double lastTx = 0;
    private final double gain = 0.3; // 增益值越小越平滑，建議範圍 0.1 ~ 0.3

    public double update(double currentTx) {
        double filtered = (gain * currentTx) + (1 - gain) * lastTx;
        lastTx = filtered;
        return filtered;
    }
}