package org.firstinspires.ftc.teamcode.util.controller;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    public double debug_error = 0;
    public double debug_pTerm = 0;
    public double debug_iTerm = 0;
    public double debug_dTerm = 0;
    public double debug_fTerm = 0; // Feedforward
    public double debug_dt = 0;    // 監測時間差
    // PIDF 係數
    private double kP, kI, kD, kF;
    private IntegrationControl integrationControl = new IntegrationControl();
    // === 來自代碼 B 的參數 ===
    private double iZone = 0;
    private double filterGain = 0.4;
    private double tolerance = 1.0;
    // === [新增] 輸出限制參數 ===
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    // 系統狀態變數
    private double setPoint = 0;
    private double error = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastDerivative = 0;
    private long lastTimestamp = 0;
    // --- 核心計算 ---
    private double lastPosition = 0; // 用於計算位置變化

    public PIDFController(double p, double i, double d, double f) {
        setPIDF(p, i, d, f);
    }

    // --- 參數設定 ---
    public void setPIDF(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    /**
     * [新增] 設定輸出的最大與最小範圍
     * 例如 setOutputBounds(-0.8, 0.8) 可以限制馬達只用 80% 力
     */
    public void setOutputBounds(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public void setIntegrationControl(IntegrationControl integrationControl) {
        this.integrationControl = integrationControl;
    }

    public void setIZone(double iZone) {
        this.iZone = iZone;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setDerivativeFilterGain(double gain) {
        this.filterGain = Range.clip(gain, 0, 0.99);
    }

    public void setPoint(double target) {
        setSetPoint(target);
    }

    public void setPoint(double target, double currentPos) {
        this.setPoint = target;
        this.integralSum = 0;
        this.lastError = target - currentPos;
        this.lastDerivative = 0;
    }

    public double update(double currentPos) {
        long currentTime = System.nanoTime();
        if (lastTimestamp == 0) {
            lastTimestamp = currentTime;
            this.lastError = error;
            lastPosition = currentPos; // 初始化位置
            return 0;
        }

        double rawDt = (currentTime - lastTimestamp) / 1.0e9;
        double dt = Math.min(rawDt, 0.04);
        if (dt < 1E-6) dt = 1E-6;
        lastTimestamp = currentTime;

        // 防止除以零
        if (dt < 1E-6) dt = 1E-6;

        // 記錄 Debug 數據 (ms)
        this.debug_dt = dt * 1000.0;

        // === 1. 計算誤差 ===
        this.error = setPoint - currentPos;
        this.debug_error = error;

        // === 2. P 項 ===
        double pTerm = kP * error;
        this.debug_pTerm = pTerm;

        // === 3. I 項 ===
        double iTerm = 0;
        if (kI != 0) {
            if (iZone == 0 || abs(error) < iZone) {
                // [注意] 這裡現在會直接乘上真實的 dt (例如 0.07s)，掉幀時積分會增加很快
                integralSum += error * dt;

                double minI, maxI;
                if (integrationControl.getMinIntegral() != -1.0 || integrationControl.getMaxIntegral() != 1.0) {
                    minI = integrationControl.getMinIntegral();
                    maxI = integrationControl.getMaxIntegral();
                } else {
                    minI = -Double.MAX_VALUE;
                    maxI = Double.MAX_VALUE;
                }
                integralSum = Range.clip(integralSum, minI, maxI);

                if (Math.signum(integralSum) != Math.signum(error)) {
                    integralSum *= integrationControl.getDecayFactor();
                }

                if (atSetPoint() && integrationControl.getIntegrationBehavior() == IntegrationBehavior.CLEAR_AT_SP) {
                    integralSum = 0;
                }

                iTerm = kI * integralSum;
            } else {
                integralSum = 0;
            }
        }
        this.debug_iTerm = iTerm;

        // === 4. D 項 ===
        double dTerm = 0;
        if (kD != 0) {
            // [關鍵修改]：不看誤差變化，改看「位置變化」 = 速度
            double velocity = (currentPos - lastPosition) / dt;

            // 低通濾波 (解決感測器雜訊，防止抖動)
            double filteredVel = (filterGain * lastDerivative) + ((1 - filterGain) * velocity);
            lastDerivative = filteredVel;

            // [關鍵修改]：阻尼輸出 = -kD * 速度
            // 速度越大，阻尼給予的反向推力越大
            dTerm = -kD * filteredVel;
        }
        this.debug_dTerm = dTerm;

        // === 5. F 項 ===
        double fTerm = 0;
        if (kF != 0) {
            fTerm = Math.signum(error) * kF;
        }
        this.debug_fTerm = fTerm;

        lastError = error;
        lastPosition = currentPos;
        double output = pTerm + iTerm + dTerm + fTerm;

        // === 6. 輸出限制 ===
        return Range.clip(output, minOutput, maxOutput);
    }

    public boolean atSetPoint() {
        return Math.abs(error) < tolerance;
    }

    public void clearTotalError() {
        integralSum = 0;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastDerivative = 0;
        lastTimestamp = 0;
    }

    // --- Getters ---
    public double getError() {
        return error;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetPoint(double target) {
        if (Math.abs(target - this.setPoint) > 1) {
            this.setPoint = target;
            this.integralSum = 0; // 換大目標時清空積分防止衝過頭
        }
    }

    public double getTargetPosition() {
        return setPoint;
    }

    // === 來自代碼 A 的積分控制模組 ===
    public enum IntegrationBehavior {
        NONE,           // 無特殊行為
        CLEAR_AT_SP     // 到位時清除積分 (防止 Overshoot)
    }

    public static class IntegrationControl {
        IntegrationBehavior integrationBehavior;
        double decayFactor;
        double minIntegral;
        double maxIntegral;

        public IntegrationControl(IntegrationBehavior integrationBehavior, double decayFactor, double minIntegral, double maxIntegral) {
            this.integrationBehavior = integrationBehavior;
            this.decayFactor = decayFactor;
            this.minIntegral = minIntegral;
            this.maxIntegral = maxIntegral;
        }

        public IntegrationControl() {
            this(IntegrationBehavior.NONE, 1.0, -1.0, 1.0);
        }

        public double getDecayFactor() {
            return decayFactor;
        }

        public double getMinIntegral() {
            return minIntegral;
        }

        public double getMaxIntegral() {
            return maxIntegral;
        }

        public IntegrationBehavior getIntegrationBehavior() {
            return integrationBehavior;
        }
    }
}