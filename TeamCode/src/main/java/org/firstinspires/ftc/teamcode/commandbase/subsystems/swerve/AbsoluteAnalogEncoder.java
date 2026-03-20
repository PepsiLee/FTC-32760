package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class AbsoluteAnalogEncoder extends SubsystemBase {
    public static double DEFAULT_RANGE = 3.3;
    public static double ENCODER_FULL_TURN = 360.0;

    // 低通濾波係數 (0.0 ~ 1.0)，數值越小越平滑，但延遲越高。建議 0.2 ~ 0.5
    public static double VELOCITY_LPF = 0.3;

    private final AnalogInput encoder;
    private double offset;
    private final double analogRange;
    private boolean inverted;

    private double rotationOffset = 0;
    private double currentRawAngle = 0;
    private double lastRawAngle = 0;
    private double currentContinuousAngle = 0;

    // 用於計算速度的變數
    private double currentVelocity = 0; // Degrees per Second
    private long lastUpdateTime = 0;
    private boolean initialized = false;

    public AbsoluteAnalogEncoder(AnalogInput enc) {
        this(enc, DEFAULT_RANGE);
    }

    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange) {
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }

    public AbsoluteAnalogEncoder zero(double off) {
        offset = off;
        return this;
    }

    public AbsoluteAnalogEncoder setInverted(boolean invert) {
        inverted = invert;
        return this;
    }

    public void update() {
        long currentTime = System.nanoTime();
        double voltage = encoder.getVoltage();

        if (voltage < 0) voltage = 0;
        if (voltage > analogRange) voltage = analogRange;

        this.currentRawAngle = (voltage / analogRange) * ENCODER_FULL_TURN;

        double lastContinuousAngle = 0;
        if (!initialized) {
            lastRawAngle = currentRawAngle;
            currentContinuousAngle = currentRawAngle;
            lastContinuousAngle = currentContinuousAngle;
            lastUpdateTime = currentTime;
            initialized = true;
            return;
        }

        // 1. 處理圈數跳轉邏輯
        double deltaRaw = currentRawAngle - lastRawAngle;
        double threshold = ENCODER_FULL_TURN / 2.0;

        if (deltaRaw < -threshold) {
            rotationOffset += ENCODER_FULL_TURN;
        } else if (deltaRaw > threshold) {
            rotationOffset -= ENCODER_FULL_TURN;
        }

        lastRawAngle = currentRawAngle;
        lastContinuousAngle = currentContinuousAngle;
        currentContinuousAngle = currentRawAngle + rotationOffset;

        // 2. 計算速度 (Velocity = deltaAngle / deltaTime)
        double dt = (currentTime - lastUpdateTime) / 1e9; // 轉為秒
        if (dt > 0) {
            double deltaAngle = currentContinuousAngle - lastContinuousAngle;
            double rawVelocity = deltaAngle / dt;

            // 處理反轉對速度的影響
            if (inverted) rawVelocity = -rawVelocity;

            // 3. 低通濾波處理 (避免類比雜訊導致速度跳變)
            currentVelocity = (VELOCITY_LPF * rawVelocity) + ((1 - VELOCITY_LPF) * currentVelocity);
        }

        lastUpdateTime = currentTime;
    }

    /**
     * 獲取當前角速度
     * @return Degrees per Second
     */
    public double getVelocity() {
        return currentVelocity;
    }

    public double getAbsoluteAngle() {
        double angle = currentRawAngle - offset;
        if (inverted) {
            angle = -angle;
        }
        angle %= 360.0;
        if (angle < 0) {
            angle += 360.0;
        }
        if (angle > 180.0) {
            angle -= 360.0;
        }
        return angle;
    }

    public double getRawAngle() {
        return currentRawAngle;
    }

    public double getPosition() {
        double finalAngle = currentContinuousAngle;
        finalAngle -= offset;
        return inverted ? -finalAngle : finalAngle;
    }

    public double getDegrees() {
        return getPosition();
    }

    public void setCurrentRawAngleAsZero() {
        this.offset = this.currentRawAngle;
    }

    public double getOffset() {
        return this.offset;
    }
}