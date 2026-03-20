package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import androidx.annotation.NonNull;

import java.util.Locale;

public class SwerveModuleState {
    public double power; // 馬達速度 (-1.0 ~ 1.0)
    public double angle; // 模組角度 (Radians)

    public SwerveModuleState(double power, double angle) {
        this.power = power;
        this.angle = angle;
    }

    // 為了除錯方便，可以加個 toString
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US,"P: %.2f, A: %.2f", power, angle);
    }
}