package org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter;

public class ShotState {
    public final double rpm;          // 目標轉速
    public final double hoodAngle;    // 目標仰角 (Servo 0.0-1.0)
    public final double turretOffset; // 砲塔修正角 (度)

    public ShotState(double rpm, double hoodAngle, double turretOffset) {
        this.rpm = rpm;
        this.hoodAngle = hoodAngle;
        this.turretOffset = turretOffset;
    }
}