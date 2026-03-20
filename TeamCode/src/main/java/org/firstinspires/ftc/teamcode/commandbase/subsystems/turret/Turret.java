package org.firstinspires.ftc.teamcode.commandbase.subsystems.turret;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.constant.Constant;

@Config
public class Turret extends SubsystemBase {
    // --- 硬體物件 ---
    private final ServoEx servoDrive;
    private final ServoEx servoHoodL, servoHoodR;
    // --- Debug 變數 ---
    double totalWaitTime;
    // --- 狀態變數 ---
    private double targetAngle = 0.0;
    private double lastServoAngle = 0;
    private double angle = 0;
    private ElapsedTime rotationTimer;

    public Turret(HardwareMap hwMap, VoltageMonitor voltageMonitor, IMU imu) {
        servoDrive = new ServoEx(hwMap, "turret").setPwm(new PwmControl.PwmRange(500, 2500));
        servoDrive.setInverted(false);

        servoHoodL = new ServoEx(hwMap, "HoodOne").setPwm(new PwmControl.PwmRange(1050, 1950));
        servoHoodR = new ServoEx(hwMap, "HoodTwo").setPwm(new PwmControl.PwmRange(1050, 1950));

        rotationTimer = new ElapsedTime();
    }

    public void update() {
        if (rotationTimer.milliseconds() > totalWaitTime) {
            lastServoAngle = angle;
        }
    }

    @Override
    public void periodic() {
        update();
    }

    public double getAngle() {
        // 1. 取得 Servo 目前的原始數值 (0.0 ~ 1.0)
        double currentServoValue = servoDrive.get();

        // 2. 反向計算回物理角度
        // 公式：((Servo數值 * 最大行程) - 零點偏移) * 齒輪比
        double currentPhysicalAngle = ((currentServoValue * Constant.MAX_SERVO_ANGLE) - Constant.ZERO_POINT) * Constant.GEAR_RATIO;

        return currentPhysicalAngle;
    }
    // --- 對外介面 ---
    public void setTargetAngle(double angle) {
        this.targetAngle = Range.clip(angle, Constant.MIN_ANGLE, Constant.MAX_ANGLE);
        double servoAngle = (Constant.ZERO_POINT / Constant.MAX_SERVO_ANGLE) + ((targetAngle / Constant.GEAR_RATIO) / Constant.MAX_SERVO_ANGLE);
        servoDrive.set(servoAngle);
        rotationTimer.reset();

        double deltaAngle = Math.abs(targetAngle - lastServoAngle);
        double estimatedTimeNeeded = (deltaAngle / Constant.SERVO_DEGREES_PER_SECOND) * 1000; // 換算成 ms
        totalWaitTime = estimatedTimeNeeded + 100;
    }

    public void adjustTargetAngle(double delta) {
        setTargetAngle(targetAngle + delta);
    }

    public boolean atTarget() {
        return true; //TODO: SOLVE THE fast setTarget made the timer not satisfied
//        return rotationTimer.milliseconds() > totalWaitTime;
    }

    public double getHoodPosition() {
        return servoHoodL.get();
    }

    public void setHoodPosition(double pos) {
        System.out.println("HOOD set to : "+ pos);
        servoHoodL.set(pos);
        servoHoodR.set(pos);
    }

    public double getError() {
        return targetAngle - getAngle();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

}