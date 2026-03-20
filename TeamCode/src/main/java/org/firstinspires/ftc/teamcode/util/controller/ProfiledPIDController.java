package org.firstinspires.ftc.teamcode.util.controller;

import org.firstinspires.ftc.teamcode.util.motion.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProfiledPIDController {
    public PIDFController pidController; // 改為 public 方便外部調整 PID
    private TrapezoidProfile profile;
    private final ElapsedTime timer = new ElapsedTime();

    private TrapezoidProfile.State setpoint; // 當前理想狀態 (Pos, Vel, Accel)
    private double goalPosition;
    private double profileTotalTime = 0;

    // 新增 Feedforward 係數
    private double kV = 0;
    private double kA = 0;
    private double tolerance = 1.0; // 判斷到位的容許誤差

    public ProfiledPIDController(double p, double i, double d, double f,
                                 double maxVel, double maxAccel,
                                 double kV, double kA) { // 建構子加入 kV, kA
        this.pidController = new PIDFController(p, i, d, f);
        this.profile = new TrapezoidProfile(maxVel, maxAccel);
        this.setpoint = new TrapezoidProfile.State(0, 0, 0);
        this.goalPosition = 0;
        this.kV = kV;
        this.kA = kA;
    }

    public void setGoal(double goal) {
        this.goalPosition = goal;

        // [修正 1] 傳入當前的 setpoint.velocity 作為起始速度，保持運動連續性
        // 注意：你需要確認你的 TrapezoidProfile 有支援 setTarget(startPos, startVel, endPos, endVel)
        // 如果沒有，你需要去修改 TrapezoidProfile 類別
        profile.setTarget(setpoint.position, setpoint.velocity, goal, 0);

        profileTotalTime = profile.getTotalTime();
        timer.reset();
    }

    public double calculate(double measurement) {
        double currentTime = timer.seconds();

        // 1. 更新 Profile 狀態
        if (currentTime >= profileTotalTime) {
            setpoint = new TrapezoidProfile.State(goalPosition, 0, 0);
        } else {
            setpoint = profile.calculate(currentTime);
        }

        // 2. PID 計算 (修正位置誤差)
        pidController.setPoint(setpoint.position);
        double pidOutput = pidController.update(measurement);

        // 3. [修正 2] 加入 Feedforward (速度與加速度前饋)
        // kF (Static) 已經在 pidController 裡處理了 (抗重力/靜摩擦)
        double feedforward = (kV * setpoint.velocity) + (kA * setpoint.acceleration);

        return pidOutput + feedforward;
    }

    public void reset(double currentMeasurement) {
        // 重置時，假設速度與加速度歸零
        this.setpoint = new TrapezoidProfile.State(currentMeasurement, 0, 0);
        this.goalPosition = currentMeasurement;
        // 起始速度設為 0
        profile.setTarget(currentMeasurement, currentMeasurement);
        profileTotalTime = 0;
        timer.reset();
        pidController.reset();
    }

    // [修正 3] 判斷是否真正到位
    public boolean atGoal() {
        boolean profileFinished = timer.seconds() >= profileTotalTime;

        return profileFinished && (Math.abs(pidController.getError()) < tolerance);
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    // Getters
    public double getSetpointPosition() { return setpoint.position; }
    public double getSetpointVelocity() { return setpoint.velocity; }
    public double getSetpointAcceleration() { return setpoint.acceleration; }
    public double getGoal() { return goalPosition; }

    public void setPIDF(double kPAuto, double kIAuto, double kDAuto, double baseKStatic) {
        pidController.setPIDF(kPAuto, kIAuto, kDAuto, baseKStatic);
    }
    public void setFeedForward(double kV, double kA){
        this.kV = kV;
        this.kA = kA;
    }
}