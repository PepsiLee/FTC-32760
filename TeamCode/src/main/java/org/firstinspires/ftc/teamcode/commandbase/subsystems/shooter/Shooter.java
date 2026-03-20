package org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.constant.Constant;

@Config
public class Shooter extends SubsystemBase {
    // PIDF 參數 (需根據實際電壓調整)
    // kP: 誤差 100 RPM 時補償多少伏特? 100 * 0.003 = 0.3V (看起來合理)
    public static double kP = 0.032, kI = 0, kD = 0;
    // kS: 靜摩擦力與單位無關，保持不變（但建議測量出一個微小值，如 0.05）
    public static double kS = 0.05, kV = 0.00514, kA = 0.01;
    public static double TOLERANCE = 23; // RPM
    private final MotorEx flywheelOne, flywheelTwo;
    private final Servo gate;
    private final VoltageMonitor voltageMonitor;
    public String tag = "Shooter";
    private PIDController pidController;
    private SimpleMotorFeedforward feedforward;

    private boolean activeControl = false;

    public boolean isOverdrive() {
        return isOverdrive;
    }

    private boolean isOverdrive = false;
    private double targetTPS = 0.0;
    private double finalPower = 0;

    public Shooter(HardwareMap hwMap, VoltageMonitor voltageMonitor) {
        flywheelOne = new MotorEx(hwMap, "FlywheelOne");
        flywheelTwo = new MotorEx(hwMap, "FlywheelTwo");
        gate = hwMap.get(Servo.class, "Gate");

        this.voltageMonitor = voltageMonitor;

        // 設定 Coast 模式非常重要 (FLOAT)
        configureMotor(flywheelOne, DcMotorEx.Direction.REVERSE);
        configureMotor(flywheelTwo, DcMotorEx.Direction.FORWARD);

        flywheelOne.setCachingTolerance(0.01);
        flywheelTwo.setCachingTolerance(0.01);

        pidController = new PIDController(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    // [關鍵] 提供電流數據給 System 層做射擊偵測
    public double getCurrent(){
        return flywheelOne.getCurrent(CurrentUnit.AMPS);
    }

    private void configureMotor(Motor motor, DcMotorEx.Direction dir) {
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT); // Coast Mode
        motor.setInverted(dir == DcMotor.Direction.REVERSE);
    }

    public void setFlywheel(double rpm) {
        this.targetTPS = rpm;
        activeControl = true;
    }

    // [新增] 允許外部強制開啟 Overdrive (例如偵測到電流突波時)
    public void setOverdrive(boolean state) {
        this.isOverdrive = state;
    }

    public void stop() {
        activeControl = false;
        isOverdrive = false;
        targetTPS = 0;
        flywheelOne.set(0);
        flywheelTwo.set(0);
    }

    public void shoot() {
        activeControl = true;
    }

    public void stopShoot() {
        activeControl = false;
        targetTPS = 0; // 待機轉速
    }

    public boolean flywheelReady() {
        return activeControl && targetTPS > 46 && abs(getCurrentRPM() - targetTPS) < TOLERANCE;
    }

    public double getTargetTPS() {
        return targetTPS;
    }

    public double getCurrentRPM() {
//        return abs(flywheelOne.getVelocity() * 60.0) / Constant.TICKS_PER_REV;
        return  flywheelOne.getVelocity() * -1;
    }

    public double getFinalPower() {
        return finalPower;
    }

    @Override
    public void periodic() {
        pidController.setPID(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        if (activeControl && targetTPS > 0) {

            double currentRpm = getCurrentRPM();

            double ffVolts = feedforward.calculate(targetTPS);
            double pidVolts = pidController.calculate(currentRpm, targetTPS);
            double targetVoltage = ffVolts + pidVolts;

            double batteryVoltage = voltageMonitor.getVoltage();

            finalPower = targetVoltage / batteryVoltage;
            finalPower = Range.clip(finalPower, 0.0, 1.0);

//            // ✅ 分離三種模式
//            if (isOverdrive) {
//                if(abs(targetTPS - currentRpm) < 200){
//                    isOverdrive = false;
//                } else {
//                    finalPower = 1.0;
//                    RobotLog.vv(tag, "Overdrive Mode");
//                }
//            }
//            else if (currentRpm < targetTPS * 0.8) {
//                finalPower = 1.0;
//            }
//            else {
//                // === 模式 4: PIDF 精細控制 ===
//
//            }

            flywheelOne.set(finalPower);
            flywheelTwo.set(finalPower);
        } else {
            flywheelOne.set(0);
            flywheelTwo.set(0);
        }
    }

    // Gate
    public boolean isGateOpen() {
        return gate.getPosition() == Constant.GATE_OPEN;
    }

    public void setGateOpen(boolean engaged) {
        gate.setPosition(engaged ? Constant.GATE_OPEN : Constant.GATE_CLOSE);
    }
}