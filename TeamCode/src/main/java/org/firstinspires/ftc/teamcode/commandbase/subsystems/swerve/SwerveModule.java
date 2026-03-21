package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.constant.Constant;


@Config
public class SwerveModule extends SubsystemBase {

    // === 硬體參數 ===
    public static double SERVO_MAX_ANGLE = 355.0;
    private static final double DEGREES_TO_SERVO = 1.0 / SERVO_MAX_ANGLE;
    private final VoltageMonitor voltageMonitor;
    public String tag = "SwerveModule";
    public MotorEx driveMotor;
    public ServoEx turnServo;
    public AbsoluteAnalogEncoder encoder;
    public double currentAngle = 0; // Public 讓 Drive 可以讀取做里程計
    private double targetSpeed = 0;
    private double targetAngle = 0;
    private double finalServoPosition = 0; // 預設中間
    private double finalDrivePower = 0;
    private double lastAppliedServoPosition = Double.NaN;
    private double lastAppliedDrivePower = Double.NaN;

    // ✅ FIX: 保存傳入的 Bias
    private double servoBias;
    private String driveName;
    /**
     * 階段一：純計算 (Logic)
     */

    private boolean isInverted = false;

    // ✅ FIX: 建構式加入 servoBias 參數
    public SwerveModule(@NonNull HardwareMap hardwareMap, String driveName, String servoName, String encoderName,
                        double offset, double servoBias, VoltageMonitor voltageMonitor, boolean invert) {
        this.driveName = driveName;
        this.voltageMonitor = voltageMonitor;
        this.servoBias = servoBias;

        driveMotor = new MotorEx(hardwareMap, driveName);
        // 設定 Servo 範圍 0-355
        turnServo = new ServoEx(hardwareMap, servoName, 0, 355);
        turnServo.setPwm(new PwmControl.PwmRange(500, 2500));
        turnServo.setInverted(invert);

        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, encoderName);
        encoder = new AbsoluteAnalogEncoder(analogInput, Constant.Servo_RANGE).zero(offset).setInverted(false);

        driveMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        driveMotor.setRunMode(Motor.RunMode.RawPower);
        driveMotor.setCachingTolerance(0.01);
    }

    public void setTarget(double speed, double angle) {
        this.targetSpeed = speed;
        this.targetAngle = angle;
    }

    public void setServoBias(double bias){
        this.servoBias = bias;
    }

    public void prepare() {
        encoder.update();
        this.currentAngle = encoder.getAbsoluteAngle();

        // 2. 正規化目標角度 (-180 ~ 180)
        double optimizedAngle = normalizeNeg180To180(targetAngle);
        double speedMultiplier = targetSpeed;

        // === 3. 滯後邏輯 (Hysteresis) 開始 ===

        // 設定門檻值
        double enterThreshold = 100.0; // 超過這個值 -> 進入反轉
        double exitThreshold = 80.0;  // 低於這個值 -> 離開反轉 (這就是你不想要馬上跳回來的原因)

        double absAngle = Math.abs(optimizedAngle);

        if (!isInverted) {
            // 狀態 A: 目前是「正常模式」
            if (absAngle > enterThreshold) {
                isInverted = true;
            }
        } else {
            // 狀態 B: 目前是「反轉模式」
            if (absAngle < exitThreshold) {
                isInverted = false;
            }
        }

        // === 根據上面的狀態，執行反轉運算 ===
        if (isInverted) {
            if (optimizedAngle > 0) {
                optimizedAngle -= 180.0;
            } else {
                optimizedAngle += 180.0;
            }
            speedMultiplier *= -1.0; // 反轉驅動馬達
        }

        // 4. 計算 Servo 最終位置 (Bias + 角度偏移)
        this.finalServoPosition = this.servoBias + optimizedAngle + SERVO_MAX_ANGLE / 2;

        // 6. 設定驅動馬達速度
        this.finalDrivePower = speedMultiplier;
    }

    /**
     * 階段二：硬體寫入 (Write)
     */
    public void apply() {
        String writeTag = "Swerve:" + driveName + ":Write";
        Robot.getInstance().profiler.start(writeTag);

        if (Double.isNaN(lastAppliedServoPosition) ||
                Math.abs(finalServoPosition - lastAppliedServoPosition) > Constant.TURN_SERVO_CACHING_TOLERANCE) {
            turnServo.set(finalServoPosition);
            lastAppliedServoPosition = finalServoPosition;
        }

        if (Double.isNaN(lastAppliedDrivePower) ||
                Math.abs(finalDrivePower - lastAppliedDrivePower) > Constant.DRIVE_MOTOR_CACHING_TOLERANCE) {
            driveMotor.set(finalDrivePower);
            lastAppliedDrivePower = finalDrivePower;
        }

        Robot.getInstance().profiler.end(writeTag);
    }

    // 輔助方法
    private double normalizeNeg180To180(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public void stop() {
        turnServo.disable();
        driveMotor.set(0);
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}
