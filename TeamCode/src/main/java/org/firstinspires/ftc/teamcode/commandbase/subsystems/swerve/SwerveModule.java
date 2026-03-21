package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
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
    // 舵機角度變化速度限制（deg/sec），用來平滑邊走邊轉
    public static double MAX_STEER_RATE_DEG_PER_SEC = 540.0;
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

    private double lastCommandedAngle = Double.NaN;
    private long lastPrepareNanos = 0L;

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

        // 1) 角度 wrap：先將目標角度正規化到標準範圍 (-180 ~ 180)
        double optimizedAngle = normalizeNeg180To180(targetAngle);
        double speedMultiplier = targetSpeed;

        // 2) 最短路徑優化：若需要轉超過 90 度，改走「角度 +180 並反轉輪速」
        double angleError = shortestAngleDifference(currentAngle, optimizedAngle);
        if (Math.abs(angleError) > 90.0) {
            optimizedAngle = normalizeNeg180To180(optimizedAngle + 180.0);
            speedMultiplier *= -1.0;
        }

        // 3) 平滑限制（slew rate）：限制每秒可改變的角度，避免邊走邊轉時頓挫
        optimizedAngle = rateLimitAngle(optimizedAngle);

        // 4) 計算 Servo 最終位置 (Bias + 角度偏移)
        this.finalServoPosition = this.servoBias + optimizedAngle + SERVO_MAX_ANGLE / 2;

        // 5) 設定驅動馬達速度
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

    private double shortestAngleDifference(double current, double target) {
        return normalizeNeg180To180(target - current);
    }

    private double rateLimitAngle(double targetAngle) {
        long nowNanos = System.nanoTime();

        if (Double.isNaN(lastCommandedAngle) || lastPrepareNanos == 0L) {
            lastCommandedAngle = targetAngle;
            lastPrepareNanos = nowNanos;
            return targetAngle;
        }

        double dtSec = (nowNanos - lastPrepareNanos) / 1e9;
        if (dtSec <= 0) {
            return lastCommandedAngle;
        }

        double maxStep = MAX_STEER_RATE_DEG_PER_SEC * dtSec;
        double diff = shortestAngleDifference(lastCommandedAngle, targetAngle);
        double limitedStep = Math.max(-maxStep, Math.min(maxStep, diff));

        lastCommandedAngle = normalizeNeg180To180(lastCommandedAngle + limitedStep);
        lastPrepareNanos = nowNanos;
        return lastCommandedAngle;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public void stop() {
        turnServo.disable();
        driveMotor.set(0);
        lastCommandedAngle = Double.NaN;
        lastPrepareNanos = 0L;
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}
