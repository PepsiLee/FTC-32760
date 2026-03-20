package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.constant.SwerveConstants;

import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class SwerveDrive extends CustomDrivetrain implements Subsystem {
    private static final boolean voltageCompensation = true;

    public static double TRANSLATION_DEADZONE = 0.01; // 前後左右的死區
    public static double ROTATION_DEADZONE = 0.01;    // 旋轉的死區（通常稍微大一點）
    public static double POW_SCALING = 1;

    // === 硬體物件 ===
    public final SwerveModule lf;
    public final SwerveModule rf;
    public final SwerveModule lb;
    public final SwerveModule rb;
    private final SwerveModule[] modules;

    // === 常數配置 ===
    private final SwerveConstants constants;
    private final PIDFController headingPID;
    private final VoltageMonitor voltageMonitor;

    // === 參數設定 (從 SwerveConstants 同步) ===
    public double TRACK_WIDTH;
    public double WHEEL_BASE;
    // Offset
    public double LF_OFFSET;
    public double RF_OFFSET;
    public double LB_OFFSET;
    public double RB_OFFSET;

    // ========== TeleOp 手動控制 ==========
    public double targetHeading = 0;
    private boolean isHeadingLocked = false;

    public SwerveDrive(HardwareMap hardwareMap, VoltageMonitor voltageMonitor,
                       SwerveConstants swerveConstants) {
        super();
        this.maxPowerScaling = swerveConstants.maxPower;

        // === 初始化常數 ===
        this.constants = swerveConstants;
        this.TRACK_WIDTH = constants.TRACK_WIDTH;
        this.WHEEL_BASE = constants.WHEEL_BASE;
        this.LF_OFFSET = constants.LF_OFFSET;
        this.RF_OFFSET = constants.RF_OFFSET;
        this.LB_OFFSET = constants.LB_OFFSET;
        this.RB_OFFSET = constants.RB_OFFSET;


        // Initialize PID
        headingPID = new PIDFController(new PIDCoefficients(Constant.HEADING_P, Constant.HEADING_I, Constant.HEADING_D));
        headingPID.setInputBounds(-PI, PI);

        // Initialize Hardware
        lf = new SwerveModule(hardwareMap, "fl", "fls", "fle", LF_OFFSET, Constant.LF_BIAS, voltageMonitor, false);
        rf = new SwerveModule(hardwareMap, "fr", "frs", "fre", RF_OFFSET, Constant.RF_BIAS, voltageMonitor, false);
        lb = new SwerveModule(hardwareMap, "rl", "rls", "rle", LB_OFFSET, Constant.LB_BIAS, voltageMonitor, false);
        rb = new SwerveModule(hardwareMap, "rr", "rrs", "rre", RB_OFFSET, Constant.RB_BIAS, voltageMonitor, false);

        //lf, rf, lb, rb
        modules = new SwerveModule[]{lf, rf, lb, rb};

        // === 初始化 Kinematics ===
        SwerveKinematics.initialize(constants);

        this.voltageMonitor = voltageMonitor;

        // Set Motor Direction (從 SwerveConstants 取得)
        lf.driveMotor.setInverted(constants.leftFrontMotorDirection == DcMotor.Direction.REVERSE);
        rf.driveMotor.setInverted(constants.rightFrontMotorDirection == DcMotor.Direction.REVERSE);
        lb.driveMotor.setInverted(constants.leftRearMotorDirection == DcMotor.Direction.REVERSE);
        rb.driveMotor.setInverted(constants.rightRearMotorDirection == DcMotor.Direction.REVERSE);
    }

    /**
     * 主要的 TeleOp 控制方法
     * 修正：最後一個參數改為 double fieldHeading，不再依賴 IMU 物件
     *
     * @param forward      前後移動 (-gamepad.left_stick_y)
     * @param strafe       左右平移 (gamepad.left_stick_x)
     * @param rotation     旋轉 (gamepad.right_stick_x)
     * @param fieldHeading 當前機器人的場地角度 (由外部 IMU 提供)
     */
    public void TeleOpDrive(double forward, double strafe, double rotation, double fieldHeading) {
        double magnitude = Math.hypot(forward, strafe);
        double fInput = 0, sInput = 0;

        if (magnitude > TRANSLATION_DEADZONE) {
            double scaledMag = Math.pow(magnitude, POW_SCALING);
            fInput = (forward / magnitude) * scaledMag;
            sInput = (strafe / magnitude) * scaledMag;
        }

        // 2. 獨立處理旋轉
        double rInput = 0;
        if (Math.abs(rotation) > ROTATION_DEADZONE) {
            rInput = Math.signum(rotation) * Math.pow(Math.abs(rotation), POW_SCALING);
        }

        // 3. 判斷並執行
        if (fInput != 0 || sInput != 0 || rInput != 0) {
            // 修正：calculateFinalRotation 改為接收 currentHeading 數值
            double finalRotation = calculateFinalRotation(rInput, fieldHeading);

            // 修正：SwerveKinematics 直接使用傳入的 fieldHeading
            SwerveModuleState[] states = SwerveKinematics.calculateModuleStates(
                    fInput, sInput, finalRotation, fieldHeading
            );
            runDrive(states);
        } else {
            alignWheelsWhenStopped();
        }
    }

    /**
     * 計算最終旋轉量（包含 Heading Lock 邏輯）
     * 修正：移除 IMU 依賴，改為接收 double currentHeading
     */
    private double calculateFinalRotation(double manualRotation, double currentHeading) {
        // 使用傳入的 currentHeading
        double botheading = currentHeading;

        if (Math.abs(manualRotation) > 0) {
            targetHeading = botheading;
            isHeadingLocked = false;
            return manualRotation;
        } else {
            // 沒有手動輸入：嘗試鎖定角度
            if (!isHeadingLocked) {
                // 原本邏輯：檢查 velocity 是否夠低
                // 修改邏輯：因為沒有 IMU 物件，暫時移除 velocity 檢查，直接鎖定當前角度
                // 如果需要更精確的停止邏輯，建議在 TeleOpMain 計算完 velocity 後一併傳入
                targetHeading = botheading;
                isHeadingLocked = true;
            }

            // 使用 PID 修正角度
            headingPID.setTargetPosition(targetHeading);
            return headingPID.update(botheading);
        }
    }

    /**
     * 停止時對齊所有輪子（避免輪子角度不一致）
     */
    private void alignWheelsWhenStopped() {
//        double masterAngle = lf.encoder.getAbsoluteAngle();

        for (SwerveModule module : modules) {
            module.setTarget(0, module.getTargetAngle());
        }
    }

    /**
     * 檢查輪子是否已對齊
     */
    private boolean isWheelAligned(double targetAngle, double currentAngle, double tolerance) {
        // 使用已有的方法
        double diff = shortestAngleDifference(currentAngle, targetAngle);
        // 正向對齊
        if (Math.abs(diff) <= tolerance) return true;
        // 反向對齊（180度，適用於有 optimize 的情況）
        if (Math.abs(Math.abs(diff) - 180) <= tolerance) return true;

        return false;
    }

    // ========== Pedro Pathing 自動控制（繼承自 CustomDrivetrain）==========

    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {

        //Robot Centric
        SwerveModuleState[] states = SwerveKinematics.calculateModuleStates(
                forward, strafe, rotation, 0
        );
        runDrive(states);
        update();
    }

    /**
     * 計算最短角度差（處理 360° 環繞）
     */
    private double shortestAngleDifference(double current, double target) {
        double diff = target - current;

        // 歸一化到 -180 ~ 180
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;

        return diff;
    }

    // ========== 執行與更新 ==========

    public void runDrive(SwerveModuleState[] states) {
        // 1. 電壓補償
        if (voltageCompensation) {
            double compensationFactor = voltageMonitor.getCompensationFactor();
            for (SwerveModuleState s : states) {
                s.power *= compensationFactor;
            }
        }

        // 2. 正規化最大功率
        double maxSpeed = 0;
        for (SwerveModuleState s : states) {
            maxSpeed = Math.max(maxSpeed, Math.abs(s.power));
        }

        if (maxSpeed > maxPowerScaling) {
            for (SwerveModuleState s : states) {
                s.power = (s.power / maxSpeed) * maxPowerScaling;
            }
        }

        for (int i = 0; i < 4; i++) {
            double finalPower = states[i].power;
            modules[i].setTarget(finalPower, states[i].angle);
        }
    }

    public void update() {
        Robot.getInstance().profiler.start("Swerve");
        Robot.getInstance().profiler.start("Swerve:Logic");
        lf.prepare();
        rf.prepare();
        lb.prepare();
        rb.prepare();
        Robot.getInstance().profiler.end("Swerve:Logic");
        Robot.getInstance().profiler.start("Swerve:WriteHardware");
        lf.apply();
        rf.apply();
        lb.apply();
        rb.apply();
        Robot.getInstance().profiler.end("Swerve:WriteHardware");
        Robot.getInstance().profiler.end("Swerve");
    }

    @Override
    public void periodic() {
        update();
    }

    // ========== 控制方法 ==========

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    @Override
    public void breakFollowing() {
        SwerveModuleState zeroState = new SwerveModuleState(0, 0);
        runDrive(new SwerveModuleState[]{zeroState, zeroState, zeroState, zeroState});
    }


    public void resetHeading() {
        targetHeading = 0;
        isHeadingLocked = false;
    }

    public double getVoltage() {
        return voltageMonitor.getVoltage();
    }

    // ========== Getter/Setter ==========

    public double getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(double heading) {
        this.targetHeading = heading;
        this.isHeadingLocked = false; // 強制重新鎖定
    }

    public boolean isHeadingLocked() {
        return isHeadingLocked;
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    // ========== TeleOp 設定 ==========

    @Override
    public void startTeleopDrive() {
        startTeleopDrive(constants.useBrakeModeInTeleOp);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        setMotorsPowerBehavior(
                brakeMode ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT
        );
    }

    private void setMotorsPowerBehavior(Motor.ZeroPowerBehavior behavior) {
        for (SwerveModule module : modules) {
            module.driveMotor.setZeroPowerBehavior(behavior);
        }
    }

    // ========== 速度控制（Pedro Pathing）==========

    @Override
    public double xVelocity() {
        return constants.xVelocity;
    }

    @Override
    public double yVelocity() {
        return constants.yVelocity;
    }

    @Override
    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    @Override
    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    @Override
    public void updateConstants() {
        // 同步 Dashboard 中修改的常數
        this.TRACK_WIDTH = constants.TRACK_WIDTH;
        this.WHEEL_BASE = constants.WHEEL_BASE;
        this.LF_OFFSET = constants.LF_OFFSET;
        this.RF_OFFSET = constants.RF_OFFSET;
        this.LB_OFFSET = constants.LB_OFFSET;
        this.RB_OFFSET = constants.RB_OFFSET;
    }

    // ========== Telemetry ==========

    @Override
    public String debugString() {
        return String.format(
                "Heading: %.1f° (Target: %.1f°, Locked: %b) | Voltage: %.1fV",
                Math.toDegrees(targetHeading),
                Math.toDegrees(targetHeading),
                isHeadingLocked,
                getVoltage()
        );
    }

    public void setXLock() {
        // X 形態的角度配置（度）
        double LF_ANGLE = 45.0;
        double RF_ANGLE = -45.0;  // 或 315.0
        double LB_ANGLE = -45.0;  // 或 315.0
        double RB_ANGLE = 45.0;

        // 停止所有驅動，只控制轉向
        lf.setTarget(0, LF_ANGLE);
        rf.setTarget(0, RF_ANGLE);
        lb.setTarget(0, LB_ANGLE);
        rb.setTarget(0, RB_ANGLE);
    }
}