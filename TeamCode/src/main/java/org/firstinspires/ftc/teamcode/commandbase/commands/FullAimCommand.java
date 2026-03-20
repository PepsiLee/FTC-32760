package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpInputHandler;
import org.firstinspires.ftc.teamcode.util.Drawing;

@Config
public class FullAimCommand extends CommandBase {
    private static final Vector2d ZERO_VELOCITY = new Vector2d(0, 0);
    // === 砲塔限制與底盤輔助參數 ===
    public static double TURRET_SOFT_LIMIT = 60.0;      // 砲塔舒適區（度）
    public static double CHASSIS_ASSIST_KP = 0.04;     // 底盤輔助增益
    public static double CHASSIS_MAX_SPEED = 0.9;       // 底盤最大轉速
    public static double MANUAL_ROTATION_DEADZONE = 0.03; // 手把死區
    public static double TURRET_X = 0.0;
    public static double TURRET_Y = 0.0;
    // === 依賴注入 ===
    private final SwerveDrive drive;
    private final ShooterSystem shooterSystem;
    private final GamepadEx driverGamepad;
    private final IMU imu;
    private final Translation2d goalPosition;
    private final TeleOpInputHandler inputHandler;
    private final Vision vision;
    // === 狀態追蹤 ===
    private boolean wasReady = false;

    public FullAimCommand(SwerveDrive drive, ShooterSystem shooterSystem, Vision vision,
                          Translation2d goalPosition, GamepadEx driverGamepad, IMU imu, TeleOpInputHandler inputHandler) {
        this.drive = drive;
        this.shooterSystem = shooterSystem;
        this.goalPosition = goalPosition;
        this.driverGamepad = driverGamepad;
        this.imu = imu;
        this.inputHandler = inputHandler;
        this.vision = vision;

        addRequirements(shooterSystem, drive);
    }

    @Override
    public void initialize() {
        wasReady = false;
    }

    @Override
    public void execute() {
//        updateOdometryWithVision();
        // 1. 獲取當前狀態
        Pose2d robotPose = imu.getPose();
        double headingRad = robotPose.getHeading();
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);

        // 手動旋轉向量，避免建立 Translation2d 中間物件
        double fieldTurretX = robotPose.getX() + (TURRET_X * cosH - TURRET_Y * sinH);
        double fieldTurretY = robotPose.getY() + (TURRET_X * sinH + TURRET_Y * cosH);

        // 建立最終 Turret Pose (這是必要的 new，但我們省去了中間的 Translation2d)
        Pose2d turretPose = new Pose2d(fieldTurretX, fieldTurretY, new Rotation2d(headingRad));


        // 2. 獲取手把輸入
        TeleOpInputHandler.DriveInputs inputs = inputHandler.getDriveInputs(driverGamepad);

        // 3. 更新砲塔瞄準計算
        shooterSystem.aimAtRunning(turretPose, ZERO_VELOCITY, goalPosition);
        double turretRelativeTarget = shooterSystem.getTurretRelativeTarget();

        // 4. 計算底盤旋轉
        double chassisRotation = calculateMixedRotation(
                inputs.rotation,
                turretRelativeTarget
        );

        // 5. 執行移動
        // 修正點：傳入 headingRad (double) 而非 imu 物件
        drive.TeleOpDrive(inputs.forward, inputs.strafe, chassisRotation, headingRad);

        // 6. 狀態反饋
        boolean isReady = shooterSystem.isReady();
        if (isReady && !wasReady) {
            driverGamepad.gamepad.rumbleBlips(1);
        }
        wasReady = isReady;
    }

    private void updateOdometryWithVision() {
        // 傳入 IMU 角度給 MT2
        Pose2d visionPose = vision.getRobotPoseMT2WithHeading(imu.getHeadingDegree());

        if (visionPose != null) {
            Pose2d currentPose = imu.getPose();

            // 安全檢查：只有誤差在合理範圍內才修正 (例如 12 英吋)
            if (currentPose.getTranslation().getDistance(visionPose.getTranslation()) < 12.0) {

                // 使用較低的 Gain (0.1) 讓修正過程平滑，不要一次跳過去
                double newX = currentPose.getX() + (visionPose.getX() - currentPose.getX()) * 0.1;
                double newY = currentPose.getY() + (visionPose.getY() - currentPose.getY()) * 0.1;

                // 寫回 IMU/Odometry
                imu.setPose(new Pose2d(newX, newY, currentPose.getRotation()));
            }
        }
    }

    /**
     * 計算混合旋轉控制（手動 + 自動輔助）
     *
     * @param manualRotation       手把右搖桿輸入
     * @param turretRelativeTarget 砲塔相對於車身的目標角度（度）
     * @return 最終底盤旋轉速度
     */
    private double calculateMixedRotation(double manualRotation, double turretRelativeTarget) {
        double absTurretAngle = Math.abs(turretRelativeTarget);

        // === 情況 1：砲塔在舒適區內 ===
        if (absTurretAngle <= TURRET_SOFT_LIMIT) {
            return manualRotation;
        }

        // === 情況 2：砲塔超出舒適區 ===
        double overflow = absTurretAngle - TURRET_SOFT_LIMIT;
        double autoRotation = Math.signum(turretRelativeTarget) * overflow * CHASSIS_ASSIST_KP;
        autoRotation = Range.clip(autoRotation, -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);

        // 檢查手把是否在對抗輔助
        boolean isManualOverriding = isManualOverridingAssist(manualRotation, turretRelativeTarget);

        if (isManualOverriding) {
            // 手把正在往砲塔更偏的方向轉 → 限制手把速度
            double scaleFactor = Math.max(0.3, 1.0 - (overflow / TURRET_SOFT_LIMIT));
            return manualRotation * scaleFactor;
        } else {
            // 手把沒有對抗 → 混合手動 + 輔助
            double mixedRotation = manualRotation + autoRotation;
            return Range.clip(mixedRotation, -1.0, 1.0);
        }
    }

    /**
     * 檢查手把是否在對抗輔助
     */
    private boolean isManualOverridingAssist(double manualRotation, double turretRelativeTarget) {
        // 沒有明確的手把輸入
        if (Math.abs(manualRotation) < MANUAL_ROTATION_DEADZONE) {
            return false;
        }
        // 檢查手把方向是否與砲塔偏移方向相同
        return Math.signum(manualRotation) == Math.signum(turretRelativeTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSystem.setCalculatedBaseRpm(1073);
    }
}
