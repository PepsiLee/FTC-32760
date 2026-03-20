package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static com.seattlesolvers.solverslib.purepursuit.PurePursuitUtil.angleWrap;
import static com.seattlesolvers.solverslib.util.MathUtils.clamp;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.ShotState;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.constant.AprilTags;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.util.SimpleFilter;

@Config
public class ShooterSystem extends SubsystemBase {
    // 設定判斷標準：如果誤差超過 200 RPM，視為吃到球
    private static final double SHOT_DETECTION_THRESHOLD = 150;
    // Boost 持續時間 (例如 0.5 秒)
    private static final double BOOST_DURATION = 0.5;
    // === Dashboard 參數 ===
    public static double RPM_TOLERANCE = 50;     // RPM
    public static double HOOD_MIN = 0.0, HOOD_MAX = 1;
    public static double kComp = 0; // 掉速時的仰角補償係數
    // [新增] 聯盟顏色係數 (預設 Blue=1, Red=-1)
    public static double ALLIANCE_MULTIPLIER = 1.0;
    public static double CAMERA_FORWARD_OFFSET = 5.0; // 相機前方偏移 (X_offset in camera frame)
    public static double CAMERA_RIGHT_OFFSET = 4.0;   // 相機右側偏移 (Y_offset in camera frame)
    private final Turret turret;
    private final Shooter shooter;
    private final ShooterCalculator calculator;
    SimpleFilter visionFilter = new SimpleFilter();
    double VISION_DEADBAND = 1.0;     // < 1° 不修
    double MAX_BIAS = 20.0;           // servo 可接受
    double kP_Vision = 0.4;  // 比例項：負責快速對準，但不累積
    double kI_Vision = 0.02; // 積分項：負責修正 Odometry 長期漂移 (原本的 kVision 改小)
    private SystemStatus currentStatus = SystemStatus.IDLE;
    private boolean isTriggerPulled = false;
    private double currentDist = 0, baseHood = 1.0;
    private double manualAngleOffset = 0;
    private double turretRelativeTarget = 0;
    private double calculatedBaseRpm = 0;
    private double visionBiasDeg = 0.0;      // 視覺修正偏移
    // === [合併變數] 統一管理 Overdrive 狀態 ===
    private boolean isOverdriveActive = false;
    private ElapsedTime overdriveTimer = new ElapsedTime();

    public ShooterSystem(Shooter shooter, Turret turret) {
        this.shooter = shooter;
        this.turret = turret;
        this.calculator = new ShooterCalculator();
    }

    /**
     * 全自動移動射擊瞄準
     */
    public void aimAtRunning(Pose2d robotPose, Vector2d robotVel, Translation2d targetPos) {
        // === [Step 1: 進球角度補償 (LUT)] ===
        Vector2d originalTargetVec = new Vector2d(targetPos.getX(), targetPos.getY());
        Vector2d adjustedTargetVec = calculator.getAdjustedTarget(robotPose, originalTargetVec, ALLIANCE_MULTIPLIER);
        Translation2d finalTargetPos = new Translation2d(adjustedTargetVec.getX(), adjustedTargetVec.getY());

        // === [Step 2: 物理與移動補償] ===
        Translation2d displacement = finalTargetPos.minus(robotPose.getTranslation());
        double distance = displacement.getNorm();
        Vector2d distVec = new Vector2d(displacement.getX(), displacement.getY());

        Vector2d radialUnit = distVec.div(distVec.magnitude());
        Vector2d tangentialUnit = new Vector2d(-radialUnit.getY(), radialUnit.getX());

        double radialVel = robotVel.dot(radialUnit);
        double tangentialVel = robotVel.dot(tangentialUnit);

        ShotState solution = calculator.calculate(distance, radialVel, tangentialVel);

        double baseAngleDeg = Math.toDegrees(Math.atan2(distVec.getY(), distVec.getX()));
        double compensatedFieldAngle = baseAngleDeg + solution.turretOffset;
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        double rawRelativeAngle = compensatedFieldAngle - robotHeadingDeg;

        this.turretRelativeTarget = Math.toDegrees(angleWrap(Math.toRadians(rawRelativeAngle)));

        // === [Step 4: 視覺微調 (Vision Fine-tuning)] ===
        Robot.getInstance().vision.setTargetID(Constant.ALLIANCE_COLOR == Constant.AllianceColor.BLUE ? AprilTags.BLUE_GOAL.id : AprilTags.RED_GOAL.id);
        double rawTx = Robot.getInstance().vision.getTx(1);
        double tx = visionFilter.update(rawTx);

        double parallaxCorrection = 0.0;
        double effectiveDistance = distance - CAMERA_FORWARD_OFFSET;

        if (effectiveDistance > 1.0) {
            parallaxCorrection = -Math.toDegrees(Math.atan2(CAMERA_RIGHT_OFFSET, effectiveDistance));
        }

        double visionCorrectionP = 0;
        double error = 0.0; // 將 error 宣告移到外面以便列印

        if (Math.abs(tx) > 0.1) {
            error = -tx + parallaxCorrection;

            if (Math.abs(tx) > VISION_DEADBAND) {
                visionBiasDeg += error * kI_Vision;
                visionBiasDeg = clamp(visionBiasDeg, -MAX_BIAS, MAX_BIAS);
            }

            visionCorrectionP = error * kP_Vision;
        }

        double finalTarget = turretRelativeTarget + visionBiasDeg + visionCorrectionP;

        turret.setTargetAngle(finalTarget);
        turretRelativeTarget = finalTarget;
        this.calculatedBaseRpm = solution.rpm;
        this.baseHood = solution.hoodAngle;
        this.currentDist = distance;

        // ==========================================
        // 🖨️ DEBUG: 輸出所有關鍵資訊到 Logcat
        // ==========================================
        System.out.println("\n=== 🎯 AIM DEBUG START ===");

        // 1. 機器人狀態
        System.out.printf("🤖 Pose: X:%.2f, Y:%.2f, H:%.2f | Vel: R:%.2f, T:%.2f%n",
                robotPose.getX(), robotPose.getY(), robotHeadingDeg, radialVel, tangentialVel);

        // 2. 物理計算
        System.out.printf("📏 Dist: %.2f (Eff: %.2f) | BaseAng: %.2f | LeadOffset: %.2f%n",
                distance, effectiveDistance, baseAngleDeg, solution.turretOffset);

        // 3. 里程計目標
        System.out.printf("🧭 OdomTarget: %.2f (RawRel: %.2f)%n",
                turretRelativeTarget, rawRelativeAngle);

        // 4. 視覺數據
        System.out.printf("👁️ Vision: RawTx:%.2f -> FiltTx:%.2f | PC: %.2f%n",
                rawTx, tx, parallaxCorrection);

        // 5. 誤差與修正
        System.out.printf("⚠️ Error: %.2f (-tx+PC) | Terms: P:%.2f, I:%.2f%n",
                error, visionCorrectionP, visionBiasDeg);

        // 6. 最終輸出
        System.out.printf("🚀 FINAL TARGET: %.2f (Odom + P + I)%n", finalTarget);
        System.out.printf("🔫 Shooter: RPM:%.0f, Hood:%.2f%n", solution.rpm, solution.hoodAngle);

        System.out.println("=== 🎯 AIM DEBUG END ===\n");
    }

    private void startOverdrive() {
        // 只有在還沒啟動時重置計時器，避免連續觸發導致計時器一直被歸零
        if (!isOverdriveActive) {
            isOverdriveActive = true;
            overdriveTimer.reset();
        }
    }

    public void triggerBoost() {
        startOverdrive();
    }

    /**
     * 手動微調砲塔角度
     */
    public void adjustTurretManual(double delta) {
        this.manualAngleOffset += delta;
    }

    public void setAllianceMultiplier(double multiplier) {
        ALLIANCE_MULTIPLIER = multiplier;
    }

    public void setCalculatedBaseRpm(double rpm) {
        this.calculatedBaseRpm = rpm;
    }

    @Override
    public void periodic() {
        updateLogic();
    }

    private void updateLogic() {
        // === 1. 獲取當前狀態 ===
        double currentRPM = shooter.getCurrentRPM();
        double baseError = calculatedBaseRpm - currentRPM;
        boolean turretAligned = turret.atTarget();
        boolean shooterRevved = shooter.flywheelReady();
        // 判斷轉速是否穩定 (誤差小於容許值的 2 倍)
        boolean rpmRecovered = Math.abs(baseError) < RPM_TOLERANCE * 2;

        // === 2. 觸發條件 (Trigger) ===
        // 如果偵測到嚴重掉速 (且目前沒在 Overdrive)，自動觸發
        if (!isOverdriveActive && calculatedBaseRpm > 1000 && baseError > SHOT_DETECTION_THRESHOLD) {
            startOverdrive();
            System.out.println("Auto Triggered Overdrive: Drop = " + baseError);
        }

        // === 3. 解除條件 (Deactivation) ===
        if (isOverdriveActive) {
            double timeElapsed = overdriveTimer.seconds();

            // 條件 A: 超時強制關閉
            boolean timeExpired = timeElapsed > BOOST_DURATION;

            if (timeExpired || rpmRecovered) {
                isOverdriveActive = false;
            }
        }

        // === 4. 執行輸出 ===
        // 將統一的狀態傳給 Shooter
        shooter.setOverdrive(isOverdriveActive);

        // 設定目標轉速（只設定一次！）
        if (calculatedBaseRpm > 0) {
            shooter.setFlywheel(calculatedBaseRpm);
        } else {
            shooter.setFlywheel(0);
        }

        // === 4. Hood 動態補償 ===
        double compHood = Range.clip(baseHood - (kComp * baseError), HOOD_MIN, HOOD_MAX);
        turret.setHoodPosition(baseHood);

        // === 5. 更新狀態機 ===
        if (isTriggerPulled) {
            currentStatus = SystemStatus.FIRING;
        } else if (calculatedBaseRpm == 0) {
            currentStatus = SystemStatus.IDLE;
        } else if (turretAligned && shooterRevved) {
            currentStatus = SystemStatus.READY;
        } else {
            currentStatus = SystemStatus.SEEKING;
        }
    }

    public void shootIfReady() {
        if (currentStatus == SystemStatus.READY || currentStatus == SystemStatus.FIRING) {
            isTriggerPulled = true;
        }
    }

    public void stopShoot() {
        shooter.stopShoot();
        isTriggerPulled = false;
    }

    public void stopAll() {
        stopShoot();
        shooter.setFlywheel(0);
        currentStatus = SystemStatus.IDLE;
    }

    public boolean isReady() {
        return currentStatus == SystemStatus.READY;
    }

    public SystemStatus getStatus() {
        return currentStatus;
    }

    public double getTurretRelativeTarget() {
        return turretRelativeTarget;
    }

    public void displayTelemetry(Telemetry t) {
        t.addData("Shooter Status", currentStatus);
        t.addData("Dist to Target", "%.2f in", currentDist);
        t.addData("Turret Err", "%.2f deg", turret.getError());
        t.addData("RPM Err", "%.0f", shooter.getTargetTPS() - shooter.getCurrentRPM());
        t.addData("Manual Offset", "%.1f", manualAngleOffset);
    }

    public enum SystemStatus {IDLE, SEEKING, READY, FIRING}
}