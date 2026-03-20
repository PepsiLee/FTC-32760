package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constant.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.constant.Constants.localizerConstants;
import static org.firstinspires.ftc.teamcode.constant.Constants.pathConstraints;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.PoseFusion;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Rgb;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.constant.SwerveConstants;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

/**
 * Robot 容器類別 - 集中管理所有子系統
 * <p>
 * 使用 Singleton 模式，確保 TeleOp 和 Autonomous 使用相同的硬體實例
 * 整合 Command-Based 架構和 Pedro Pathing
 */
public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    // ==================== Singleton ====================/
    private static final Robot instance = new Robot();
    public Profiler profiler;
    public File file;
    // ==================== 子系統 ====================
    public VoltageMonitor voltageMonitor;
    public SwerveDrive drive;
    public Intake intake;
    public Turret turret;
    public Shooter shooter;
    public ShooterSystem shooterSystem;
    public Vision vision;
    public IMU imu;
    public PoseFusion poseFusion;
    public Rgb rgb;
    // ==================== Pedro Pathing ====================
    public Follower follower;
    // ==================== 常數 ====================
    public SwerveConstants swerveConstants;
    // ==================== 硬體管理 ====================
    private HardwareMap hardwareMap;
    private List<LynxModule> allHubs;
    // ==================== 狀態標記 ====================
    private boolean isInitialized = false;
    private boolean autoPoseSynced = false;
    private final ElapsedTime autoVisionTimer = new ElapsedTime();
    private static final double AUTO_VISION_MIN_INTERVAL_MS = 50.0;

    /**
     * 私有建構子（Singleton 模式）
     */
    private Robot() {
    }

    public static Robot getInstance() {
        return instance;
    }

    /**
     * 初始化所有子系統和硬體
     *
     * @param hardwareMap FTC HardwareMap
     */
    public void init(HardwareMap hardwareMap) {
        if (isInitialized) {
            cleanup();
        }

        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");

        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false) // Log EVERYTHING
                .build();

        this.hardwareMap = hardwareMap;

        // 1. 設定 Swerve 常數
        this.swerveConstants = new SwerveConstants();
        swerveConstants.defaults();

        // 2. 設定 Bulk Reading（提升效能）
        setupBulkReading();

        // 3. 初始化基礎子系統
        initializeBaseSubsystems();

        // 4. 初始化遊戲子系統
        initializeGameSubsystems();

        // 5. 初始化 Pedro Pathing
        initializePedroPathing();

        // 6. 註冊子系統到 CommandScheduler
        registerSubsystems();

        isInitialized = true;
        autoPoseSynced = false;
    }

    /**
     * 設定 Bulk Reading 模式
     */
    private void setupBulkReading() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * 初始化基礎子系統（電壓監控、IMU、底盤）
     */
    private void initializeBaseSubsystems() {
        voltageMonitor = new VoltageMonitor(hardwareMap);
        imu = new IMU(hardwareMap);
        drive = new SwerveDrive(hardwareMap, voltageMonitor, swerveConstants);
        rgb = new Rgb(hardwareMap);
    }

    /**
     * 初始化遊戲子系統（射擊、進球、視覺）
     */
    private void initializeGameSubsystems() {
        turret = new Turret(hardwareMap, voltageMonitor, imu);
        shooter = new Shooter(hardwareMap, voltageMonitor);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);
        poseFusion = new PoseFusion(imu, vision);
        shooterSystem = new ShooterSystem(shooter, turret);
    }

    /**
     * 初始化 Pedro Pathing Follower
     */
    private void initializePedroPathing() {
        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(drive)  // SwerveDrive
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

    /**
     * 註冊所有子系統到 CommandScheduler
     */
    private void registerSubsystems() {
        register(drive, intake, turret, shooter, shooterSystem);
    }

    // ==================== TeleOp 方法 ====================

    /**
     * TeleOp 更新循環
     * 包含 CommandScheduler 和 Bulk Cache 清理
     */
    public void updateTeleOp() {
        // 更新 IMU
        imu.update();
        vision.update();
        if (poseFusion != null) {
            poseFusion.update();
        }
    }

    /**
     * 重置機器人朝向（TeleOp 用）
     */
    public void resetIMU() {
        imu.resetIMU();
        imu.resetHeading();
    }

    /**
     * 獲取當前機器人位置（TeleOp 用）
     */
    public Pose2d getPose() {
        if (poseFusion != null) {
            return poseFusion.getPose();
        }
        return imu.getPose();
    }

    public void setPose(Pose2d pose) {
        imu.setPose(pose);
        if (poseFusion != null) {
            poseFusion.reset(pose);
        }
    }

    // ==================== Autonomous 方法 ====================

    /**
     * Autonomous 更新循環
     * 包含 Pedro Pathing 更新、CommandScheduler 和 Bulk Cache 清理
     */
    public void updateAuto() {
        // 更新 Pedro Pathing
        follower.update();
        // 更新 IMU + Vision (讓 Auto 也能使用 Vision Pose)
        imu.update();
        vision.update();

        if (poseFusion != null) {
            if (!autoPoseSynced) {
                Pose pPose = follower.getPose();
                Pose2d startPose = new Pose2d(pPose.getX(), pPose.getY(), pPose.getHeading());
                poseFusion.reset(startPose);
                autoPoseSynced = true;
            }
            poseFusion.update();
            updateFollowerWithVision();
        }
    }

    private void updateFollowerWithVision() {
        if (autoVisionTimer.milliseconds() < AUTO_VISION_MIN_INTERVAL_MS) {
            return;
        }
        autoVisionTimer.reset();

        Pose2d fusedPose = poseFusion.getPose();
        Pose followerPose = follower.getPose();

        double dist = Math.hypot(
                fusedPose.getX() - followerPose.getX(),
                fusedPose.getY() - followerPose.getY()
        );

        if (dist > org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.PoseFusion.VISION_MAX_DISTANCE_IN) {
            return;
        }

        double newX = lerp(followerPose.getX(), fusedPose.getX(),
                org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.PoseFusion.VISION_BLEND);
        double newY = lerp(followerPose.getY(), fusedPose.getY(),
                org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.PoseFusion.VISION_BLEND);
        double newHeading = followerPose.getHeading();

//        Pose correctedInv = new Pose(newX, newY, newHeading, InvertedFTCCoordinates.INSTANCE);
//        Pose correctedPedro = correctedInv.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        follower.setPose(new Pose(newX, newY, newHeading, PedroCoordinates.INSTANCE));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }


    /**
     * 設定 Autonomous 起始位置
     *
     * @param startPose 起始位置（Pedro Pathing Pose）
     */
    public void setStartingPose(Pose startPose) {
        follower.setStartingPose(startPose);
    }

    /**
     * 獲取當前機器人位置（Autonomous 用，Pedro Pathing）
     */
    public Pose getAutoPose() {
        return follower.getPose();
    }

    /**
     * 檢查 Follower 是否正在執行路徑
     */
    public boolean isFollowerBusy() {
        return follower.isBusy();
    }

    // ==================== 工具方法 ====================

    /**
     * 清除所有 Hub 的 Bulk Cache
     */
    public void clearBulkCache() {

        if (allHubs != null) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
    }

    /**
     * 停止所有子系統
     */
    public void stopAll() {
        drive.stop();
        shooterSystem.stopAll();
        intake.setState(Intake.IntakeState.STOP);
    }

    /**
     * 緊急停止 - 取消所有 Command 並停止所有子系統
     */
    public void emergencyStop() {
        CommandScheduler.getInstance().cancelAll();
        stopAll();
    }

    /**
     * 清理資源（在重新初始化前呼叫）
     */
    private void cleanup() {
        if (CommandScheduler.getInstance() != null) {
            CommandScheduler.getInstance().cancelAll();
        }
        CommandScheduler.getInstance().reset();
        stopAll();
        isInitialized = false;
        autoPoseSynced = false;
    }

    /**
     * 檢查是否已初始化
     */
    public boolean isInitialized() {
        return isInitialized;
    }


    public void exportProfiler(File file) {
        RobotLog.i("Starting async profiler export to: " + file.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                Log.e("An error occurred", e.toString());
                Log.e(e.toString(), Arrays.toString(e.getStackTrace()));
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }
}
