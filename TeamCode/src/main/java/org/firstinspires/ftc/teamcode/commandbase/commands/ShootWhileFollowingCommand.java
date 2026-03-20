package org.firstinspires.ftc.teamcode.commandbase.commands;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.util.PedroDrawing;

/**
 * ShootWhileFollowingCommand - 邊跟隨路徑邊射擊（只射一次）
 * <p>
 * 整合你現有的 ShooterSystem.aimAtRunning() 邏輯
 * 在路徑執行過程中瞄準並射擊一次
 * <p>
 * 使用方式：
 * new ShootWhileFollowingCommand(robot, path, goalPosition, 0.3, 0.8)
 * .schedule();
 */
public class ShootWhileFollowingCommand extends CommandBase {

    private static final String TAG = "ShootWhileFollowing";

    // ==================== 依賴注入 ====================
    private final Robot robot;
    private final PathChain path;
    private final Translation2d goalPosition;

    // ==================== 參數 ====================
    private final double startProgress;     // 開始射擊進度 (0.0 - 1.0)
    private final double endProgress;       // 停止射擊進度 (0.0 - 1.0)
    private final double maxPower; // 新增：限制最大功率 (0.0 - 1.0)
    // ==================== 狀態追蹤 ====================
    private boolean hasShot = false;        // 是否已經射過
    private boolean enteredShootingZone = false;  // 是否進入過射擊區

    /**
     * 建構子
     *
     * @param robot         Robot 實例
     * @param path          要執行的路徑
     * @param goalPosition  目標位置（籃框座標）
     * @param startProgress 開始射擊的進度 (0.0 = 起點, 1.0 = 終點)
     * @param endProgress   停止射擊的進度
     */
    public ShootWhileFollowingCommand(Robot robot, PathChain path, Translation2d goalPosition,
                                      double startProgress, double endProgress, double maxPower) {
        this.robot = robot;
        this.path = path;
        this.goalPosition = goalPosition;
        this.startProgress = startProgress;
        this.endProgress = endProgress;
        this.maxPower = maxPower; // 儲存速度限制
        addRequirements(robot.shooterSystem, robot.drive);
    }

    @Override
    public void initialize() {
        // 開始執行路徑
        robot.follower.followPath(path, true);
        robot.follower.setMaxPower(maxPower);
        // 重置狀態
        hasShot = false;
        enteredShootingZone = false;
    }

    @Override
    public void execute() {
        // ========== 1. 獲取當前狀態 ==========
        double progress = robot.follower.getCurrentTValue();
        Pose Pose = robot.follower.getPose().getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
        Pose2d robotPose = new Pose2d(Pose.getX(), Pose.getY(), Pose.getHeading());
        Vector2d velocity = new Vector2d();
        // ========== 2. 根據進度執行不同邏輯 ==========
        if (progress >= startProgress && progress <= endProgress) {
            // === 射擊區間 ===
            if (!enteredShootingZone) {
                enteredShootingZone = true;
                Log.d(TAG, "Entered shooting zone at " + String.format("%.1f%%", progress * 100));
            }
            executeShootingPhase(robotPose, velocity);
        } else if (progress < startProgress) {
            executePreparePhase(robotPose, velocity);
        } else {
            executeEndPhase();
        }
    }

    /**
     * 準備階段 - 提前啟動射擊系統
     */
    private void executePreparePhase(Pose2d robotPose, Vector2d velocity) {
        // 使用 ShooterSystem 的 aimAtRunning 計算瞄準
        robot.shooterSystem.aimAtRunning(robotPose, velocity, goalPosition);
        robot.intake.setState(Intake.IntakeState.STOP);
    }

    /**
     * 射擊階段 - 持續瞄準，只射擊一次
     */
    private void executeShootingPhase(Pose2d robotPose, Vector2d velocity) {
        // 1. 持續更新瞄準（即使已經射過，也要保持瞄準）
        robot.shooterSystem.aimAtRunning(robotPose, velocity, goalPosition);
        System.out.println("[ShootOnPath] ShooterSystem :" + robot.shooterSystem.isReady() + String.format("%.1f%%", robot.follower.getCurrentTValue() * 100));
        // 2. 只在第一次進入射擊區且準備好時射擊
        if (!hasShot) {
            robot.shooterSystem.shootIfReady();
            robot.intake.setState(Intake.IntakeState.INTAKE);
            hasShot = true;
            Log.i(TAG, "Shot fired at " + String.format("%.1f%%", robot.follower.getCurrentTValue() * 100));
        }
    }

    /**
     * 結束階段 - 停止射擊
     */
    private void executeEndPhase() {
        robot.shooterSystem.stopShoot();
        robot.intake.setState(Intake.IntakeState.STOP);
    }

    @Override
    public boolean isFinished() {
        return !robot.isFollowerBusy();
    }

    @Override
    public void end(boolean interrupted) {
        // 停止所有射擊相關系統
        robot.shooterSystem.stopShoot();
        robot.intake.setState(Intake.IntakeState.STOP);

        // 輸出統計
        Log.i(TAG, "ShootWhileFollowing completed - Shot fired: " + hasShot +
                ", Interrupted: " + interrupted);

        if (interrupted) {
            robot.follower.breakFollowing();
            Log.w(TAG, "Path following was interrupted");
        }
    }

    // ==================== Getters（用於 Telemetry）====================

    public double getCurrentProgress() {
        return robot.follower.getCurrentTValue();
    }

    public boolean isInShootingZone() {
        double progress = robot.follower.getCurrentTValue();
        return progress >= startProgress && progress <= endProgress;
    }

    public boolean hasShot() {
        return hasShot;
    }
}