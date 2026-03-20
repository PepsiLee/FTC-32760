package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Shooter Tuner (RPM/Hood)", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    // ==================== Dashboard 可調參數 ====================
    // 這些變數可以在 FTC Dashboard (192.168.43.1:8080) 上直接修改
    public static double targetRPM = 1500;
    public static double targetHood = 0.45; // 假設 Hood 是 Servo 位置 (0.0 - 1.0)
    public static double currentDistance = 72.0; // 當前測試距離 (inch)

    // 微調步進值
    public static double RPM_STEP = 50;
    public static double HOOD_STEP = 0.01;

    // ==================== 硬體與狀態 ====================
    private Robot robot;
    private ElapsedTime buttonTimer = new ElapsedTime();
    private List<String> dataLog = new ArrayList<>(); // 儲存測試成功的數據

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 初始化 Robot
        robot = Robot.getInstance();
        robot.init(hardwareMap);

        // 2. 設定 Telemetry (同時顯示在 Driver Station 和 Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 3. 啟動非同步更新 (如果你的 Robot 架構需要)
        // robot.start();

        telemetry.addLine("✅ Shooter Tuner Ready");
        telemetry.addLine("按 'Start' 清除數據");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ==================== 1. 手把控制 (微調) ====================
            handleInput();

            // ==================== 2. 執行硬體動作 ====================

            // 設定飛輪速度
            robot.shooter.setFlywheel(targetRPM);

            robot.turret.setHoodPosition(targetHood);

            // 確保 Turret 指向正前方 (避免干擾測試)
            robot.turret.setTargetAngle(0);

            // 定期更新 Robot 的所有子系統 (PID loop 等)
            robot.updateTeleOp();

            // ==================== 3. Telemetry 顯示 ====================
            updateTelemetry();
        }
    }

    private void handleInput() {
        // 防止按鍵連點 (Debounce)
        if (buttonTimer.milliseconds() < 150) return;

        // --- RPM 調整 (Dpad 上下) ---
        if (gamepad1.dpad_up) {
            targetRPM += RPM_STEP;
            buttonTimer.reset();
        } else if (gamepad1.dpad_down) {
            targetRPM -= RPM_STEP;
            buttonTimer.reset();
        }

        // --- Hood 調整 (Dpad 左右) ---
        if (gamepad1.dpad_right) {
            targetHood += HOOD_STEP;
            buttonTimer.reset();
        } else if (gamepad1.dpad_left) {
            targetHood -= HOOD_STEP;
            buttonTimer.reset();
        }

        // --- 距離標記調整 (Y/A) - 方便記錄 ---
        if (gamepad1.y) {
            currentDistance += 12; // 加 1 foot
            buttonTimer.reset();
        } else if (gamepad1.a) { // 注意：A 鍵通常也用於射擊，這裡分開處理
            // 如果 A 鍵被用於射擊，這裡可以用 gamepad1.x 或其他鍵
        }

        // --- 發射動作 (右扳機 / B 鍵) ---
        if (gamepad1.right_bumper) {
            robot.intake.setState(Intake.IntakeState.INTAKE); // 推彈
        } else {
            robot.intake.setState(Intake.IntakeState.STOP);
        }

        // --- 記錄數據 (X 鍵) ---
        // 當你覺得這一球射得很準時，按下 X
        if (gamepad1.x) {
            String logEntry = String.format("add(%.1f, %.0f, %.3f);", currentDistance, targetRPM, targetHood);
            dataLog.add(logEntry);
            gamepad1.rumble(500); // 震動回饋
            buttonTimer.reset();
        }

        // --- 清除數據 (Start 鍵) ---
        if (gamepad1.start) {
            dataLog.clear();
            buttonTimer.reset();
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== 🎯 Shooter Tuner ===");
        telemetry.addLine("使用 Dashboard 或 Dpad 調整參數");
        telemetry.addLine("按 'RB' 射擊, 按 'X' 記錄數據");
        telemetry.addLine();

        telemetry.addData("📏 Distance (in)", currentDistance);
        telemetry.addData("🚀 Target RPM", targetRPM);
        telemetry.addData("🔧 Target Hood", "%.3f", targetHood);

        // 顯示實際 RPM (確認 PID 是否穩定)
        double actualRPM = robot.shooter.getCurrentRPM(); // 根據你的方法名稱修改
        telemetry.addData("📊 Actual RPM", "%.1f", actualRPM);
        telemetry.addData("Error", "%.1f", targetRPM - actualRPM);

        telemetry.addLine();
        telemetry.addLine("=== 📝 Recorded Data (Copy This) ===");
        for (String log : dataLog) {
            telemetry.addLine(log);
        }

        telemetry.update();
    }
}