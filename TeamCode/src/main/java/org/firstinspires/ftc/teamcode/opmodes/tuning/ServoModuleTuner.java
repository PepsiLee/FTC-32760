package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.SwerveModule;

@Config
@TeleOp(name = "🔧 Servo Tuner (Step Response)", group = "Tuning")
public class ServoModuleTuner extends CommandOpMode {

    // ==================== Dashboard 設定 ====================
    public static String TARGET_MODULE = "LF"; // LF, RF, LB, RB

    // 測試角度：按下 A 鍵會切換 0度 <-> 90度
    public static double TEST_ANGLE = 90.0;

    // ==================== 內部變數 ====================
    private Robot robot;
    private GamepadEx driver;
    private double targetAngle = 0.0;

    @Override
    public void initialize() {
        robot = Robot.getInstance();
        robot.init(hardwareMap);
        driver = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 按下 A 鍵切換角度
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    if (targetAngle == 0.0) targetAngle = TEST_ANGLE;
                    else targetAngle = 0.0;
                }));

        telemetry.addLine("✅ Servo Tuner Ready");
        telemetry.addLine("Press 'A' to toggle between 0 and " + TEST_ANGLE);
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // 1. 選擇模組
        SwerveModule activeModule;
        switch (TARGET_MODULE.toUpperCase()) {
            case "RF": activeModule = robot.drive.rf; break;
            case "LB": activeModule = robot.drive.lb; break;
            case "RB": activeModule = robot.drive.rb; break;
            case "LF": default: activeModule = robot.drive.lf; break;
        }

        // 2. 設定目標 (Servo 模式不需要 PID 參數)
        activeModule.setTarget(0, targetAngle);

        // 3. 執行計算與寫入
        activeModule.prepare();
        activeModule.apply();

        // 4. 取得數據 (用於繪圖)
        double currentAngle = activeModule.currentAngle;
        double error = angleDifference(targetAngle, currentAngle);

        // 5. Dashboard 繪圖數據
        // 打開 Dashboard Graph 觀察這兩個數值
        telemetry.addData("0_Target", targetAngle);
        telemetry.addData("1_Actual", currentAngle);
        telemetry.addData("2_Error", error);

        // 除錯資訊
        telemetry.addData("Module", TARGET_MODULE);
        telemetry.addData("Servo Pos", activeModule.turnServo.getRawPosition());

        telemetry.update();

        // 更新機器人其餘部分 (如果有必要)
        // robot.updateTeleOp(); // 注意：如果你的 robot.updateTeleOp 裡有別的 Swerve 邏輯，可能會衝突，建議這裡只跑模組邏輯
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }
}