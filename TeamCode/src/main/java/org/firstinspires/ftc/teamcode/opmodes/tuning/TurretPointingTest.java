    package org.firstinspires.ftc.teamcode.opmodes.tuning;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.seattlesolvers.solverslib.gamepad.GamepadEx;
    import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

    import org.firstinspires.ftc.teamcode.Robot;
    import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
    import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
    import org.firstinspires.ftc.teamcode.commandbase.subsystems.turret.Turret;

    @Config
    @TeleOp(name = "Turret Pointing Test (PID Tuning)", group = "Test")
    public class TurretPointingTest extends LinearOpMode {

        // 可在 Dashboard 直接修改目標角度
        public static double TARGET_ANGLE = 0.0;

        // 用於產生階梯訊號測試的參數
        public static double STEP_SIZE = 45.0;
        private GamepadEx driverOp;
        private IMU imu;

        @Override
        public void runOpMode() throws InterruptedException {
            // 初始化 Telemetry，同時傳送到 Driver Station 和 Dashboard
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            imu = new IMU(hardwareMap);
            VoltageMonitor voltageMonitor = new VoltageMonitor(hardwareMap);
            Turret turret = new Turret(hardwareMap, voltageMonitor, imu);
            driverOp = new GamepadEx(gamepad1);
            Robot.getInstance().init(hardwareMap);

            telemetry.addLine("=== 砲塔指向測試準備就緒 ===");
            telemetry.addLine("操作說明：");
            telemetry.addLine("1. 使用 Dashboard 修改 TARGET_ANGLE 數值");
            telemetry.addLine("2. 或使用手把 D-Pad 快速切換預設角度");
            telemetry.addLine("   上: 0度, 右: +90度, 左: -90度, 下: 180度(後方)");
            telemetry.addLine("3. 按下 A / B 鍵可增加/減少目前的目標角度 (微調)");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                driverOp.readButtons();
                voltageMonitor.periodic();

                // === 手把快捷控制 ===
                // D-Pad 快速定位
                if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP))    TARGET_ANGLE = 0;
                if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) TARGET_ANGLE = 90;
                if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))  TARGET_ANGLE = -90;
                if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))  TARGET_ANGLE = 180;

                // A/B 鍵做階梯測試 (Step Response)
                if (driverOp.wasJustPressed(GamepadKeys.Button.A)) TARGET_ANGLE += STEP_SIZE;
                if (driverOp.wasJustPressed(GamepadKeys.Button.B)) TARGET_ANGLE -= STEP_SIZE;

                // === 核心邏輯更新 ===
                // 設定目標並執行 PID 計算
                turret.setTargetAngle(TARGET_ANGLE);
                turret.update();

                // === Telemetry 與 Dashboard 繪圖數據 ===
                // 在 Dashboard 上勾選這兩個變數，觀察它們的曲線重合度
                telemetry.addData("Target Angle", TARGET_ANGLE);
                telemetry.addData("Current Angle", turret.getAngle());

                // 輔助診斷數據
                telemetry.addData("Error", turret.getError());

                // 3. 輸出監控
                telemetry.addData("Out/Error", turret.getError());
                telemetry.addLine("--- Active PIDF ---");
                telemetry.update();
                Robot.getInstance().updateTeleOp();
            }
        }
    }