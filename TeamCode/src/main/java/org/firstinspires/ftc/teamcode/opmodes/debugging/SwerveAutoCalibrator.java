package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve.SwerveModule;

/*
 * 【使用說明】
 * 1. 啟動後，四個輪子會先回到 0.5 的位置。
 * 2. 使用 Gamepad1 的 Dpad 來選擇要調整的輪子 (Left Stick 微調位置)。
 * 3. 調整到輪子完全「筆直朝前」後，按下 Gamepad1.X 鎖定數值。
 * 4. 抄下 Telemetry 顯示的 Bias 與 Offset。
 */
@Config
@TeleOp(name = "Swerve Auto-Calibrator")
public class SwerveAutoCalibrator extends LinearOpMode {

    // 暫存四個模組的校準位置 (預設中點)
    public static double lfPos = 0.5, rfPos = 0.5, lbPos = 0.5, rbPos = 0.5;
    private int selectedPod = 0; // 0:LF, 1:RF, 2:LB, 3:RB

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 初始化硬體 (請對應你的 Config 名稱)
        ServoEx[] servos = {
                new ServoEx(hardwareMap, "fls", 0, SwerveModule.SERVO_MAX_ANGLE),
                new ServoEx(hardwareMap, "frs", 0, SwerveModule.SERVO_MAX_ANGLE),
                new ServoEx(hardwareMap, "rls", 0, SwerveModule.SERVO_MAX_ANGLE),
                new ServoEx(hardwareMap, "rrs", 0, SwerveModule.SERVO_MAX_ANGLE)
        };

        AbsoluteAnalogEncoder[] encoders = {
                new AbsoluteAnalogEncoder(hardwareMap, "fle"),
                new AbsoluteAnalogEncoder(hardwareMap, "fre"),
                new AbsoluteAnalogEncoder(hardwareMap, "rle"),
                new AbsoluteAnalogEncoder(hardwareMap, "rre")
        };

        waitForStart();

        while (opModeIsActive()) {
            // A. 選擇輪子
            if (gamepad1.dpad_up) selectedPod = 0;
            if (gamepad1.dpad_right) selectedPod = 1;
            if (gamepad1.dpad_down) selectedPod = 2;
            if (gamepad1.dpad_left) selectedPod = 3;

            // B. 微調當前選擇的輪子位置 (Left Stick Y 軸)
            double adjustment = -gamepad1.left_stick_y * 0.0005;
            if (selectedPod == 0) lfPos += adjustment;
            if (selectedPod == 1) rfPos += adjustment;
            if (selectedPod == 2) lbPos += adjustment;
            if (selectedPod == 3) rbPos += adjustment;

            // C. 即時輸出到硬體
            servos[0].set(lfPos);
            servos[1].set(rfPos);
            servos[2].set(lbPos);
            servos[3].set(rbPos);

            // D. 顯示校準數據
            String[] names = {"LF", "RF", "LB", "RB"};
            double[] currentPositions = {lfPos, rfPos, lbPos, rbPos};

            telemetry.addLine("--- Swerve Calibration Tool ---");
            telemetry.addData("Selected Pod", names[selectedPod]);
            telemetry.addLine("使用 Dpad 選擇輪子, Left Stick 微調位置");
            telemetry.addLine("當輪子筆直朝前時，記錄以下數據：");
            telemetry.addLine("--------------------------------");

            for (int i = 0; i < 4; i++) {
                // 這裡讀取的 Encoder 角度就是你未來的 OFFSET
                double rawAngle = encoders[i].getCurrentPosition();
                telemetry.addData(names[i] + " BIAS (Servo Pos)", "%.4f", currentPositions[i]);
                telemetry.addData(names[i] + " OFFSET (Encoder)", "%.2f", rawAngle);
            }

            telemetry.update();
        }
    }
}
