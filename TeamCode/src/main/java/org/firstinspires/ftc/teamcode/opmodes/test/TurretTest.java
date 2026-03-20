package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.turret.Turret;

@Disabled
@Config
@TeleOp(name = "Turret Diagnostic & Calibration", group = "Test")
public class TurretTest extends LinearOpMode {

    public static double TEST_TARGET = 0.0;
    public static boolean ENABLE_CONTROL = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VoltageMonitor voltageMonitor = new VoltageMonitor(hardwareMap);
        IMU imu = new IMU(hardwareMap);
        Turret turret = new Turret(hardwareMap, voltageMonitor, imu);

        telemetry.addLine("=== 砲塔校準模式 ===");
        telemetry.addLine("手動轉動砲塔，紀錄 Raw 角度以設定 OFFSET");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 更新電壓與砲塔邏輯
            voltageMonitor.periodic();

            if (ENABLE_CONTROL) {
                turret.setTargetAngle(TEST_TARGET);
                turret.update();
            } else {
                // 測試模式下關閉動力，方便手動轉動校準
//                turret.getServoL().setPower(0);
//                turret.getServoR().setPower(0);
            }

            telemetry.addData("1. Status", ENABLE_CONTROL ? "PID ACTIVE" : "MANUAL/COAST");
            telemetry.addData("2. Target Angle", TEST_TARGET);
            telemetry.addData("3. Current Angle", turret.getAngle());
            telemetry.addData("4. Error", turret.getError());

            telemetry.update();
        }
    }
}