package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

@Config
public class SwerveStatus {

    // 1. 定義單一模組的數據結構 (必須是 public static 才能讓 Dashboard 讀取內部欄位)
    public static class SwerveModuleState {
        public Rotation2d angle = Rotation2d.fromDegrees(0);
        public double speedMetersPerSecond = 0; // Inches
    }

    // 2. 宣告四個模組的狀態物件 (Dashboard 會顯示為 LF, RF... 的資料夾)
    public static SwerveModuleState LF = new SwerveModuleState();
    public static SwerveModuleState RF = new SwerveModuleState();
    public static SwerveModuleState LB = new SwerveModuleState();
    public static SwerveModuleState RB = new SwerveModuleState();
    public static SwerveModuleState[] moduleStates = {LF, RF, LB, RB};
    /**
     * 更新所有數據的方法
     */
    public static void record(SwerveModuleState lf, SwerveModuleState rf, SwerveModuleState lb, SwerveModuleState rb) {
        updateModule(LF, lf);
        updateModule(RF, rf);
        updateModule(LB, lb);
        updateModule(RB, rb);
    }

    // 輔助方法：把 SwerveModule 的數據填入 ModuleData 物件
    private static void updateModule(SwerveModuleState data, SwerveModuleState module) {
        data.angle = module.angle;
        data.speedMetersPerSecond = module.speedMetersPerSecond;
    }
}