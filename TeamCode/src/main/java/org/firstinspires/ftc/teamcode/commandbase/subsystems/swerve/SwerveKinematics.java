package org.firstinspires.ftc.teamcode.commandbase.subsystems.swerve;

import org.firstinspires.ftc.teamcode.constant.SwerveConstants;


public class SwerveKinematics {
    private static SwerveConstants constants;

    /**
     * 初始化 Kinematics（必須在使用前調用）
     */
    public static void initialize(SwerveConstants swerveConstants) {
        constants = swerveConstants;
    }

    /**
     * 計算四個 Swerve 模組的目標狀態
     * @param xChassis X 軸速度（前後）
     * @param yChassis Y 軸速度（左右）
     * @param rotChassis 旋轉速度（度/秒或弧度/秒）
     * @param botHeading 機器人當前朝向（用於 Field Centric）
     * @return 四個輪子的狀態陣列 [LF, RF, LB, RB]
     */
    public static SwerveModuleState[] calculateModuleStates(
            double xChassis, double yChassis, double rotChassis, double botHeading) {

        if (constants == null) {
            throw new RuntimeException("SwerveKinematics not initialized! Call initialize() first.");
        }


        // ========== A. 座標轉換 (Field Centric) ==========
        double rotX = xChassis * Math.cos(-botHeading) - yChassis * Math.sin(-botHeading);
        double rotY = xChassis * Math.sin(-botHeading) + yChassis * Math.cos(-botHeading);

        // ========== B. 逆運動學計算 (Swerve Math) ==========
        double TRACK_WIDTH = constants.TRACK_WIDTH;
        double WHEEL_BASE = constants.WHEEL_BASE;

        double R_val = Math.hypot(TRACK_WIDTH, WHEEL_BASE);
        double rx = rotChassis * (TRACK_WIDTH / R_val);
        double ry = rotChassis * (WHEEL_BASE / R_val);

        // ========== C. 計算四個輪子的向量 (LF, RF, LB, RB) ==========
        double[] vx = {
                rotX - rx,  // LF
                rotX + rx,  // RF
                rotX - rx,  // LB
                rotX + rx   // RB
        };

        double[] vy = {
                rotY + ry,  // LF
                rotY + ry,  // RF
                rotY - ry,  // LB
                rotY - ry   // RB
        };

        SwerveModuleState[] states = new SwerveModuleState[4];

        // ========== D. 計算每個輪子的速度和角度 ==========
        for (int i = 0; i < 4; i++) {
            // 計算速度大小
            double speed = Math.hypot(vx[i], vy[i]);

            double angleRadians = Math.atan2(vy[i], vx[i]);
            double angleDegrees = Math.toDegrees(angleRadians);

            states[i] = new SwerveModuleState(speed, angleDegrees);
        }

        return states;
    }
}