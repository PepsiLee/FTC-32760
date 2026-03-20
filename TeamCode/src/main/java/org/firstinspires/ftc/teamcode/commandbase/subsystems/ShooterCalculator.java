package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT; // 確保引用正確的 Library

import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.ShotState;
import org.firstinspires.ftc.teamcode.util.InterpolationLUT;

import java.util.Arrays;

public class ShooterCalculator {
    private final InterpolationLUT rpmLUT;
    private final InterpolationLUT hoodLUT;
    private final InterpolationLUT timeLUT;

    // === [新增] 進球角度補償 LUT ===
    public InterpolationLUT goalAdjustmentLUT;

    public ShooterCalculator() {
        goalAdjustmentLUT = new InterpolationLUT("GOAL");
        rpmLUT = new InterpolationLUT("RPM");
        hoodLUT = new InterpolationLUT("Hood");
        timeLUT = new InterpolationLUT("Time");


        //TODO: TEST IF NO COMPENSATE IS BETTER
//        //跟對邊牆夾角
//        // -90度
        goalAdjustmentLUT.add(-Math.PI / 2, 0);
//        goalAdjustmentLUT.add(-0.94,-1.0);
//
//        // --- 過渡區 ---
//        goalAdjustmentLUT.add(-0.9, 0.0);
//        goalAdjustmentLUT.add(-Math.PI / 4, 0.0);  // -45度
//
//        // 0度
        goalAdjustmentLUT.add(0, 0); // 0度


        // 實測數據 (inch, RPM, Hood, 飛行秒數)
        add(30, 900, 0, 0.25);   // 原 RPM: 2000
        add(50, 950, 0, 0.25);   // 原 RPM: 2100
        add(60, 980, 1, 0.5);   // 原 RPM: 2200
        add(70, 1070, 1, 0.6);   // 原 RPM: 2400
        add(80, 1100, 1, 0.6);   // 原 RPM: 2500
        add(90, 1150, 1, 0.7);   // 原 RPM: 2600
        add(100, 1210, 1, 0.8);  // 原 RPM: 2700
        add(110, 1250, 1, 0.8);  // 原 RPM: 2800
        add(120, 1300, 1, 0.8);  // 原 RPM: 2850
        add(130, 1350, 1, 0.8);  // 原 RPM: 3000
        add(140, 1400, 1, 0.8);  // 原 RPM: 3200
        add(150, 1450, 1, 0.9);  // 原 RPM: 3300
        add(160, 1500, 1, 0.9);  // 原 RPM: 3400
        add(170, 1550, 1, 0.9);  // 原 RPM: 3500
    }

    public void add(double distance, double rpm, double hoodAngle, double timeOfFlight) {
        rpmLUT.add(distance, rpm);
        hoodLUT.add(distance, hoodAngle);
        timeLUT.add(distance, timeOfFlight);
    }

    /**
     * [新增] 計算經過角度補償後的「虛擬目標位置」
     * * @param robotPose 機器人當前位置
     *
     * @param originalTarget     原始目標位置 (籃框中心)
     * @param allianceMultiplier 聯盟顏色係數 (Blue: 1, Red: -1)，用於處理鏡像場地
     * @return 修正後的目標位置 (Vector2d)
     */
    public Vector2d getAdjustedTarget(Pose2d robotPose, Vector2d originalTarget, double allianceMultiplier) {
        // 1. 計算機器人到目標的向量角度 (場地座標系)
        double angleToGoal = Math.atan2(
                originalTarget.getY() - robotPose.getY(),
                originalTarget.getX() - robotPose.getX()
        );

        // 2. 計算與後牆垂直線的夾角 (假設後牆是 90度/PI/2 方向)
        // 這是計算機器人相對於正對籃框偏了多少
        double angleToWall = (Math.PI / 2.0) + angleToGoal * allianceMultiplier;

        // 4. 查表取得修正量 (Inches)
        double adjustment = goalAdjustmentLUT.get(angleToWall);

        // 5. 應用修正量
        Vector2d adjustedTarget;

        if (adjustment < 0) {
            adjustedTarget = new Vector2d(
                    originalTarget.getX(),
                    originalTarget.getY() - (adjustment * allianceMultiplier)
            );
        } else {
            adjustedTarget = new Vector2d(
                    originalTarget.getX() + adjustment,
                    originalTarget.getY()
            );
        }

        return adjustedTarget;
    }

    public ShotState calculate(double distance, double radialVel, double tangentialVel) {
        // 1. 取得飛行時間
        double flightTime = timeLUT.get(distance);

        // 2. 徑向補償 (前後移動 -> 影響有效距離)
        double radialCorrection = radialVel * flightTime;
        double effectiveDistance = distance - radialCorrection;

        double targetRPM = rpmLUT.get(effectiveDistance);
        double targetHood = hoodLUT.get(effectiveDistance);

        // 3. 切向補償 (左右移動 -> 影響砲塔角度)
        double lateralShift = tangentialVel * flightTime;

        double correctionDeg = Math.toDegrees(Math.atan2(lateralShift, effectiveDistance));
        double turretOffset = -correctionDeg;

        return new ShotState(targetRPM, targetHood, turretOffset);
    }
}