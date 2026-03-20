package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry; // 引入正確的 Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.constant.Constant;

import java.lang.reflect.Method;
import java.util.List;

public class Vision extends SubsystemBase {

    private static final double METERS_TO_INCHES = 39.3700787;
    // 狀態變數
    public boolean hasTarget = false;
    public double tx = 0;       // 水平角度誤差 (度)
    public double ty = 0;       // 垂直角度誤差 (度)
    public double distance = 0; // 計算出的距離 (由三角函數算出)
    public double distance3D = 0; // 由 3D Pose 算出的距離 (Z軸)
    public int targetID = -1;   // 目前鎖定的 ID
    public Pose3D targetPose = null; // 目前鎖定的 Pose
    private Limelight3A limelight;
    private Method getBotposeMT1Method;
    private boolean mt1MethodChecked = false;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(200);
        limelight.pipelineSwitch(Constant.ALLIANCE_COLOR == Constant.AllianceColor.BLUE ? 1 : 0);
        limelight.start();
    }

    public void switchPipeline(){
        limelight.pipelineSwitch(Constant.ALLIANCE_COLOR == Constant.AllianceColor.BLUE ? 1 : 0);
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        // 1. 重置狀態
        hasTarget = false;

        // 2. 安全檢查
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            LLResultTypes.FiducialResult bestTag = null;

            // 3. 搜尋目標 ID
            for (LLResultTypes.FiducialResult tag : fiducials) {
                if (targetID == -1 || tag.getFiducialId() == targetID) {
                    bestTag = tag;
                    break;
                }
            }

            // 4. 如果找到了，更新數據
            if (bestTag != null) {
                hasTarget = true;
                targetID = bestTag.getFiducialId();

                // 這是重點：獲取機器人空間下的 Pose
                Pose3D targetPose = bestTag.getTargetPoseRobotSpace();

                // 單位通常是公尺 (Meters)，如果要顯示英吋需要轉換 ( * 39.37)
                double x_m = targetPose.getPosition().x; // 上下偏移
                double y_m = targetPose.getPosition().y; // 左右偏移
                double z_m = targetPose.getPosition().z; // 距離

                // 獲取旋轉角度 (Yaw/Heading) - 這是物體相對於你的面的角度
                double yaw_deg = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);

                // 2. 計算平面距離 (Distance)
                distance = Math.hypot(x_m, y_m);
                // 3. 計算 3D 距離 (包含高度差)
                distance3D = Math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m);

                // 1. 計算 tx (Steer Angle)：利用 atan2 計算這一點相對車頭的角度
                // 當 y 為 0 時角度為 0，適合用來做 PID 轉向對準
                this.tx = result.getTx();
                this.ty = result.getTy();

            }
        }

        if (!hasTarget) {
            tx = 0;
        }
    }

    /**
     * 使用 MegaTag 2 獲取機器人位置
     * @param imuHeadingRobotDegrees 機器人目前的 IMU 角度 (度)，必須是 Field-Centric 的
     * @return Pose2d (Inches)
     */
    public Pose2d getRobotPoseMT2(double imuHeadingRobotDegrees) {
        limelight.updateRobotOrientation(imuHeadingRobotDegrees);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose3D = result.getBotpose_MT2();

            if (botPose3D != null) {
                // 3. 單位轉換 (Meters -> Inches)
                double x_inch = botPose3D.getPosition().x * METERS_TO_INCHES;
                double y_inch = botPose3D.getPosition().y * METERS_TO_INCHES;

                double heading_deg = botPose3D.getOrientation().getYaw(AngleUnit.DEGREES);

                return new Pose2d(x_inch, y_inch, Rotation2d.fromDegrees(heading_deg));
            }
        }
        return null;
    }

    /**
     * MegaTag2 提供位置，MegaTag1(如果存在)提供 Heading
     * 若 MT1 不可用，會回退到 IMU Heading
     */
    public Pose2d getRobotPoseMT2WithHeading(double imuHeadingRobotDegrees) {
        limelight.updateRobotOrientation(imuHeadingRobotDegrees);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        Pose3D botPoseMT2 = result.getBotpose_MT2();
        if (botPoseMT2 == null) {
            return null;
        }

        Double mt1HeadingDeg = tryGetMT1HeadingDegrees(result);
        double headingDeg = (mt1HeadingDeg != null) ? mt1HeadingDeg : imuHeadingRobotDegrees;

        double x_inch = botPoseMT2.getPosition().x * METERS_TO_INCHES;
        double y_inch = botPoseMT2.getPosition().y * METERS_TO_INCHES;

        return new Pose2d(x_inch, y_inch, Rotation2d.fromDegrees(headingDeg));
    }

    private Double tryGetMT1HeadingDegrees(LLResult result) {
        Pose3D poseMT1 = tryGetBotposeMT1(result);
        if (poseMT1 == null) {
            return null;
        }
        return poseMT1.getOrientation().getYaw(AngleUnit.DEGREES);
    }

    private Pose3D tryGetBotposeMT1(LLResult result) {
        if (!mt1MethodChecked) {
            mt1MethodChecked = true;
            try {
                getBotposeMT1Method = result.getClass().getMethod("getBotpose_MT1");
            } catch (NoSuchMethodException e) {
                getBotposeMT1Method = null;
            }
        }

        if (getBotposeMT1Method == null) {
            return null;
        }

        try {
            Object pose = getBotposeMT1Method.invoke(result);
            if (pose instanceof Pose3D) {
                return (Pose3D) pose;
            }
        } catch (Exception ignored) {
            return null;
        }

        return null;
    }

    public void setTargetID(int id) {
        targetID = id;
    }

    public double getTx(double kp) {
        return tx * kp;
    }

    public double getTy(double kp) {
        return ty * kp;
    }

    public double getDistance(){
        return distance;
    }


    @Override
    public void periodic() {
        super.periodic();
        update();
    }
}
