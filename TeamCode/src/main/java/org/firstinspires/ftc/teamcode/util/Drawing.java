package org.firstinspires.ftc.teamcode.util; // 請依照你的專案路徑修改 package

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.constant.Constant;

/**
 * 通用版 Dashboard 繪圖工具
 * 移除特定的 Path/Follower 依賴，改用基本座標輸入。
 */


public class Drawing {
    private static TelemetryPacket packet;

    public static void drawRobot(Pose2d pose, String color) {
        if (pose == null) return;
        // SolversLib 的 Heading 可能是 Rotation2d 物件，需轉成 Radians
        drawRobot(pose.getX(), pose.getY(), pose.getRotation().getRadians(), color);
    }
    /**
     * 繪製速度向量
     * @param robotX 機器人 X 座標
     * @param robotY 機器人 Y 座標
     * @param currentVx X 軸速度 (Inches per second)
     * @param currentVy Y 軸速度 (Inches per second)
     * @param hashtag 顏色代碼 (e.g., "#00FF00")
     */
    public static void drawVelocityVector(double robotX, double robotY, double currentVx, double currentVy, String hashtag) {
        // 1. 確保 packet 存在
        if (packet == null) packet = new TelemetryPacket();

        // 2. 設定顏色
        packet.fieldOverlay().setStroke(hashtag);

        // 3. 計算向量終點
        // 建議：可以乘上一個倍率 (例如 5.0)，讓線條更明顯
        // 物理意義：如果不乘倍率，這條線代表「1秒後」機器人會到的位置
        double scaleFactor = 1.0;

        double endX = robotX + (currentVx * scaleFactor);
        double endY = robotY + (currentVy * scaleFactor);

        // 4. 畫線
        packet.fieldOverlay().strokeLine(robotX, robotY, endX, endY);
    }

    /**
     * 一個簡單的內部 Pose 類別，或者你可以替換成 RoadRunner 的 Pose2d
     */
    public static class SimplePose {
        public double x;
        public double y;
        public double heading; // 弧度 (Radians)

        public SimplePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    /**
     * 繪製機器人 (使用座標與角度)
     * @param x 機器人 X 座標
     * @param y 機器人 Y 座標
     * @param heading 機器人朝向 (弧度)
     * @param color 顏色 (Hex String, e.g., "#3F51B5")
     */
    public static void drawRobot(double x, double y, double heading, String color) {
        if (packet == null) packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke(color);
        drawRobotOnCanvas(packet.fieldOverlay(), x, y, heading);
    }

    /**
     * 繪製機器人 (使用物件)
     * @param pose 包含 x, y, heading 的物件
     * @param color 顏色
     */
    public static void drawRobot(SimplePose pose, String color) {
        drawRobot(pose.x, pose.y, pose.heading, color);
    }

    /**
     * 繪製路徑/線條 (使用 X 和 Y 的陣列)
     * @param xPoints X 座標陣列
     * @param yPoints Y 座標陣列
     * @param color 顏色
     */
    public static void drawPath(double[] xPoints, double[] yPoints, String color) {
        if (packet == null) packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokePolyline(xPoints, yPoints);
    }

    /**
     * 將當前的繪圖封包發送到 Dashboard
     * 呼叫此方法後，畫面才會更新
     */
    public static boolean sendPacket() {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = null; // 清空，準備下一幀
            return true;
        }
        return false;
    }

    /**
     * 取得目前的 Packet (如果你想往同一個 Packet 加一般 Telemetry 文字數據)
     */
    public static TelemetryPacket getPacket() {
        if (packet == null) packet = new TelemetryPacket();
        return packet;
    }

    // --- 內部繪圖邏輯 ---

    /**
     * 在 Canvas 上實際畫出機器人 (圓形 + 方向線)
     */
    private static void drawRobotOnCanvas(Canvas c, double x, double y, double heading) {
        // 1. 畫圓 (代表機器人本體)
        c.strokeCircle(x, y, Constant.ROBOT_RADIUS);

        // 2. 畫方向線 (代表車頭)
        // 使用三角函數計算線條終點
        double xEnd = x + (Constant.ROBOT_RADIUS * Math.cos(heading));
        double yEnd = y + (Constant.ROBOT_RADIUS * Math.sin(heading));

        c.strokeLine(x, y, xEnd, yEnd);
    }

    /**
     * 繪製砲塔與瞄準線
     * @param turretPose 砲塔在場地上的座標與旋轉角度
     * @param targetAngle 砲塔當前的目標角度 (Field Centric Radians)
     * @param color 顏色
     */
    public static void drawTurret(Pose2d turretPose, double targetAngle, String color) {
        if (packet == null) packet = new TelemetryPacket();
        Canvas c = packet.fieldOverlay();
        c.setStroke(color);

        double x = turretPose.getX();
        double y = turretPose.getY();
        double turretRadius = 4.0; // 砲塔顯示半徑，可自行調整

        // 1. 畫砲塔本體
        c.strokeCircle(x, y, turretRadius);

        // 2. 畫砲塔當前朝向線 (使用砲塔的 Rotation)
        double headingX = x + (turretRadius * Math.cos(turretPose.getRotation().getRadians()));
        double headingY = y + (turretRadius * Math.sin(turretPose.getRotation().getRadians()));
        c.strokeLine(x, y, headingX, headingY);
    }
}
