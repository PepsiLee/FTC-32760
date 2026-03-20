package org.firstinspires.ftc.teamcode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.constant.Constant;
 //自動射遠的自己開門
public class AutoPathNearOpenGate {

    private final Follower follower;

    // ==================== 1. 座標點位定義 (數值完全保留) ====================

    // --- Start & Preload ---
    private final Pose startPos          = p(39.800, 136.000, 270);
    private final Pose preloadScorePos   = p(55.729, 83.915, 180);

    // --- Sample 3 (Spark III) ---
    private final Pose spikeIIIIntakePos = p(26.417, 83.915, 180);
    private final Pose gate3InterPos     = p(26.417, 80, 180);    // Sample 3 特有的開門中繼點
    private final Pose gate3EndPos       = p(18, 80, 180);        // Sample 3 特有的開門結束點
    private final Pose scorePos3         = p(54.448, 88.230, 180); // 注意：這個座標跟下面 commonScore 略有不同

    // --- Sample 2 (Spark II) ---
    private final Pose sparkIIReadyPos = p(46.945, 61.000, 180);
    private final Pose sparkIIIntakePos = p(27.000, 61.000, 180);
    private final Pose gate2EndPos       = p(18.000, 65.000, 180); // Sample 2 & 1 的開門/射擊點

    // --- Sample 1 (Spark I) ---
    private final Pose commonScorePos    = p(54.363, 88.262, 180);
    private final Pose spikeIReadyPos = p(47.957, 38.522, 180);
    private final Pose spikeIIntakePos = p(25.265, 38.522, 180);
    private final Pose gate1InterPos     = p(27.000, 60, 180);    // Sample 1 特有的開門轉折點

    // --- Park ---
    private final Pose parkPos           = p(24.000, 72.000, 90);

    // ==================== 2. 路徑物件定義 ====================
    public PathChain preloadShoot;
    public PathChain spikeIII, openGate, shootAfterSparkIII;
    public PathChain spikeII, intakeSpikeII, openGate2, shootAfterSpikeII;
    public PathChain spikeI, intakeSpikeI, openGate1, shootAfterSpikeI;
    public PathChain park;

    /**
     * 建構子
     */
    public AutoPathNearOpenGate(Follower follower) {
        this.follower = follower;
        buildPaths();
    }

    /**
     * 路徑建構邏輯
     * 現在這裡只引用變數，邏輯非常清晰
     */
    private void buildPaths() {
        // --- Preload ---
        preloadShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPos, preloadScorePos))
                .setLinearHeadingInterpolation(startPos.getHeading(), preloadScorePos.getHeading())
                .build();

        // --- Sample 3 Sequence ---
        spikeIII = follower.pathBuilder()
                .addPath(new BezierLine(preloadScorePos, spikeIIIIntakePos))
                .setConstantHeadingInterpolation(spikeIIIIntakePos.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(spikeIIIIntakePos, gate3InterPos))
                .setConstantHeadingInterpolation(gate3InterPos.getHeading())
                .addPath(new BezierLine(gate3InterPos, gate3EndPos))
                .setConstantHeadingInterpolation(gate3EndPos.getHeading())
                .build();

        shootAfterSparkIII = follower.pathBuilder()
                .addPath(new BezierLine(gate3EndPos, scorePos3))
                .setConstantHeadingInterpolation(scorePos3.getHeading())
                .build();

        // --- Sample 2 Sequence ---
        spikeII = follower.pathBuilder()
                .addPath(new BezierLine(scorePos3, sparkIIReadyPos))
                .setConstantHeadingInterpolation(sparkIIReadyPos.getHeading())
                .build();

        intakeSpikeII = follower.pathBuilder()
                .addPath(new BezierLine(sparkIIReadyPos, sparkIIIntakePos))
                .setConstantHeadingInterpolation(sparkIIIntakePos.getHeading())
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierLine(sparkIIIntakePos, gate2EndPos))
                .setConstantHeadingInterpolation(gate2EndPos.getHeading())
                .build();

        shootAfterSpikeII = follower.pathBuilder()
                .addPath(new BezierLine(gate2EndPos, commonScorePos))
                .setConstantHeadingInterpolation(commonScorePos.getHeading())
                .build();

        // --- Sample 1 Sequence ---
        spikeI = follower.pathBuilder()
                .addPath(new BezierLine(commonScorePos, spikeIReadyPos))
                .setConstantHeadingInterpolation(spikeIReadyPos.getHeading())
                .build();

        intakeSpikeI = follower.pathBuilder()
                .addPath(new BezierLine(spikeIReadyPos, spikeIIntakePos))
                .setConstantHeadingInterpolation(spikeIIntakePos.getHeading())
                .build();

        openGate1 = follower.pathBuilder()
                .addPath(new BezierLine(spikeIIntakePos, gate1InterPos))
                .setConstantHeadingInterpolation(gate1InterPos.getHeading())
                .addPath(new BezierLine(gate1InterPos, gate2EndPos))
                .setConstantHeadingInterpolation(gate2EndPos.getHeading())
                .build();

        shootAfterSpikeI = follower.pathBuilder()
                .addPath(new BezierLine(gate2EndPos, commonScorePos))
                .setConstantHeadingInterpolation(commonScorePos.getHeading())
                .build();

        // --- Park ---
        park = follower.pathBuilder()
                .addPath(new BezierLine(commonScorePos, parkPos))
                .setLinearHeadingInterpolation(commonScorePos.getHeading(), parkPos.getHeading())
                .build();
    }

    // ==================== 工具函數 ====================

    // 將此方法設為 static 以便在變數初始化時使用
    private static Pose p(double x, double y, double degrees) {
        if (Constant.ALLIANCE_COLOR == Constant.AllianceColor.RED) {
            return new Pose(144 - x, y, Math.toRadians(180 - degrees));
        } else {
            return new Pose(x, y, Math.toRadians(degrees));
        }
    }

    // 備用工具，雖然上面的 getHeading() 已經處理了大部分需求
    private double angleMirror(double degrees) {
        if (Constant.ALLIANCE_COLOR == Constant.AllianceColor.RED) {
            return Math.toRadians(180 - degrees);
        } else {
            return Math.toRadians(degrees);
        }
    }

    public Pose getStartPose() {
        return startPos;
    }
}