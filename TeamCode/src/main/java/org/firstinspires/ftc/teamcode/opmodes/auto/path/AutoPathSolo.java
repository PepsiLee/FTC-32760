package org.firstinspires.ftc.teamcode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.constant.Constant;

/**
 * 自動階段路徑定義（Solo 模式 - 4 顆樣本）
 */
public class AutoPathSolo {

    private final Follower follower;

    // ==================== 1. 座標點位定義 (Pose Definitions) ====================

    // --- Start & Preload ---
    private final Pose startPos            = p(39.8, 136, 270);
    private final Pose preloadScorePos     = p(55.72923220536756, 83.915, 180);

    // --- Sample 3 (Spike 3) ---
    private final Pose spike3IntakePos     = p(26, 83.915, 180);
    private final Pose spike3GateInterPos  = p(26, 80, 180);     // 開門中繼點
    private final Pose spike3GateEndPos    = p(18, 80, 180);     // 開門結束點 / 射擊起始點
    private final Pose scorePos3           = p(54.448, 88.230, 180);

    // --- Sample 2 (Spike 2) ---
    private final Pose spike2ReadyPos      = p(46.945, 61, 180);
    private final Pose spike2IntakePos     = p(26.417, 61, 180);
    // 注意：原程式碼中 Shoot2 直接從 (18, 69) 開始，中間沒有路徑連接 Intake2
    private final Pose spike2ShootStartPos = p(18, 69, 180);
    private final Pose commonScorePos      = p(54.363, 88.262, 180); // 用於 Spike 2 & 4

    // --- Sample 1 (Spike 1) ---
    private final Pose spike1ReadyPos      = p(47.957, 38.522, 180);
    private final Pose spike1IntakePos     = p(17, 38.522, 180);
    private final Pose scorePos1           = p(53, 88.262, 180);     // 注意：Spike 1 的射擊點 X 是 53，與其他的略有不同

    // --- Sample 4 (Spike 4 / Submersible) ---
    private final Pose spike4ReadyPos      = p(9.24, 37.3, 270);
    private final Pose spike4IntakePos     = p(8.76, 12, 270);

    // --- Parking ---
    private final Pose parkPos             = p(24, 72, 90);


    // ==================== 2. 路徑物件定義 ====================
    public PathChain preloadShoot;

    // Spike 3
    public PathChain spikeIII, openGate3, shootAfterSpike3;

    // Spike 2
    public PathChain spikeII, intakeSpikeII, shootAfterSpike2;

    // Spike 1
    public PathChain spikeI, intakeSpikeI, shootAfterSpike1;

    // Spike 4 (原 SparkIIII)
    public PathChain spikeIV, intakeSpikeIV, shootAfterSpikeIV;

    public PathChain park;

    /**
     * 建構子
     */
    public AutoPathSolo(Follower follower) {
        this.follower = follower;
        buildPaths();
    }

    /**
     * 核心映射工具函數
     */
    private Pose p(double x, double y, double degrees) {
        if (Constant.ALLIANCE_COLOR == Constant.AllianceColor.RED) {
            return new Pose(144 - x, y, Math.toRadians(180 - degrees));
        } else {
            return new Pose(x, y, Math.toRadians(degrees));
        }
    }

    // 角度鏡像工具
    private double angleMirror(double degrees) {
        if (Constant.ALLIANCE_COLOR == Constant.AllianceColor.RED) {
            return Math.toRadians(180 - degrees);
        } else {
            return Math.toRadians(degrees);
        }
    }

    private void buildPaths() {
        // --- Preload ---
        preloadShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPos, preloadScorePos))
                .setLinearHeadingInterpolation(startPos.getHeading(), preloadScorePos.getHeading())
                .build();

        // --- Sample 3 (Spike 3) ---
        spikeIII = follower.pathBuilder()
                .addPath(new BezierLine(preloadScorePos, spike3IntakePos))
                .setConstantHeadingInterpolation(spike3IntakePos.getHeading())
                .build();

        // 原本叫 openGate2，但座標在 Y=80 (Sample 3)，所以更名為 openGate3
        openGate3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3IntakePos, spike3GateInterPos))
                .setConstantHeadingInterpolation(spike3GateInterPos.getHeading())
                .addPath(new BezierLine(spike3GateInterPos, spike3GateEndPos))
                .setConstantHeadingInterpolation(spike3GateEndPos.getHeading())
                .build();

        shootAfterSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3GateEndPos, scorePos3))
                .setConstantHeadingInterpolation(scorePos3.getHeading())
                .build();

        // --- Sample 2 (Spike 2) ---
        spikeII = follower.pathBuilder()
                .addPath(new BezierLine(scorePos3, spike2ReadyPos))
                .setConstantHeadingInterpolation(spike2ReadyPos.getHeading())
                .build();

        intakeSpikeII = follower.pathBuilder()
                .addPath(new BezierLine(spike2ReadyPos, spike2IntakePos))
                .setConstantHeadingInterpolation(spike2IntakePos.getHeading())
                .build();

        // 這裡保留原邏輯：直接從 (18, 69) 開始，沒有連接前面的 Intake 點
        shootAfterSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2ShootStartPos, commonScorePos))
                .setConstantHeadingInterpolation(commonScorePos.getHeading())
                .build();

        // --- Sample 1 (Spike 1) ---
        spikeI = follower.pathBuilder()
                .addPath(new BezierLine(commonScorePos, spike1ReadyPos))
                .setConstantHeadingInterpolation(spike1ReadyPos.getHeading())
                .build();

        intakeSpikeI = follower.pathBuilder()
                .addPath(new BezierLine(spike1ReadyPos, spike1IntakePos))
                .setConstantHeadingInterpolation(spike1IntakePos.getHeading())
                .build();

        shootAfterSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1IntakePos, scorePos1)) // 終點為 scorePos1 (X=53)
                .setConstantHeadingInterpolation(scorePos1.getHeading())
                .build();

        // --- Sample 4 (Spike 4 / Submersible) ---
        // 原 sparkIIII
        spikeIV = follower.pathBuilder()
                .addPath(new BezierLine(scorePos1, spike4ReadyPos))
                .setLinearHeadingInterpolation(scorePos1.getHeading(), spike4ReadyPos.getHeading())
                .build();

        // 原 intakeSparkIIII
        intakeSpikeIV = follower.pathBuilder()
                .addPath(new BezierLine(spike4ReadyPos, spike4IntakePos))
                .setConstantHeadingInterpolation(spike4IntakePos.getHeading())
                .build();

        // 原 shootAfterSparkIIII
        shootAfterSpikeIV = follower.pathBuilder()
                .addPath(new BezierLine(spike4IntakePos, commonScorePos))
                .setLinearHeadingInterpolation(spike4IntakePos.getHeading(), commonScorePos.getHeading())
                .build();

        // --- Parking ---
        park = follower.pathBuilder()
                .addPath(new BezierLine(commonScorePos, parkPos))
                .setLinearHeadingInterpolation(commonScorePos.getHeading(), parkPos.getHeading())
                .build();
    }

    /**
     * 獲取起始位置
     */
    public Pose getStartPose() {
        return startPos;
    }
}