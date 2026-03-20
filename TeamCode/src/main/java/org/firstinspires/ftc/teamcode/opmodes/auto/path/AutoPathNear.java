package org.firstinspires.ftc.teamcode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.constant.Constant;

public class AutoPathNear {

    private final Follower follower;

    // pose fuction
    private final Pose startPos          = p(39.8, 136, 270);
    private final Pose preloadScorePos   = p(55.72923220536756, 83.915, 180);
    private final Pose spike3IntakePos   = p(26.417, 83.915, 180);
    private final Pose commonScorePos    = p(54.363, 88.262, 180);
    private final Pose spike2StartPos    = p(54.448, 88.230, 180);
    private final Pose spike2TransitPos  = p(46.945, 61, 180);
    private final Pose spike2IntakePos   = p(27, 61, 180);

    // Gate (推門動作)
    private final Pose gateStartPos      = p(27, 65, 180);
    private final Pose gateEndPos        = p(18, 65, 180);
    private final Pose spike1TransitPos  = p(47.957, 38.522, 180);
    private final Pose spike1IntakePos   = p(25.265, 38.522, 180);
    // Parking
    private final Pose parkPos           = p(24, 72, 90);

 //path function define
    public PathChain preloadShoot;

    public PathChain spikeIII, shootAfterSpike3;
    public PathChain spikeII, intakeSpikeII, openGate2, shootAfterSpike2;
    public PathChain spikeI, intakeSpikeI, shootAfterSpike1;

    public PathChain park;


    public AutoPathNear(Follower follower) {
        this.follower = follower;
        buildPaths();
    }

    //change alliance
    private Pose p(double x, double y, double degrees) {
        if (Constant.ALLIANCE_COLOR == Constant.AllianceColor.RED) {
            return new Pose(144 - x, y, Math.toRadians(180 - degrees));
        } else {
            return new Pose(x, y, Math.toRadians(degrees));
        }
    }

    // 雖然變數化了，但這個函數保留著以防萬一需要手動轉角度
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
                .addPath(new BezierLine(preloadScorePos, spike3IntakePos)) // 起點修正為 preloadScorePos，確保連續性
                .setConstantHeadingInterpolation(spike3IntakePos.getHeading())
                .build();

        shootAfterSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3IntakePos, commonScorePos))
                .setConstantHeadingInterpolation(commonScorePos.getHeading())
                .build();

        // --- Sample 2 (Spike 2) ---
        spikeII = follower.pathBuilder()
                .addPath(new BezierLine(spike2StartPos, spike2TransitPos))
                .setConstantHeadingInterpolation(spike2TransitPos.getHeading())
                .build();

        intakeSpikeII = follower.pathBuilder()
                .addPath(new BezierLine(spike2TransitPos, spike2IntakePos))
                .setConstantHeadingInterpolation(spike2IntakePos.getHeading())
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierLine(gateStartPos, gateEndPos))
                .setConstantHeadingInterpolation(gateEndPos.getHeading())
                .build();

        shootAfterSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(gateEndPos, commonScorePos))
                .setConstantHeadingInterpolation(commonScorePos.getHeading())
                .build();

        // --- Sample 1 (Spike 1) ---
        spikeI = follower.pathBuilder()
                // 這裡原本是從 commonScorePos 出發
                .addPath(new BezierLine(commonScorePos, spike1TransitPos))
                .setConstantHeadingInterpolation(spike1TransitPos.getHeading())
                .build();

        intakeSpikeI = follower.pathBuilder()
                .addPath(new BezierLine(spike1TransitPos, spike1IntakePos))
                .setConstantHeadingInterpolation(spike1IntakePos.getHeading())
                .build();

        shootAfterSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1IntakePos, commonScorePos))
                .setConstantHeadingInterpolation(commonScorePos.getHeading())
                .build();

        // --- Parking ---
        park = follower.pathBuilder()
                .addPath(new BezierLine(commonScorePos, parkPos))
                .setLinearHeadingInterpolation(commonScorePos.getHeading(), parkPos.getHeading())
                .build();
    }


    public Pose getStartPose() {
        return startPos;
    }
}