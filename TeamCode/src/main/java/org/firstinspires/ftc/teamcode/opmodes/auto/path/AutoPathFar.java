package org.firstinspires.ftc.teamcode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.constant.Constant;

/**
 * 自動階段路徑定義（Far Side / 遠端）- 點位變數化版本
 */
public class AutoPathFar {

    private final Follower follower;

    // 起始點
    private final Pose startPos          = p(55.9, 8.000, 180);

    private final Pose intakePos         = p(9, 9, 180);
    private final Pose shootPos         = p(52.743, 10.880, 180);
    private final Pose intakeUpperPos    = p(14, 18.5, 180);
    private final Pose intakeUpperUpPos  = p(14, 19.5, 180);
    private final Pose parkPos           = p(56, 36, 180);
    // ==================== 2. 路徑物件定義 ====================
    public PathChain loadingZone, loadingZoneShoot;
    public PathChain pick, pickShoot;
    public PathChain pickUpper, pickUpperShoot;
    public PathChain pickUpperUp, pickUpperUpShoot;
    public PathChain park;

    /**
     * 建構子
     */
    public AutoPathFar(Follower follower) {
        this.follower = follower;
        buildPaths();
    }

    //核心映射工具函數

    private Pose p(double x, double y, double degrees) {
        if (Constant.ALLIANCE_COLOR == Constant.AllianceColor.RED) {
            return new Pose(144 - x, y, Math.toRadians(180 - degrees));
        } else {
            return new Pose(x, y, Math.toRadians(degrees));
        }
    }

    private void buildPaths() {
        loadingZone = follower.pathBuilder()
                .addPath(new BezierLine(startPos, intakePos))
                .setConstantHeadingInterpolation(intakePos.getHeading())
                .build();

        loadingZoneShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakePos, shootPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();

        pick = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, intakePos))
                .setConstantHeadingInterpolation(intakePos.getHeading())
                .build();

        pickShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakePos, shootPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();

        // 原本程式碼中定義了兩次，這裡是定義路徑物件，OpMode如果要跑兩次，呼叫兩次 follower.followPath(paths.pickUpper) 即可
        pickUpper = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, intakeUpperPos))
                .setConstantHeadingInterpolation(intakeUpperPos.getHeading())
                .build();

        pickUpperShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeUpperPos, shootPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();

        pickUpperUp = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, intakeUpperUpPos))
                .setConstantHeadingInterpolation(intakeUpperUpPos.getHeading())
                .build();

        pickUpperUpShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakeUpperUpPos, shootPos))
                .setConstantHeadingInterpolation(shootPos.getHeading())
                .build();

        // --- Park ---
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, parkPos))
                .setConstantHeadingInterpolation(parkPos.getHeading())
                .build();
    }

    //get start point
    public Pose getStartPose() {
        return startPos;
    }
}