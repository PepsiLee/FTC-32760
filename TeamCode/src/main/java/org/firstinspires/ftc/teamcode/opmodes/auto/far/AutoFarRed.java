package org.firstinspires.ftc.teamcode.opmodes.auto.far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveAndAim;
import org.firstinspires.ftc.teamcode.commandbase.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SmartGateCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.StationaryAimShootCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.opmodes.auto.path.AutoPathFar;
import org.firstinspires.ftc.teamcode.util.PedroDrawing;

@Config
@Autonomous(name = "Auto (Far)(Red)", group = "Competition", preselectTeleOp = "TeleOpTuner")
public class AutoFarRed extends CommandOpMode {

    // ==================== 開關設定 ====================
    // 比賽時務必設為 false！
    public static boolean IS_DEBUG_MODE = false;

    // ==================== Robot & Paths ====================
    private Robot robot;
    private AutoPathFar paths;

    @Override
    public void initialize() {
        Constant.OP_MODE_TYPE = Constant.OpModeType.AUTO;
        Constant.ALLIANCE_COLOR = Constant.AllianceColor.RED;

        // 1. 初始化 Robot
        robot = Robot.getInstance();
        robot.init(hardwareMap);
        paths = new AutoPathFar(robot.follower);
        robot.follower.setPose(paths.getStartPose());

        // 2. 預先設定 Telemetry (讓 Dashboard 在 Init 階段就能看到)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 顯示初始訊息
        telemetry.addData("Color", "🔴 RED");
        telemetry.update();

        CommandScheduler.getInstance().schedule(buildAutoSequence());
    }

    /**
     * 建立完整的 Auto Command 序列
     */
    private SequentialCommandGroup buildAutoSequence() {
        return new SequentialCommandGroup(
                // --- Step 1: Preload (預載樣本射擊) ---
                debugWait("Press A to start Preload"),
                logState("🚀 Executing Preload..."),
                shootAndClean(),

                // --- Step 2: Sample 3 (吸取並射擊第3個樣本) ---
                debugWait("Press A for Sample 3"),
                logState("📦 Executing Sample 3..."),

                // 1
                new ParallelDeadlineGroup(
                        new FollowPathCommand(robot.follower, paths.loadingZone),
                        new IntakeCommand(robot.intake, ()-> true)
                ),

                debugWait("Press A to Shoot Sample 3"),
                new ParallelDeadlineGroup(
                        new DriveAndAim(robot, paths.loadingZoneShoot),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                shootAndClean(),

                // 2
                debugWait("Press A for Sample 2"),
                logState("📦 Executing Sample 2..."),

                new ParallelDeadlineGroup(
                        new FollowPathCommand(robot.follower, paths.pickUpper),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                new ParallelDeadlineGroup(
                        new DriveAndAim(robot, paths.pickUpperShoot),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                shootAndClean(),

                //3
                new ParallelDeadlineGroup(
                        new FollowPathCommand(robot.follower, paths.pickUpper),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                new ParallelDeadlineGroup(
                        new DriveAndAim(robot, paths.pickUpperShoot),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                shootAndClean(),

                //4
                new ParallelDeadlineGroup(
                        new FollowPathCommand(robot.follower, paths.pick),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                debugWait("Press A to Shoot Sample 2"),
                new ParallelDeadlineGroup(
                        new DriveAndAim(robot, paths.pickShoot),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                shootAndClean(),

                // 5
                debugWait("Press A for Sample 1"),
                logState("📦 Executing Sample 1..."),

                new ParallelDeadlineGroup(
                        new FollowPathCommand(robot.follower, paths.pick),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                debugWait("Press A to Shoot Sample 2"),
                new ParallelDeadlineGroup(
                        new DriveAndAim(robot, paths.pickShoot),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                shootAndClean(),

                //6
                new ParallelDeadlineGroup(
                        new FollowPathCommand(robot.follower, paths.pickUpperUp),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                new ParallelDeadlineGroup(
                        new DriveAndAim(robot, paths.pickUpperUpShoot),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                shootAndClean(),

                // --- Step 5: Park (停車) ---
                debugWait("Press A to Park"),
                new FollowPathCommand(robot.follower, paths.park),
                new InstantCommand(() -> robot.stopAll()), // 停止所有馬達

                logState("✅ DONE")
        );
    }

    /**
     * 輔助方法：只有在 DEBUG_MODE 為 true 時才會等待按鍵
     */
    private SequentialCommandGroup debugWait(String message) {
        if (IS_DEBUG_MODE) {
            return new SequentialCommandGroup(
                    logState("WAITING: " + message),
                    new WaitUntilCommand(() -> gamepad1.a),
                    new WaitUntilCommand(() -> !gamepad1.a), // 防彈跳
                    new WaitCommand(250) // 小延遲避免誤觸
            );
        } else {
            // 如果不是 Debug 模式，回傳一個空的 Command，不執行任何等待
            return new SequentialCommandGroup();
        }
    }

    private SequentialCommandGroup shootAndClean() {
        return new SequentialCommandGroup(
                new SmartGateCommand(robot.shooter, true),
                // 1. 執行你寫的瞄準射擊指令
                new StationaryAimShootCommand(
                        robot.shooterSystem,
                        robot.intake,
                        Constant.GOAL_POSE()
                ),
                // 4. 最後停止吸入馬達
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.STOP)),
                new SmartGateCommand(robot.shooter,false)
        );
    }

    /**
     * 輔助方法：記錄狀態變化
     */
    private InstantCommand logState(String message) {
        return new InstantCommand(() -> {
            telemetry.addLine("=== " + message + " ===");
            telemetry.update();
        });
    }

    double loopTime;
    @Override
    public void run() {
        robot.updateAuto();
        super.run();

        updateVisualization();
        updateTelemetry();
    }

    private void updateVisualization() {
        Pose currentPose = robot.follower.getPose();
        PedroDrawing.drawRobot(currentPose);
        PedroDrawing.sendPacket();
    }

    private void updateTelemetry() {
        Pose currentPose = robot.follower.getPose();

        telemetry.addLine("=== 📍 Position ===");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Follower Busy", robot.isFollowerBusy());

        telemetry.addLine("\n=== ⚙️ Subsystems ===");
        telemetry.addData("Shooter Ready", robot.shooterSystem.isReady());
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Turret Error",robot.turret.getAngle() - robot.turret.getTargetAngle());
        telemetry.addData("Shooter Current" , robot.shooter.getCurrent());
        telemetry.addData("LoopTime" , loopTime);
        telemetry.addData("Shooter Rpm", robot.shooter.getCurrentRPM());
        telemetry.addData("Rpm target", robot.shooter.getTargetTPS());
    }

    @Override
    public void end() {
        Constant.END_POSE = PoseConverter.poseToPose2D(robot.follower.getPose(), InvertedFTCCoordinates.INSTANCE);
    }
}