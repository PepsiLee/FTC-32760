package org.firstinspires.ftc.teamcode.opmodes.auto.solo;

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
import org.firstinspires.ftc.teamcode.opmodes.auto.path.AutoPathSolo;
import org.firstinspires.ftc.teamcode.util.PedroDrawing;

@Config
@Autonomous(name = "Auto (Solo)(Red)", group = "Competition", preselectTeleOp = "TeleOpTuner")
public class AutoSoloRed extends CommandOpMode {

    // switch setting
    // 比賽時務必設為 false！
    public static boolean IS_DEBUG_MODE = false;
    double loopTime;

    //robot path
    private Robot robot;
    private AutoPathSolo paths;

    @Override
    public void initialize() {
        Constant.OP_MODE_TYPE = Constant.OpModeType.AUTO;
        Constant.ALLIANCE_COLOR = Constant.AllianceColor.RED;

        // 1. 初始化 Robot
        robot = Robot.getInstance();
        robot.init(hardwareMap);
        paths = new AutoPathSolo(robot.follower);
        robot.follower.setPose(paths.getStartPose());
        // 2. 預先設定 Telemetry (讓 Dashboard 在 Init 階段就能看到)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 顯示初始訊息
        telemetry.addData("Color", "🔴 RED");
        telemetry.update();

        // inint path


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
                new DriveAndAim(robot, paths.preloadShoot),
                shootAndClean(),

                // --- Step 2: Spike 3 (吸取並射擊第3個樣本) ---
                debugWait("Press A for Spike 3"),
                logState("📦 Executing Spike 3..."),

                // 平行執行：跑路徑 + 開吸入
                new ParallelDeadlineGroup(
                        new FollowPathCommand(robot.follower, paths.spikeIII), // sparkIII -> spikeIII
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                // 路徑變數名已從 openGate2 改為 openGate3
                new FollowPathCommand(robot.follower, paths.openGate3),

                debugWait("Press A to Shoot Spike 3"),
                new DriveAndAim(robot, paths.shootAfterSpike3), // spark3 -> spike3
                shootAndClean(),

                // --- Step 3: Spike 2 (吸取並射擊第2個樣本) ---
                debugWait("Press A for Spike 2"),
                logState("📦 Executing Spike 2..."),

                new FollowPathCommand(robot.follower, paths.spikeII), // sparkII -> spikeII
                new ParallelDeadlineGroup(
                        // intakeSparkII -> intakeSpikeII
                        new FollowPathCommand(robot.follower, paths.intakeSpikeII).setGlobalMaxPower(0.3),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                new DriveAndAim(robot, paths.shootAfterSpike2), // shootAfterSpark2 -> shootAfterSpike2
                shootAndClean(),

                // --- Step 4: Spike 1 (吸取並射擊第1個樣本) ---
                debugWait("Press A for Spike 1"),
                logState("📦 Executing Spike 1..."),

                new FollowPathCommand(robot.follower, paths.spikeI).setGlobalMaxPower(1), // sparkI -> spikeI
                new ParallelDeadlineGroup(
                        // intakeSparkI -> intakeSpikeI
                        new FollowPathCommand(robot.follower, paths.intakeSpikeI),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                new DriveAndAim(robot, paths.shootAfterSpike1), // shootAfterSpark1 -> shootAfterSpike1
                shootAndClean(),

                // --- Step 5: Spike 4 (Submersible) ---
                debugWait("Press A for Spike 4"),
                logState("📦 Executing Spike 4..."),

                new FollowPathCommand(robot.follower, paths.spikeIV), // sparkIIII -> spikeIV
                new ParallelDeadlineGroup(
                        // intakeSparkIIII -> intakeSpikeIV
                        new FollowPathCommand(robot.follower, paths.intakeSpikeIV),
                        new IntakeCommand(robot.intake, ()-> true)
                ),
                new DriveAndAim(robot, paths.shootAfterSpikeIV), // shootAfterSparkIIII -> shootAfterSpikeIV
                shootAndClean(),

                // --- Step 6: Parking ---
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
            return new SequentialCommandGroup();
        }
    }


    private SequentialCommandGroup shootAndClean() {
        return new SequentialCommandGroup(
                new SmartGateCommand(robot.shooter, true),
                // 執行瞄準射擊指令
                new StationaryAimShootCommand(
                        robot.shooterSystem,
                        robot.intake,
                        Constant.GOAL_POSE()
                ),
                // 最後停止吸入馬達
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
        telemetry.addData("Turret Error", robot.turret.getAngle() - robot.turret.getTargetAngle());
        telemetry.addData("Shooter Current", robot.shooter.getCurrent());
        telemetry.addData("LoopTime", loopTime);
        telemetry.addData("Shooter Rpm", robot.shooter.getCurrentRPM());
        telemetry.addData("Rpm target", robot.shooter.getTargetTPS());
        telemetry.update();
    }

    @Override
    public void end() {
        Constant.END_POSE = PoseConverter.poseToPose2D(robot.follower.getPose(), InvertedFTCCoordinates.INSTANCE);
    }
}