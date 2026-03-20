package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.constant.Constant.ANGLE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.DISTANCE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.END_POSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.commands.FullAimCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.SmartGateCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpInputHandler;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name = "TeleOpTuner")
public class Tuner extends CommandOpMode {

    // ==================== 參數設定 ====================
    private static final double RPM_STEP = 50;      // 每次調整 RPM 的步進值
    private static final double HOOD_STEP = 0.05;   // 每次調整 Hood 的步進值
    private static final double MIN_RPM = 0;
    private static final double MAX_RPM = 6000;
    private static final double MIN_HOOD = 0.0;
    private static final double MAX_HOOD = 1;

    // [新增功能] 砲塔手動控制與預設點參數
    private static final double TURRET_MANUAL_SPEED = 0.2; // 手動旋轉速度 (度/loop)
    // Near
    private static final double PRESET_A_RPM = 2600;
    private static final double PRESET_A_HOOD = 1.0;
    // Far
    private static final double PRESET_B_RPM = 3600;
    private static final double PRESET_B_HOOD = 1.0;
    private static final double DISTANCE_THRESHOLD = 120;
    // ==================== 硬體與輸入 ====================
    private Robot robot;
    private GamepadEx driverOp;
    private GamepadEx toolOp;
    private TeleOpInputHandler inputHandler;
    // ==================== Commands ====================
    private FullAimCommand fullAimCommand;
    private IntakeCommand intakeCommand;
    private RunCommand outTakeCommand;
    private RunCommand manualTuningCommand;
    // [新增功能] 手動砲塔旋轉命令
    private RunCommand manualTurretCommand;
    // ==================== 狀態變數 ====================
    private double manualRPM = 2000;
    private double manualHood = 1.0;
    private boolean wasIntakingBeforeOuttake = false; // 吐球記憶功能
    private boolean wasIntakingBeforeShot = false;
    private int loopCounter = 0;

    @Override
    public void initialize() {
        // Set the op mode type
        Constant.OP_MODE_TYPE = Constant.OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        while (!isStarted() && !isStopRequested()) {
            super.run();

            // 2. 選擇聯盟顏色邏輯
            if (gamepad1.x || gamepad2.x) {
                Constant.ALLIANCE_COLOR = Constant.AllianceColor.BLUE;
            } else if (gamepad1.b || gamepad2.b) {
                Constant.ALLIANCE_COLOR = Constant.AllianceColor.RED;
            }

            // 3. Telemetry 顯示
            telemetry.addData("Status", "⚠️ Waiting for Start (Subsystems Active) ⚠️");
            telemetry.addData("Select Alliance", "X = 🔵 BLUE | B = 🔴 RED");
            String colorStr = (Constant.ALLIANCE_COLOR == Constant.AllianceColor.BLUE) ? "🔵 BLUE" : "🔴 RED";
            telemetry.addData(">> SELECTED", colorStr);

            telemetry.update();
        }

        // Initialize the robot
        robot = Robot.getInstance();
        robot.init(hardwareMap);

        // End Pose
        robot.imu.setPose(new Pose2d(END_POSE.getX(DISTANCE_UNIT), END_POSE.getY(DISTANCE_UNIT), END_POSE.getHeading(ANGLE_UNIT)));
        robot.drive.setTargetHeading(END_POSE.getHeading(ANGLE_UNIT));

        // Open Gate
        robot.shooter.setGateOpen(false);

        //Initialize Gamepad
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        // Initialize Input Handler
        inputHandler = new TeleOpInputHandler();

        initializeCommands();
        setupDriverControls();
        setupToolControls();
    }

    private void initializeCommands() {
        // 1. AutoAim
        fullAimCommand = new FullAimCommand(
                robot.drive, robot.shooterSystem, robot.vision,
                Constant.GOAL_POSE(), driverOp, robot.imu,
                inputHandler
        );

        // 2. Intake/Outtake
        intakeCommand = new IntakeCommand(robot.intake, () -> true);
        outTakeCommand = new RunCommand(
                () -> robot.intake.setState(Intake.IntakeState.OUTTAKE),
                robot.intake
        );

        // 4. Tuning Command
        manualTuningCommand = new RunCommand(() -> {
            robot.shooterSystem.setCalculatedBaseRpm(manualRPM);
            robot.turret.setHoodPosition(manualHood);
        }, robot.shooterSystem);

        // 5. Manual Turret Command
        manualTurretCommand = new RunCommand(() -> {
            double stickInput = toolOp.getRightX();
            if (Math.abs(stickInput) > 0.1) {
                double currentTarget = robot.turret.getTargetAngle();
                double newTarget = currentTarget + (stickInput * TURRET_MANUAL_SPEED);

                robot.turret.setTargetAngle(newTarget);
            }
        }, robot.turret);
    }

    private void setupDriverControls() {
        // ==== Default Command ====

        // Drive Control
        robot.drive.setDefaultCommand(new RunCommand(() -> {
            TeleOpInputHandler.DriveInputs inputs = inputHandler.getDriveInputs(driverOp);

            // ✅ FIX: 原本傳入 robot.imu，現在改為 robot.imu.getHeading()
            robot.drive.TeleOpDrive(inputs.forward, inputs.strafe, inputs.rotation, robot.imu.getHeading());

        }, robot.drive));

        // Intake Default
        robot.intake.setDefaultCommand(intakeCommand);

        // ========== Driver Button ==========
        // RB: AutoAim
        new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(fullAimCommand);

        // B: Launch
        new GamepadButton(driverOp, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    // 1. 記錄是否正在跑 Intake Command
                    wasIntakingBeforeShot = CommandScheduler.getInstance().isScheduled(intakeCommand);

                    // 2. 如果正在跑，必須 "Cancel" 掉指令，避免它在背景跟我們搶控制權
                    if (wasIntakingBeforeShot) {
                        intakeCommand.cancel();
                    }
                }))
                .whileHeld(
                        new ParallelCommandGroup(
                                // 動作 1: 原本的射擊序列
                                new RunCommand(() -> {
                                    robot.shooter.setGateOpen(true);
                                    sleep(100);
                                    Pose2d currentPose = robot.getPose();
                                    double distance = Math.hypot(
                                            Constant.GOAL_POSE().getX() - currentPose.getX(),
                                            Constant.GOAL_POSE().getY() - currentPose.getY()
                                    );
                                    if (distance > DISTANCE_THRESHOLD) {
                                        robot.intake.setState(Intake.IntakeState.TRANSFER);
                                    } else {
                                        robot.intake.setState(Intake.IntakeState.TRANSFER_FAST);
                                    }
                                    sleep(30);//    射球前的加速時間
                                    robot.shooterSystem.triggerBoost();
                                }, robot.intake, robot.shooter),

                                // 動作 2: 砲塔鎖定邏輯
                                new ConditionalCommand(
                                        // True: 按下 B 且按住 RB 時 -> 執行鎖定
                                        // RunCommand requires robot.turret，這會強制中斷 AutoAim
                                        new RunCommand(() -> {
                                            // 維持當前目標角度，不再更新
                                            robot.turret.setTargetAngle(robot.turret.getTargetAngle());
                                        }, robot.turret),

                                        // False: 沒按 RB -> 什麼都不做 (讓 AutoAim 或 ManualTurret 繼續運作)
                                        new InstantCommand(),

                                        // Condition: 檢查 RB 是否被按住
                                        () -> driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                                )
                        )
                )
                .whenReleased(
                        new SequentialCommandGroup(
                                new SetIntake(robot.intake, Intake.IntakeState.TRASFER_OUTTAKE),
                                new WaitCommand(100),
                                new SmartGateCommand(robot.shooter, false),
                                new InstantCommand(() -> {
                                    if (wasIntakingBeforeShot) {
                                        intakeCommand.schedule();
                                    }
                                })
                        )
                );

        // X: Intake button (開關邏輯)
        new GamepadButton(driverOp, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    if (CommandScheduler.getInstance().isScheduled(intakeCommand)) {
                        intakeCommand.cancel();
                    } else {
                        new SmartGateCommand(robot.shooter, false).schedule();
                        intakeCommand.schedule();
                    }
                }));

        // Triangle: Outtake button
        new GamepadButton(driverOp, GamepadKeys.Button.TRIANGLE)
                .whenPressed(new InstantCommand(() -> {
                    new SmartGateCommand(robot.shooter, true).schedule();
                    wasIntakingBeforeOuttake = CommandScheduler.getInstance().isScheduled(intakeCommand);
                    outTakeCommand.schedule();
                }, robot.shooter))
                .whenReleased(new InstantCommand(() -> {
                    new SmartGateCommand(robot.shooter, false).schedule();
                    outTakeCommand.cancel();
                    if (wasIntakingBeforeOuttake) intakeCommand.schedule();
                }, robot.shooter));

        // Start: X-Lock
        new GamepadButton(driverOp, GamepadKeys.Button.START)
                .whileHeld(new RunCommand(() -> robot.drive.setXLock(), robot.drive));

        // Back: Reset Pose
        new GamepadButton(driverOp, GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    robot.setPose(Constant.RESET_POSE());
                    robot.drive.setTargetHeading(Constant.RESET_POSE().getHeading());
                }));

        // [新增功能] Driver D-Pad 預設射擊點
        // D-Pad Up: 預設點 A (近距離)
        new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    // 若正在手動調校模式，先取消，避免數值打架
                    if (CommandScheduler.getInstance().isScheduled(manualTuningCommand)) {
                        manualTuningCommand.cancel();
                    }
                    robot.shooterSystem.setCalculatedBaseRpm(PRESET_A_RPM);
                    robot.turret.setHoodPosition(PRESET_A_HOOD);
                    robot.turret.setTargetAngle(0); // 砲塔歸零回正
                }));

        // D-Pad Down: 預設點 B (遠距離)
        new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (CommandScheduler.getInstance().isScheduled(manualTuningCommand)) {
                        manualTuningCommand.cancel();
                    }
                    robot.shooterSystem.setCalculatedBaseRpm(PRESET_B_RPM);
                    robot.turret.setHoodPosition(PRESET_B_HOOD);
                }));
    }

    private void setupToolControls() {
        // [新增功能] 將手動旋轉設為 Turret 的預設命令
        // 這樣 ToolOp 的右搖桿隨時都能控制砲塔，除非 AutoAim 正在執行
        robot.turret.setDefaultCommand(manualTurretCommand);

        // ========== LB: 切換手動參數調校模式 ==========
        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    if (CommandScheduler.getInstance().isScheduled(manualTuningCommand)) {
                        manualTuningCommand.cancel();
                        robot.shooterSystem.stopAll();
                    } else {
                        // 讀取當前狀態作為起始值
                        manualRPM = robot.shooter.getTargetTPS();
                        manualHood = robot.turret.getHoodPosition();
                        manualTuningCommand.schedule();
                    }
                }));

        // ========== 參數調整 (僅在手動模式下生效) ==========

        // DPad Up/Down: 調整 RPM
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(() -> adjustParam(RPM_STEP, 0));
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> adjustParam(-RPM_STEP, 0));

        // Y/A: 快速調整 RPM
        new GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(() -> adjustParam(RPM_STEP * 5, 0));
        new GamepadButton(toolOp, GamepadKeys.Button.A).whenPressed(() -> adjustParam(-RPM_STEP * 5, 0));

        // DPad Left/Right: 調整 Hood
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> adjustParam(0, HOOD_STEP));
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> adjustParam(0, -HOOD_STEP));

        // B: RPM 歸零
        new GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(() -> adjustParam(-manualRPM, 0));
    }

    // 輔助方法：統一調整參數
    private void adjustParam(double rpmDelta, double hoodDelta) {
        if (CommandScheduler.getInstance().isScheduled(manualTuningCommand)) {
            manualRPM += rpmDelta;
            manualHood += hoodDelta;

            if (manualRPM < MIN_RPM) manualRPM = MIN_RPM;
            if (manualRPM > MAX_RPM) manualRPM = MAX_RPM;
            if (manualHood < MIN_HOOD) manualHood = MIN_HOOD;
            if (manualHood > MAX_HOOD) manualHood = MAX_HOOD;
        }
    }

    @Override
    public void run() {
        // --- 總迴圈開始 ---
        robot.profiler.start("Full Loop");

        // 1. 硬體狀態更新 (Swerve 讀寫通常在裡面)
        robot.profiler.start("Robot Update");
        robot.updateTeleOp();
        robot.vision.update();
        robot.profiler.end("Robot Update");

        // 2. 手把讀取 (通常很快，但建議追蹤)
        robot.profiler.start("Read Gamepads");
        driverOp.readButtons();
        toolOp.readButtons();
        robot.profiler.end("Read Gamepads");

        // 3. Command Scheduler 執行 (包含 FullAim, Swerve 計算等)
        robot.profiler.start("Command Scheduler");
        super.run();
        robot.profiler.end("Command Scheduler");

        boolean isManual = CommandScheduler.getInstance().isScheduled(manualTuningCommand);
        Pose2d robotPose = robot.getPose();
        double distance = Math.hypot(
                Constant.GOAL_POSE().getX() - robotPose.getX(),
                Constant.GOAL_POSE().getY() - robotPose.getY()
        );
        telemetry.addData("X: ", robotPose.getX());
        telemetry.addData("Y: ", robotPose.getY());
        telemetry.addData("Heading: ", robotPose.getHeading());
        telemetry.addData("📍 Distance to Goal", "%.2f", distance);
        double actRPM = robot.shooter.getCurrentRPM();
        telemetry.addData("rpm target", robot.shooter.getTargetTPS());
        telemetry.addData("power", robot.shooter.getFinalPower());
        telemetry.addData("rpm", actRPM);
        telemetry.addData("OverDrive", robot.shooter.isOverdrive());
        telemetry.addData("Intake Dis", robot.intake.getDistanceMM());
        telemetry.addData("Intake Current ", robot.intake.getIntakeCurrent());
        telemetry.addData("Transfer Current ", robot.intake.getTransferCurrent());
        telemetry.addData("Shooter RPM (Act/Tgt)", "%.0f / %.0f", actRPM, isManual ? manualRPM : robot.shooter.getTargetTPS());
//        Drawing.getPacket().fieldOverlay().drawImage("/dash/decode.webp", 0, 0, 144, 144);
//        Drawing.drawRobot(robot.getPose(), isManual ? "Orange" : "#3F51B5");
//        Drawing.sendPacket();
        telemetry.update();

        // --- 總迴圈結束 ---
        robot.profiler.end("Full Loop");
    }

    private void updateTelemetry() {
        boolean isManual = CommandScheduler.getInstance().isScheduled(manualTuningCommand);
        boolean isAuto = CommandScheduler.getInstance().isScheduled(fullAimCommand);

        telemetry.addLine(isManual ? "🛠️ MODE: MANUAL TUNING" : (isAuto ? "🎯 MODE: AUTO AIM" : "⚪ MODE: IDLE"));
        telemetry.addLine("--------------------------------");

        // Get Current Pose
        Pose2d robotPose = robot.getPose();
        double distance = Math.hypot(
                Constant.GOAL_POSE().getX() - robotPose.getX(),
                Constant.GOAL_POSE().getY() - robotPose.getY()
        );
        telemetry.addData("X: ", robotPose.getX());
        telemetry.addData("Y: ", robotPose.getY());
        telemetry.addData("Heading: ", robotPose.getHeading());
        telemetry.addData("📍 Distance to Goal", "%.2f", distance);

        //SubSystems
        double actRPM = robot.shooter.getCurrentRPM();
        double actHood = robot.turret.getHoodPosition();
        telemetry.addData("Shooter RPM (Act/Tgt)", "%.0f / %.0f", actRPM, isManual ? manualRPM : robot.shooter.getTargetTPS());
        telemetry.addData("Shooter RPM ACT", actRPM);
        telemetry.addData("Shooter Current", robot.shooter.getCurrent());
        telemetry.addData("Hood Pos (Act/Tgt)", "%.3f / %.3f", actHood, isManual ? manualHood : robot.turret.getHoodPosition());

        telemetry.addData("Intake", robot.intake.getState());

        telemetry.addData("Intake Current ", robot.intake.getIntakeCurrent());
        telemetry.addData("Transfer Current ", robot.intake.getTransferCurrent());
        telemetry.addData("Intake Dis", robot.intake.getDistanceMM());

        telemetry.addData("Turret (Act/Tgt)", "%.3f / %.3f", robot.turret.getAngle(), robot.turret.getTargetAngle());
        telemetry.addData("Turret Error", robot.turret.getAngle() - robot.turret.getTargetAngle());

        telemetry.addData("Ready to Fire", robot.shooterSystem.isReady());


        if (isManual) {
            telemetry.addLine("\n🎮 Tuning Controls:");
            telemetry.addLine("DPad Up/Down: RPM +/- 50");
            telemetry.addLine("DPad L/R: Hood +/- 0.05");
            telemetry.addLine("LB: Exit Manual Mode");
        } else {
            telemetry.addLine("\n🎮 Driver Controls:");
            telemetry.addLine("RB: Auto Aim");
            telemetry.addLine("D-Pad Up/Down: Presets A/B");
            telemetry.addLine("ToolOp R-Stick: Manual Turret");
        }

        telemetry.update();

        TelemetryPacket packet = Drawing.getPacket();

        Drawing.drawRobot(robot.getPose(), isManual ? "Orange" : "#3F51B5");
        packet.fieldOverlay().drawImage("/dash/decode.webp", 0, 0, 144, 144);
        Drawing.sendPacket();
    }

    @Override
    public void end() {
        robot.exportProfiler(robot.file);
    }
}