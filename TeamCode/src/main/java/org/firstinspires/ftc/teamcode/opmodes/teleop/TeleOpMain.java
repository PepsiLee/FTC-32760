package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.constant.Constant.ANGLE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.DISTANCE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.END_POSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.commands.FullAimCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SmartGateCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name = "TeleOp", group = "Competition")
public class TeleOpMain extends CommandOpMode {

    // ==================== 比賽參數 ====================
    private static final double PRESET_NEAR_RPM = 2600;
    private static final double PRESET_NEAR_HOOD = 1.0;
    private static final double PRESET_FAR_RPM = 3500;
    private static final double PRESET_FAR_HOOD = 1;
    private static final double TURRET_MANUAL_SPEED = 0.3;
    private static final double TURRET_SOFT_LIMIT = 70.0;
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
    private RunCommand manualTurretCommand;

    // ==================== 狀態變數 ====================
    private boolean wasIntakingBeforeOuttake = false;
    private double currentDistanceToGoal = 0.0;

    @Override
    public void initialize() {
        Constant.OP_MODE_TYPE = Constant.OpModeType.TELEOP;
        super.reset();

        robot = Robot.getInstance();
        robot.init(hardwareMap);

        // 初始化位置：繼承 Auto 的結束位置
        // 這裡同時設定 IMU Offset，確保一開始場地座標是準的
        Pose2d startPose = new Pose2d(END_POSE.getX(DISTANCE_UNIT), END_POSE.getY(DISTANCE_UNIT), END_POSE.getHeading(ANGLE_UNIT));
        robot.imu.setPose(startPose);
        robot.drive.setTargetHeading(startPose.getHeading());

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);
        inputHandler = new TeleOpInputHandler();

        initializeCommands();
        configureBindings();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void initializeCommands() {
        // 1. Auto Aim
        fullAimCommand = new FullAimCommand(
                robot.drive, robot.shooterSystem, robot.vision,
                Constant.GOAL_POSE(), driverOp, robot.imu, inputHandler
        );

        // 2. Intake / Outtake
        intakeCommand = new IntakeCommand(robot.intake, ()-> true);
        outTakeCommand = new RunCommand(() -> robot.intake.setState(Intake.IntakeState.OUTTAKE), robot.intake);

        // 3. Manual Turret
        manualTurretCommand = new RunCommand(() -> {
            double stickInput = toolOp.getRightX();
            if (Math.abs(stickInput) > 0.1) {
                double currentTarget = robot.turret.getTargetAngle();
                double newTarget = currentTarget + (stickInput * TURRET_MANUAL_SPEED);
                newTarget = Math.max(-TURRET_SOFT_LIMIT, Math.min(TURRET_SOFT_LIMIT, newTarget));
                robot.turret.setTargetAngle(newTarget);
            }
        }, robot.shooterSystem);

        // 4. Default Drive Command
        robot.drive.setDefaultCommand(new RunCommand(() -> {
            TeleOpInputHandler.DriveInputs inputs = inputHandler.getDriveInputs(driverOp);

            // 修正：從 IMU 讀取一次角度數值，傳入 double，而非傳入 IMU 物件
            // 這樣 Drive Subsystem 不需要依賴 IMU 硬體
            double fieldHeading = robot.imu.getHeading();

            robot.drive.TeleOpDrive(inputs.forward, inputs.strafe, inputs.rotation, fieldHeading);
        }, robot.drive));

        robot.intake.setDefaultCommand(new RunCommand(() -> robot.intake.setState(Intake.IntakeState.STOP), robot.intake));
        robot.turret.setDefaultCommand(manualTurretCommand);
    }

    private void configureBindings() {
        // ==================== DRIVER (Gamepad 1) ====================

        new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(fullAimCommand);

        new GamepadButton(driverOp, GamepadKeys.Button.B)
                .whileHeld(new SequentialCommandGroup(
                        new SmartGateCommand(robot.shooter, true),
                        new RunCommand(() -> {
                            if (currentDistanceToGoal > DISTANCE_THRESHOLD) {
                                robot.intake.setState(Intake.IntakeState.TRANSFER);
                            } else {
                                robot.intake.setState(Intake.IntakeState.INTAKE);
                            }
                        }, robot.intake)
                ))
                .whenReleased(new SequentialCommandGroup(
                        new SmartGateCommand(robot.shooter, false),
                        new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.STOP))
                ));

        // 修正：Back Button Reset 邏輯
        new GamepadButton(driverOp, GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    Pose2d resetPose = Constant.RESET_POSE();

                    // 1. 重設里程計座標
                    robot.setPose(resetPose);
                    // 2. 重設 Drive 鎖定角度
                    robot.drive.setTargetHeading(resetPose.getHeading());
                    // 3. 重設 IMU Offset (關鍵！防止 Reset 後場地座標失效)
                    robot.imu.setPose(resetPose);
                }));

        new GamepadButton(driverOp, GamepadKeys.Button.START)
                .whileHeld(new RunCommand(() -> robot.drive.setXLock(), robot.drive));

        // ==================== OPERATOR (Gamepad 2) ====================

        new GamepadButton(driverOp, GamepadKeys.Button.X)
                .toggleWhenPressed(
                        new InstantCommand(() -> {
                            new SmartGateCommand(robot.shooter, false).schedule();
                            intakeCommand.schedule();
                        }),
                        new InstantCommand(() -> intakeCommand.cancel())
                );

        new GamepadButton(driverOp, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    wasIntakingBeforeOuttake = CommandScheduler.getInstance().isScheduled(intakeCommand);
                    intakeCommand.cancel();
                    outTakeCommand.schedule();
                }))
                .whenReleased(new InstantCommand(() -> {
                    outTakeCommand.cancel();
                    if (wasIntakingBeforeOuttake) intakeCommand.schedule();
                }));

        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    robot.shooterSystem.setCalculatedBaseRpm(PRESET_FAR_RPM);
                    robot.turret.setHoodPosition(PRESET_FAR_HOOD);
                }));

        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    robot.shooterSystem.setCalculatedBaseRpm(PRESET_NEAR_RPM);
                    robot.turret.setHoodPosition(PRESET_NEAR_HOOD);
                }));
    }

    @Override
    public void run() {
        robot.profiler.start("Loop");

        // 先更新感測與按鍵，再執行 CommandScheduler，避免控制使用到上一圈資料
        robot.updateTeleOp();
        driverOp.readButtons();
        toolOp.readButtons();

        Translation2d goalPose = Constant.GOAL_POSE();
        Pose2d currentPose = robot.getPose();
        currentDistanceToGoal = Math.hypot(
                goalPose.getX() - currentPose.getX(),
                goalPose.getY() - currentPose.getY()
        );

        super.run();

        updateMatchTelemetry();

        robot.profiler.end("Loop");
    }

    private void updateMatchTelemetry() {
        boolean isAutoAim = CommandScheduler.getInstance().isScheduled(fullAimCommand);
        boolean isShooterReady = robot.shooterSystem.isReady();

        telemetry.addData("⏰ Match Time", "%.1f s", getRuntime());
        telemetry.addData("🎯 Aim Mode", isAutoAim ? "AUTO LOCKED" : "MANUAL / IDLE");
        telemetry.addData("📏 Distance", "%.1f", currentDistanceToGoal);

        if (isShooterReady && isAutoAim) {
            telemetry.addLine("\n✅✅ READY TO FIRE (PRESS B) ✅✅\n");
        } else if (isAutoAim) {
            telemetry.addLine("\n⏳ Aiming / Spooling up...\n");
        }

        telemetry.addData("Shooter RPM", "%.0f", robot.shooter.getCurrentRPM());
        telemetry.addData("Turret Angle", "%.1f", robot.turret.getAngle());
        telemetry.addData("Intake State", robot.intake.getState());

        telemetry.update();

        boolean isManual = !isAutoAim;
        Drawing.getPacket().fieldOverlay().drawImage("/dash/decode.webp", 0, 0, 144, 144);
        Drawing.drawRobot(robot.getPose(), isManual ? "Orange" : "#3F51B5");
        Drawing.sendPacket();
    }
}
