package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.constant.Constant;

@Config
public class StationaryAimShootCommand extends CommandBase {

    // === Dashboard 設定 ===
    public static double TIMEOUT_SEC = 4.0; // 預設超時秒數
    public static double SHOOT_DELAY = 0.6; // 射擊後等待球飛出的時間

    private final ShooterSystem shooterSystem;
    private final Intake intake;
    private final Translation2d goalPosition;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private boolean hasShot = false;

    // 用於計算砲塔位置的固定偏移量 (相對於機器人中心)
    private final Translation2d turretOffsetRobotFrame = new Translation2d(-1, 0.0);

    public StationaryAimShootCommand(ShooterSystem shooterSystem, Intake intake, Translation2d goalPosition) {
        this.shooterSystem = shooterSystem;
        this.intake = intake;
        this.goalPosition = goalPosition;

        addRequirements(shooterSystem, intake);
    }

    @Override
    public void initialize() {
        hasShot = false;
        timer.reset();
        intake.setState(Intake.IntakeState.STOP);
    }

    @Override
    public void execute() {
        // 1. 使用手動( TeleOp )相同的 Pose 來源：IMU + Vision 融合
        Pose2d robotPose = Robot.getInstance().getPose();
        // 將砲塔偏移量旋轉到場地座標系
        Translation2d turretOffsetFieldFrame = turretOffsetRobotFrame.rotateBy(robotPose.getRotation());

        Pose2d turretPose = new Pose2d(
                robotPose.getTranslation().plus(turretOffsetFieldFrame),
                robotPose.getRotation()
        );

        // 即使是靜止，傳入速度也能防止 IMU 漂移導致的微小誤算 (若有微小滑動)
        Vector2d velocity = new Vector2d();

        // 2. 執行瞄準 (只轉砲塔，不動底盤)
        shooterSystem.aimAtRunning(turretPose, velocity, goalPosition);

        // 3. 判斷射擊條件
        if (shooterSystem.isReady() && !hasShot) {
            shooterSystem.shootIfReady();
            double distance = Math.hypot(
                    Constant.GOAL_POSE().getX() - robotPose.getX(),
                    Constant.GOAL_POSE().getY() - robotPose.getY()
            );
            if (distance > 120) {
                intake.setState(Intake.IntakeState.TRANSFER);
            } else {
                intake.setState(Intake.IntakeState.TRANSFER_FAST);
            }

            hasShot = true;
            shootTimer.reset(); // 開始計時射擊延遲
        }
    }

    @Override
    public boolean isFinished() {
        // 條件 A: 已經射擊，並且過了延遲時間 (確保球離開)
        boolean shotComplete = hasShot && shootTimer.seconds() > SHOOT_DELAY;

        // 條件 B: 超時強制結束 (避免卡死)
        boolean timedOut = timer.seconds() > TIMEOUT_SEC;

        return shotComplete || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSystem.stopShoot();
        intake.setState(Intake.IntakeState.STOP);
    }
}
