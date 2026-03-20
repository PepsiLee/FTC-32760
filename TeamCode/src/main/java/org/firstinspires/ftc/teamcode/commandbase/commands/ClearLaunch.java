package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSystem;

public class ClearLaunch extends CommandBase {
    private final ShooterSystem shooterSystem;
    private final Intake intake;
    private final ElapsedTime timer;
    private final boolean preciseShots;

    // 設定射擊超時時間 (ms)，建議根據實際射速調整
    private static final long TIMEOUT_MS = 1000;

    /**
     * 預設為快速射擊模式 (不等待轉速完全穩定，適合近距離)
     */
    public ClearLaunch(ShooterSystem shooterSystem, Intake intake) {
        this(shooterSystem, intake, false);
    }

    /**
     * @param preciseShots true: 等待飛輪轉速達標才推彈 (適合遠距離); false: 全速推彈
     */
    public ClearLaunch(ShooterSystem shooterSystem, Intake intake, boolean preciseShots) {
        this.shooterSystem = shooterSystem;
        this.intake = intake;
        this.preciseShots = preciseShots;
        this.timer = new ElapsedTime();

        // 宣告這條命令會占用這些子系統 (Command Scheduler 會自動處理衝突)
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        // 1. 獲取飛輪狀態
        boolean isReady = shooterSystem.isReady();

        if (preciseShots && !isReady) {
            // 精確模式且轉速掉下來了 -> 暫停進彈，等待回轉
            intake.setState(Intake.IntakeState.STOP); // 假設你的 Intake 有這個方法
            shooterSystem.stopShoot(); // 收回推桿
        } else {
            // 轉速夠了 或 處於無腦掃射模式 -> 全力進彈
            intake.setState(Intake.IntakeState.INTAKE);
            shooterSystem.shootIfReady(); // 觸發推桿 (ShooterSystem 內部會處理 Servo)
        }
    }

    @Override
    public void end(boolean interrupted) {
        // 命令結束時，停止進彈和推桿
        intake.setState(Intake.IntakeState.STOP);
        shooterSystem.stopShoot();

    }

    @Override
    public boolean isFinished() {
        // 目前維持時間控制
        return timer.milliseconds() > TIMEOUT_MS;
    }
}