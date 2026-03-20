package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;

public class IntaketTestCommand extends CommandBase {
    private final Intake intake;

    // === 參數設定 ===
    private static final double JAM_THRESHOLD = 5;
    private static final double JAM_DURATION_LIMIT = 300;
    private static final double REVERSE_DURATION = 400;
    private static final double RECOVERY_COOLDOWN = 200;

    // --- 距離感測器新增參數 ---
    private static final double STOP_DISTANCE_MM = 50.0;     // 50mm (5cm)
    private static final double DISTANCE_THRESHOLD_TIME = 150; // 持續偵測到球 150ms 才停止

    private enum State {
        NORMAL_RUNNING,
        POTENTIAL_JAM,
        UNJAMMING,
        RECOVERING,
        POTENTIAL_FULL,
        AUTOMATIC_STOPPED
    }

    private State currentState;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime distanceTimer = new ElapsedTime();

    public IntaketTestCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(Intake.IntakeState.INTAKE);
        currentState = State.NORMAL_RUNNING;
        stateTimer.reset();
        distanceTimer.reset();
    }

    @Override
    public void execute() {
        double currentAmps = intake.getTransferCurrent();
        double currentDistance = intake.getDistanceMM();

        switch (currentState) {
            case NORMAL_RUNNING:
                // 優先判定距離
                if (currentDistance < STOP_DISTANCE_MM) {
                    currentState = State.POTENTIAL_FULL;
                    distanceTimer.reset();
                } else if (currentAmps > JAM_THRESHOLD) {
                    currentState = State.POTENTIAL_JAM;
                    stateTimer.reset();
                }
                intake.setState(Intake.IntakeState.INTAKE);
                break;

            case POTENTIAL_FULL:
                // 如果球突然移開了，回到正常運作
                if (currentDistance > STOP_DISTANCE_MM) {
                    currentState = State.NORMAL_RUNNING;
                }
                // 如果球持續待在位置上超過時間
                else if (distanceTimer.milliseconds() > DISTANCE_THRESHOLD_TIME) {
                    currentState = State.AUTOMATIC_STOPPED;
                }
                break;

            case AUTOMATIC_STOPPED:
                intake.setState(Intake.IntakeState.STOP);
                // 如果球被拿走了，自動恢復吸球
                if (currentDistance > STOP_DISTANCE_MM + 20) { // 加點緩衝避免抖動
                    currentState = State.NORMAL_RUNNING;
                }
                break;

            case POTENTIAL_JAM:
                if (currentAmps < JAM_THRESHOLD) {
                    currentState = State.NORMAL_RUNNING;
                } else if (stateTimer.milliseconds() > JAM_DURATION_LIMIT) {
                    startUnjamming();
                }
                break;

            case UNJAMMING:
                if (stateTimer.milliseconds() > REVERSE_DURATION) {
                    startRecovering();
                }
                break;

            case RECOVERING:
                if (stateTimer.milliseconds() > RECOVERY_COOLDOWN) {
                    currentState = State.NORMAL_RUNNING;
                }
                break;
        }
    }

    private void startUnjamming() {
        currentState = State.UNJAMMING;
        intake.setState(Intake.IntakeState.INTAKE_ONLY);
        stateTimer.reset();
    }

    private void startRecovering() {
        currentState = State.RECOVERING;
        intake.setState(Intake.IntakeState.INTAKE);
        stateTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(Intake.IntakeState.STOP);
    }
}