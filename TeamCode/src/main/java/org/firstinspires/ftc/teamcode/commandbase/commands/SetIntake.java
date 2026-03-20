package org.firstinspires.ftc.teamcode.commandbase.commands;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;

public class SetIntake extends CommandBase {
    private final Intake intake;
    private final Intake.IntakeState motorState;
    private final boolean waitForArtifacts;
    private final ElapsedTime timer;

    // 設定超時保護 (例如吸了 3 秒還沒滿，就強制結束，避免卡死)
    private static final double TIMEOUT_SEC = 3.0;

    /**
     * 模式 A: 瞬時設定 (Fire and Forget)
     * 設定完狀態後，Command 立刻結束，但馬達會繼續轉。
     */
    public SetIntake(Intake intake, Intake.IntakeState motorState) {
        this(intake, motorState, false);
    }

    /**
     * 模式 B: 條件等待 (Conditional Wait)
     * @param waitForArtifacts true: 直到感測器偵測到物體才結束 Command
     */
    public SetIntake(Intake intake, Intake.IntakeState motorState, boolean waitForArtifacts) {
        this.intake = intake;
        this.motorState = motorState;
        this.waitForArtifacts = waitForArtifacts;
        this.timer = new ElapsedTime();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        intake.setState(motorState);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (waitForArtifacts) {
            intake.setState(Intake.IntakeState.STOP);
        }
    }

    @Override
    public boolean isFinished() {
        if (waitForArtifacts) {
            // Current Overload
            boolean isFull = intake.getIntakeCurrent() > 6;
            return isFull;
        }
        // 如果不等待，直接結束
        return true;
    }
}