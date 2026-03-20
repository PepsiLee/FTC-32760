package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Rgb;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier enabledSupplier;

    // === 參數設定 ===
    private static final double JAM_THRESHOLD = 4;           // Transfer 電流閾值
    private static final double INTAKE_JAM_THRESHOLD = 2;    // Intake 電流閾值
    private static final double JAM_CONFIRM_TIME = 300;      // 電流確認時間 (ms)

    // Unjamming 超時保護：如果在只開Intake模式太久都沒反應，嘗試切回正常
    private static final double UNJAMMING_TIMEOUT = 3000;

    // --- 距離感測器參數 --
    private static final double STOP_DISTANCE_MM = 130;
    // private static final double DISTANCE_THRESHOLD_TIME = 300; // 目前沒用到，先註解掉

    private enum State {
        NORMAL_RUNNING,
        POTENTIAL_JAM,
        UNJAMMING,         // 執行 "INTAKE_ONLY"
        POTENTIAL_FULL,
        AUTOMATIC_STOPPED, // 吸滿正常結束
    }

    private State currentState;
    private final ElapsedTime stateTimer = new ElapsedTime();
    // 建議：共用 stateTimer 即可，因為狀態是線性的，每次切換狀態都 reset stateTimer

    public IntakeCommand(Intake intake, BooleanSupplier enabledSupplier) {
        this.intake = intake;
        this.enabledSupplier = enabledSupplier;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(Intake.IntakeState.INTAKE);
        currentState = State.NORMAL_RUNNING;
        stateTimer.reset();
    }

    @Override
    public void execute() {
        // 1. 總開關檢查
        if (!enabledSupplier.getAsBoolean()) {
            intake.setState(Intake.IntakeState.STOP);
            currentState = State.NORMAL_RUNNING; // 關閉時重置狀態，避免下次開啟時狀態錯誤
            return;
        }


        double currentAmps = intake.getTransferCurrent(); // Transfer 電流
        double intakeAmps = intake.getIntakeCurrent();    // Intake 電流
        double currentDistance = intake.getDistanceMM();

        switch (currentState) {
            case NORMAL_RUNNING:
                intake.setState(Intake.IntakeState.INTAKE);

                // 檢查是否高電流 (Transfer 卡住)
                if (currentAmps > JAM_THRESHOLD) {
                    currentState = State.POTENTIAL_JAM;
                    stateTimer.reset(); // 開始計時
                }
                break;

            case POTENTIAL_JAM:
                intake.setState(Intake.IntakeState.INTAKE);

                if (currentAmps < JAM_THRESHOLD) {
                    // 電流恢復正常，回到一般模式
                    currentState = State.NORMAL_RUNNING;
                } else if (stateTimer.milliseconds() > JAM_CONFIRM_TIME) {
                    // 確認卡住，進入 "只開 Intake" 模式
                    currentState = State.UNJAMMING;
                    stateTimer.reset(); // [修正] 進入新狀態要重置 timer
                }
                break;

            case UNJAMMING:
                intake.setState(Intake.IntakeState.INTAKE_ONLY);

                if (intakeAmps > INTAKE_JAM_THRESHOLD) {
                    // Intake 也開始高電流了，可能吸滿或到底了
                    currentState = State.POTENTIAL_FULL;
                    stateTimer.reset(); // [重要修正] 必須重置 Timer 才能在下一個狀態正確計時
                }
                else if (stateTimer.milliseconds() > UNJAMMING_TIMEOUT) {
                    currentState = State.NORMAL_RUNNING;
                }
                break;

            case POTENTIAL_FULL:
                intake.setState(Intake.IntakeState.INTAKE_ONLY); // 保持狀態

                if (intakeAmps < INTAKE_JAM_THRESHOLD) {
                    // 電流掉下來了，可能是雜訊，回到上一個狀態(UNJAMMING) 或 NORMAL
                    currentState = State.UNJAMMING;
                    stateTimer.reset();
                } else if (stateTimer.milliseconds() > JAM_CONFIRM_TIME) {
                    // 確認真的堵轉/吸滿了
                    currentState = State.AUTOMATIC_STOPPED;
                }
                break;

            case AUTOMATIC_STOPPED:
                intake.setState(Intake.IntakeState.STOP);

                // 當物體離開 (距離變大)，重新啟動
                if (currentDistance <= STOP_DISTANCE_MM + 20) {
                    // 如果還有物體，或是數值亂跳回到有物體的範圍，就把計時器歸零
                    stateTimer.reset();
                }

                // 只有當「持續」偵測不到物體超過 200ms，才視為真的空了
                if (stateTimer.milliseconds() > 100) {
                    currentState = State.NORMAL_RUNNING;
                    stateTimer.reset();
                }
                break;
        }

        if(currentState == State.AUTOMATIC_STOPPED){
            Robot.getInstance().rgb.setColor(Rgb.IndicatorColor.GREEN);
        }else{
            Robot.getInstance().rgb.setColor(Rgb.IndicatorColor.ORANGE);
        }
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