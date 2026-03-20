package org.firstinspires.ftc.teamcode.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake extends SubsystemBase {

    // === Tuning / Constants ===
    // Intake (吸頭) 的速度
    public static double INTAKE_SPEED_IN = 1.0;   // 吸入全速
    public static double INTAKE_SPEED_TRANSFER = 0.85;
    public static double INTAKE_SPEED_OUT = -1.0; // 吐出全速

    // Transfer (輸送帶) 的速度
    public static double TRANSFER_SPEED_IN = 0.7;  // 往上/往內送
    public static double TRANSFER_SPEED_TRANSFER_FAST = 1.0;  // 往上/往內送
    public static double TRANSFER_SPEED_TRANSFER = 0.82;
    public static double TRANSFER_SPEED_OUT = -1.0; // 退料

    // === Hardware ===
    // 假設 Intake 還是兩顆馬達 (如果變一顆，刪除 intakeR 即可)
    private final MotorEx intake;

    // 新增 Transfer 馬達
    private final MotorEx transferMotor;

    // === State Variables ===
    private IntakeState currentState = IntakeState.STOP;

    private AnalogInput distance;

    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    public Intake(final HardwareMap hardwareMap) {
        // 請確保 Config 名字跟 Driver Station 設定的一樣
        intake = new MotorEx(hardwareMap, "Intake", Motor.GoBILDA.BARE);
        transferMotor = new MotorEx(hardwareMap, "Transfer", Motor.GoBILDA.BARE);
        distance = hardwareMap.get(AnalogInput.class, "dis");

        configureMotors();

        intake.setInverted(true); // 如果輸送帶方向相反，解開這行
    }

    private void configureMotors() {
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intake.setRunMode(Motor.RunMode.RawPower);

        transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transferMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void setState(IntakeState state) {
        this.currentState = state;
    }

    public IntakeState getState() {
        return currentState;
    }

    public double getDistanceMM(){
        return (distance.getVoltage() / MAX_VOLTS) * MAX_DISTANCE_MM;
    }

    /**
     * 核心邏輯：定義每個狀態下，Intake 和 Transfer 分別該怎麼動
     */
    private void update() {
        double intakePower = 0;
        double transferPower = 0;

        switch (currentState) {
            case INTAKE:
                // 狀態：吸入。Intake 轉，Transfer 也轉 (把東西帶進來)
                intakePower = INTAKE_SPEED_IN;
                transferPower = TRANSFER_SPEED_IN;
                break;

            case INTAKE_ONLY:
                // 狀態：只吸不送 (例如已經有一顆在 Transfer 裡了，想吸第二顆)
                intakePower = INTAKE_SPEED_IN;
                transferPower = 0;
                break;

            case TRANSFER_ONLY:
                // 狀態：只送不吸 (例如把東西調整位置，或者 Intake 卡住不想動)
                intakePower = 0;
                transferPower = TRANSFER_SPEED_IN;
                break;

            case OUTTAKE:
                // 狀態：全部吐出
                intakePower = INTAKE_SPEED_OUT;
                transferPower = TRANSFER_SPEED_OUT;
                break;
            case TRANSFER_FAST:
                intakePower = INTAKE_SPEED_IN;
                transferPower = TRANSFER_SPEED_TRANSFER_FAST;
                break;
            case TRANSFER:
                // 狀態：全部吐出
                intakePower = INTAKE_SPEED_TRANSFER;
                transferPower = TRANSFER_SPEED_TRANSFER;
                break;
            case TRASFER_OUTTAKE:
                intakePower = INTAKE_SPEED_TRANSFER;
                transferPower = TRANSFER_SPEED_OUT;
            case STOP:
            default:
                intakePower = 0;
                transferPower = 0;
                break;
        }

        // 寫入馬達
        intake.set(intakePower);
        transferMotor.set(transferPower);
    }

    @Override
    public void periodic() {
        update();
    }

    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public double getTransferCurrent() {
        return transferMotor.getCurrent(CurrentUnit.AMPS);
    }

    // === State Enums ===
    public enum IntakeState {
        INTAKE,         // 兩者都轉 (標準吸入)
        INTAKE_ONLY,    // 只轉吸頭 (進階控制用)
        TRANSFER_ONLY,  // 只轉輸送帶 (索引/調整用)
        TRANSFER,
        TRANSFER_FAST,
        OUTTAKE,        // 全部反轉
        TRASFER_OUTTAKE,
        STOP            // 停止
    }
}