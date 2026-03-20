package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.constant.Constant.DEFAULT_VOLTAGE;
import static org.firstinspires.ftc.teamcode.constant.Constant.VOLTAGE_SENSOR_POLLING_RATE;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class VoltageMonitor extends SubsystemBase {
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private double cachedVoltage = 12.0;

    public VoltageMonitor(HardwareMap hwMap) {
        voltageSensor = hwMap.voltageSensor.iterator().next();
        timer.reset();
        cachedVoltage = voltageSensor.getVoltage();
    }

    @Override
    public void periodic() {
        // 定期更新快取
        if (timer.milliseconds() > 1000 / VOLTAGE_SENSOR_POLLING_RATE) {
            double reading = voltageSensor.getVoltage();
            if (!Double.isNaN(reading) && reading > 5.0) {
                cachedVoltage = reading;
            }
            timer.reset();
        }
    }

    public double getVoltage() {
        return cachedVoltage;
    }

    /**
     * 獲取補償倍率
     * 例如：基準 12V，現在 10V -> 回傳 1.2 (需要輸出 1.2 倍的力量)
     */
    public double getCompensationFactor() {
        double factor = DEFAULT_VOLTAGE / cachedVoltage;
        // 限制倍率
        return Range.clip(factor, 0.6, 1.5);
    }
}