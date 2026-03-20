package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Rgb extends SubsystemBase {

    private final ServoEx rgb;

    /**
     * Color of the indicator.
     */
    public enum IndicatorColor {
        OFF(0.0),
        RED(0.28),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.500),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0);

        public final double position;
        IndicatorColor(double position) {
            this.position = position;
        }
    }

    public Rgb(HardwareMap hwMap) {
        // 假設名稱為 "rgb"，請確保與 Configuration 一致
        rgb = new ServoEx(hwMap, "rgb");
        // 根據文件設定 PWM 範圍為 500us - 2500us
        rgb.setPwm(new PwmControl.PwmRange(500, 2500));
    }


    /**
     * Set the color of the indicator.
     * @param color
     */
    public void setColor(IndicatorColor color) {
        rgb.set(color.position);
    }

    /**
     * Set the color of the indicator.
     */
    public void setColor(double color) {
        rgb.set(color);
    }
}