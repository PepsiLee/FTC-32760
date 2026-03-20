package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.constant.Constant.ALLIANCE_COLOR;

import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

public class TeleOpInputHandler {
    // 參數設定
    private static final double MIN_SPEED = 0.6;
    private static final double MAX_SPEED = 1.0;
    private static final double ROTATION_SCALE = 0.9;
    private static final double TRANSLATION_SCALE = 1.0; // 修正拼字錯誤

    private final SlewRateLimiter fwdLimiter;
    private final SlewRateLimiter strLimiter;
    private final SlewRateLimiter rotLimiter;

    public TeleOpInputHandler() {
        this.fwdLimiter = new SlewRateLimiter(3.5, -3, 0);
        this.strLimiter = new SlewRateLimiter(3.5, -3, 0);
        this.rotLimiter = new SlewRateLimiter(3.5, -3, 0);
    }

    public DriveInputs getDriveInputs(GamepadEx gamepad) {
        double triggerVal = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double brakeVal = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double speedMultiplier = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * triggerVal - (MAX_SPEED - MIN_SPEED) * brakeVal;

        // 計算 Raw Input
        double rawFwd = -gamepad.getLeftX() * speedMultiplier * ALLIANCE_COLOR.getMultiplier() * TRANSLATION_SCALE;
        double rawStr = -gamepad.getLeftY() * speedMultiplier * ALLIANCE_COLOR.getMultiplier() * TRANSLATION_SCALE;
        double rawRot = -gamepad.getRightX() * speedMultiplier * ROTATION_SCALE;

        // 通過 Rate Limiter 平滑化
        double fwd = fwdLimiter.calculate(rawFwd);
        double str = strLimiter.calculate(rawStr);
        double rot = rotLimiter.calculate(rawRot);

        return new DriveInputs(fwd, str, rot);
    }

    public static class DriveInputs {
        public final double forward;
        public final double strafe;
        public final double rotation;

        public DriveInputs(double f, double s, double r) {
            this.forward = f;
            this.strafe = s;
            this.rotation = r;
        }
    }
}