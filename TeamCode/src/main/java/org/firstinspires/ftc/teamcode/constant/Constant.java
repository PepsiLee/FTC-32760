package org.firstinspires.ftc.teamcode.constant;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Constant {
    public static final double DRIVE_MOTOR_CACHING_TOLERANCE = 0.01;
    public static final double TURN_SERVO_CACHING_TOLERANCE = 0.01;
    // TeleOp 設定
    public static final double WHEEL_ALIGNMENT_TOLERANCE = 360.0; // 度
    // Drawing
    public static final double ROBOT_RADIUS = 9.0; // 機器人半徑 (英吋)
    // Intake
    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = -0.8;
    public static final double TRANSFER_POWER = 1.0;
    // Shooter
    public static final double TICKS_PER_REV = 28.0;
    public static final double GATE_CLOSE = 0.35;
    public static final double GATE_OPEN = 0.5;
    public static double GOAL_OFFSET = 0;
    // Robot constants
    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static double VOLTAGE_SENSOR_POLLING_RATE = 5; // Hz
    public static double DEFAULT_VOLTAGE = 12.67; // Volts
    // Units
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;
    // Turret
    public static double GEAR_RATIO = 0.54;
    public static double MAX_ANGLE = 70.0; // Degree
    public static double MIN_ANGLE = -70.0; // Degree
    public static double MAX_SERVO_ANGLE = 355.0; // Max Servo Angle / Max Limit Angle * Limit Angle
    public static double ZERO_POINT = 190; // Degree
    public static double SERVO_DEGREES_PER_SECOND = 1000; // Under Loading Speed (Use to estimate the rotation time)
    // Status Saving
    public static Pose2D END_POSE = new Pose2D(DISTANCE_UNIT, 0, 0, ANGLE_UNIT, 0);

    // Driver constants
    // === 參數設定 (可在 Dashboard 調整) ===
    public static double TRACK_WIDTH = 31.5;
    public static double WHEEL_BASE = 29.5;
    // Offset
    public static double LF_OFFSET = 69.38;
    public static double RF_OFFSET = 29.24;
    public static double LB_OFFSET = 271.31;
    public static double RB_OFFSET = 11.67;

    // --- Degrees ---
    public static double LF_BIAS = 0;
    public static double RF_BIAS = 0;
    public static double LB_BIAS = 0;
    public static double RB_BIAS = 0;

    // Module Servo Range
    public static double Servo_RANGE = 3.205;
    // Heading PID
    public static double HEADING_P = 0.2;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.02;
    //
    public static double HEADING_LOCK_VELOCITY_THRESHOLD = 10.0; // 度/秒

    public static Translation2d GOAL_POSE() {
        return new Translation2d(-70, (-70 * ALLIANCE_COLOR.getMultiplier()) + GOAL_OFFSET);
    }

    public static Pose2d RESET_POSE() {
        return new Pose2d(72 - DistanceUnit.CM.toInches(38) / 2, (72 - DistanceUnit.CM.toInches(41) / 2) * ALLIANCE_COLOR.getMultiplier(), Math.toRadians(180));
    }// Inches

    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum AllianceColor {
        BLUE(1), RED(-1);

        private int val;

        AllianceColor(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

}
