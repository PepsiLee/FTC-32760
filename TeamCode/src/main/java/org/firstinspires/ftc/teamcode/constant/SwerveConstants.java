package org.firstinspires.ftc.teamcode.constant;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SwerveConstants {
    // --- 驅動參數 ---
    public double maxPower = 1;
    public double xVelocity = 80.46;
    public double yVelocity = 65.43028;

    // --- 底盤幾何 (單位: Inches 或 Meters，需統一) ---
    public double TRACK_WIDTH = 31.5;
    public double WHEEL_BASE = 29.5;

    // --- 模組偏移量 (單位: Degrees，SwerveDrive 會自動轉弧度) ---
    public double LF_OFFSET = 118.99;
    public double RF_OFFSET = 8.75;
    public double LB_OFFSET = 186.06;
    public double RB_OFFSET = 155.21;

    public static double[] LUT_LF = {0.100, 0.106, 0.100, 0.087, 0.096, 0.079, 0.085, 0.095, 0.108, 0.110, 0.106, 0.096, 0.086, 0.084, 0.090, 0.086, 0.087, 0.098, 0.091, 0.092, 0.080, 0.083, 0.076, 0.077, 0.080, 0.085, 0.086, 0.092, 0.088, 0.086, 0.085, 0.083, 0.082, 0.086, 0.090, 0.101};
    public static double[] LUT_RF = {0.083, 0.084, 0.084, 0.083, 0.085, 0.087, 0.086, 0.086, 0.083, 0.084, 0.083, 0.086, 0.089, 0.092, 0.091, 0.098, 0.110, 0.096, 0.085, 0.089, 0.098, 0.106, 0.109, 0.114, 0.096, 0.109, 0.087, 0.083, 0.091, 0.095, 0.097, 0.089, 0.098, 0.079, 0.085, 0.083};
    public static double[] LUT_LB = {0.084, 0.085, 0.079, 0.091, 0.103, 0.111, 0.095, 0.096, 0.072, 0.089, 0.086, 0.090, 0.103, 0.111, 0.096, 0.112, 0.103, 0.096, 0.082, 0.087, 0.090, 0.087, 0.088, 0.093, 0.076, 0.084, 0.082, 0.085, 0.082, 0.079, 0.082, 0.083, 0.090, 0.088, 0.085, 0.084};
    public static double[] LUT_RB = {0.081, 0.080, 0.087, 0.085, 0.099, 0.091, 0.084, 0.088, 0.082, 0.080, 0.085, 0.093, 0.100, 0.096, 0.085, 0.081, 0.078, 0.083, 0.079, 0.082, 0.082, 0.086, 0.084, 0.082, 0.082, 0.079, 0.081, 0.079, 0.081, 0.081, 0.084, 0.079, 0.084, 0.074, 0.082, 0.079};

    // --- 硬體名稱 (Hardware Mapping) ---
    // Motors
    public String leftFrontMotorName = "leftFront";
    public String leftRearMotorName = "leftRear";
    public String rightFrontMotorName = "rightFront";
    public String rightRearMotorName = "rightRear";

    // Servos
    public String leftFrontServoName = "leftFrontServo";
    public String leftRearServoName = "leftRearServo";
    public String rightFrontServoName = "rightFrontServo";
    public String rightRearServoName = "rightRearServo";

    // --- 馬達方向 ---
    public  DcMotor.Direction leftFrontMotorDirection = DcMotor.Direction.REVERSE;
    public  DcMotor.Direction leftRearMotorDirection = DcMotor.Direction.REVERSE;
    public  DcMotor.Direction rightFrontMotorDirection = DcMotor.Direction.FORWARD;
    public  DcMotor.Direction rightRearMotorDirection = DcMotor.Direction.FORWARD;

    // --- 進階控制 ---
    public double motorCachingThreshold = 0.01;
    public boolean useBrakeModeInTeleOp = false;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;

    private double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);

    public SwerveConstants() {
        defaults();
    }

    // =========================================================
    // Builder Pattern Methods (鏈式設定方法)
    // =========================================================

    public SwerveConstants trackWidth(double trackWidth) {
        this.TRACK_WIDTH = trackWidth;
        return this;
    }

    public SwerveConstants wheelBase(double wheelBase) {
        this.WHEEL_BASE = wheelBase;
        return this;
    }

    public SwerveConstants lfOffset(double offset) {
        this.LF_OFFSET = offset;
        return this;
    }

    public SwerveConstants rfOffset(double offset) {
        this.RF_OFFSET = offset;
        return this;
    }

    public SwerveConstants lbOffset(double offset) {
        this.LB_OFFSET = offset;
        return this;
    }

    public SwerveConstants rbOffset(double offset) {
        this.RB_OFFSET = offset;
        return this;
    }

    public SwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public SwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public SwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    // --- Motor Names ---
    public SwerveConstants leftFrontMotorName(String name) {
        this.leftFrontMotorName = name;
        return this;
    }

    public SwerveConstants leftRearMotorName(String name) {
        this.leftRearMotorName = name;
        return this;
    }

    public SwerveConstants rightFrontMotorName(String name) {
        this.rightFrontMotorName = name;
        return this;
    }

    public SwerveConstants rightRearMotorName(String name) {
        this.rightRearMotorName = name;
        return this;
    }

    // --- Servo Names (新增) ---
    public SwerveConstants leftFrontServoName(String name) {
        this.leftFrontServoName = name;
        return this;
    }

    public SwerveConstants leftRearServoName(String name) {
        this.leftRearServoName = name;
        return this;
    }

    public SwerveConstants rightFrontServoName(String name) {
        this.rightFrontServoName = name;
        return this;
    }

    public SwerveConstants rightRearServoName(String name) {
        this.rightRearServoName = name;
        return this;
    }

    // --- Other Settings ---
    public SwerveConstants motorCachingThreshold(double threshold) {
        this.motorCachingThreshold = threshold;
        return this;
    }

    public SwerveConstants useBrakeModeInTeleOp(boolean useBrake) {
        this.useBrakeModeInTeleOp = useBrake;
        return this;
    }

    public SwerveConstants useVoltageCompensation(boolean useCompensation) {
        this.useVoltageCompensation = useCompensation;
        return this;
    }

    public SwerveConstants nominalVoltage(double voltage) {
        this.nominalVoltage = voltage;
        return this;
    }

    public SwerveConstants staticFrictionCoefficient(double coefficient) {
        this.staticFrictionCoefficient = coefficient;
        return this;
    }

    // =========================================================
    // Default Values (預設值重置)
    // =========================================================
    public void defaults() {
        xVelocity = 74.53;
        yVelocity = 72.69;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        maxPower = 1;

        TRACK_WIDTH = 31.5;
        WHEEL_BASE = 29.5;

        LF_OFFSET = 0;
        RF_OFFSET = 0;
        LB_OFFSET = 0;
        RB_OFFSET = 0;

        leftFrontMotorName = "lf";
        leftRearMotorName = "lr";
        rightFrontMotorName = "rf";
        rightRearMotorName = "rr";

        leftFrontServoName = "rrs";
        leftRearServoName = "lrs";
        rightFrontServoName = "rfs";
        rightRearServoName = "rrs";

        leftFrontMotorDirection = DcMotor.Direction.REVERSE;
        leftRearMotorDirection = DcMotor.Direction.REVERSE;
        rightFrontMotorDirection = DcMotor.Direction.FORWARD;
        rightRearMotorDirection = DcMotor.Direction.FORWARD;

        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
        useVoltageCompensation = true;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }

    // =========================================================
    // Getters and Setters (標準存取方法)
    // =========================================================
    public double getXVelocity() { return xVelocity; }
    public void setXVelocity(double xVelocity) { this.xVelocity = xVelocity; }

    public double getYVelocity() { return yVelocity; }
    public void setYVelocity(double yVelocity) { this.yVelocity = yVelocity; }

    public double getMaxPower() { return maxPower; }
    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }

    // Motor Names
    public String getLeftFrontMotorName() { return leftFrontMotorName; }
    public void setLeftFrontMotorName(String name) { this.leftFrontMotorName = name; }

    public String getLeftRearMotorName() { return leftRearMotorName; }
    public void setLeftRearMotorName(String name) { this.leftRearMotorName = name; }

    public String getRightFrontMotorName() { return rightFrontMotorName; }
    public void setRightFrontMotorName(String name) { this.rightFrontMotorName = name; }

    public String getRightRearMotorName() { return rightRearMotorName; }
    public void setRightRearMotorName(String name) { this.rightRearMotorName = name; }

    // Motor Directions
    public DcMotor.Direction getLeftFrontMotorDirection() { return leftFrontMotorDirection; }
    public void setLeftFrontMotorDirection(DcMotor.Direction direction) { this.leftFrontMotorDirection = direction; }

    public DcMotor.Direction getLeftRearMotorDirection() { return leftRearMotorDirection; }
    public void setLeftRearMotorDirection(DcMotor.Direction direction) { this.leftRearMotorDirection = direction; }

    public DcMotor.Direction getRightFrontMotorDirection() { return rightFrontMotorDirection; }
    public void setRightFrontMotorDirection(DcMotor.Direction direction) { this.rightFrontMotorDirection = direction; }

    public DcMotor.Direction getRightRearMotorDirection() { return rightRearMotorDirection; }
    public void setRightRearMotorDirection(DcMotor.Direction direction) { this.rightRearMotorDirection = direction; }

    public double getMotorCachingThreshold() { return motorCachingThreshold; }
    public void setMotorCachingThreshold(double threshold) { this.motorCachingThreshold = threshold; }

    public boolean isUseBrakeModeInTeleOp() { return useBrakeModeInTeleOp; }
    public void setUseBrakeModeInTeleOp(boolean useBrake) { this.useBrakeModeInTeleOp = useBrake; }

    public SwerveConstants leftFrontMotorDirection(DcMotor.Direction direction) {
        this.leftFrontMotorDirection = direction;
        return this;
    }

    public SwerveConstants leftRearMotorDirection(DcMotor.Direction direction) {
        this.leftRearMotorDirection = direction;
        return this;
    }

    public SwerveConstants rightFrontMotorDirection(DcMotor.Direction direction) {
        this.rightFrontMotorDirection = direction;
        return this;
    }

    public SwerveConstants rightRearMotorDirection(DcMotor.Direction direction) {
        this.rightRearMotorDirection = direction;
        return this;
    }

}