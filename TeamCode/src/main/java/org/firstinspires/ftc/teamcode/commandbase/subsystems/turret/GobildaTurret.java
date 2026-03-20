//package org.firstinspires.ftc.teamcode.commandbase.subsystems.turret;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.command.SubsystemBase;
//
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
//import org.firstinspires.ftc.teamcode.constant.Constant;
//
//@Config
//public class GobildaTurret extends SubsystemBase {
//    // --- 硬體物件 ---
//    private final Servo servoDrive;
//    private final Servo servoHoodL, servoHoodR;
//    // --- Debug 變數 ---
//    double totalWaitTime;
//    // --- 狀態變數 ---
//    private double targetAngle = 0.0;
//    private double lastServoAngle = 0;
//    private double angle = 0;
//    private ElapsedTime rotationTimer;
//
//    public GobildaTurret(HardwareMap hwMap, VoltageMonitor voltageMonitor, IMU imu) {
//        servoDrive = hwMap.get(Servo.class, "turret");
//        servoDrive.setDirection(Servo.Direction.FORWARD);
//
//        servoHoodL = hwMap.get(Servo.class, "HoodOne");
//        servoHoodR = hwMap.get(Servo.class, "HoodTwo");
//
//        rotationTimer = new ElapsedTime();
//    }
//
//    public void update() {
//        if (rotationTimer.milliseconds() > totalWaitTime) {
//            lastServoAngle = angle;
//        }
//    }
//
//    @Override
//    public void periodic() {
//        update();
//    }
//
//    public double getAngle() {
//        return angle;
//    }
//
//    // --- 對外介面 ---
//    public void setTargetAngle(double angle) {
//        this.targetAngle = Range.clip(angle, Constant.MIN_ANGLE, Constant.MAX_ANGLE);
//        double servoAngle = (Constant.ZERO_POINT / Constant.MAX_SERVO_ANGLE) + ((targetAngle / Constant.GEAR_RATIO) / Constant.MAX_SERVO_ANGLE);
//        servoDrive.setPosition(servoAngle);
//        rotationTimer.reset();
//
//        double deltaAngle = Math.abs(targetAngle - lastServoAngle);
//        double estimatedTimeNeeded = (deltaAngle / Constant.SERVO_DEGREES_PER_SECOND) * 1000; // 換算成 ms
//        totalWaitTime = estimatedTimeNeeded + 100;
//    }
//
//    public void adjustTargetAngle(double delta) {
//        setTargetAngle(targetAngle + delta);
//    }
//
//    public boolean atTarget() {
//        return true; //TODO: SOLVE THE fast setTarget made the timer not satisfied
////        return rotationTimer.milliseconds() > totalWaitTime;
//    }
//
//    public double getHoodPosition() {
//        return servoHoodL.getPosition();
//    }
//
//    public void setHoodPosition(double pos) {
//        pos = Range.clip(pos, 0, 1);
//        servoHoodL.setPosition(pos);
//        servoHoodR.setPosition(pos);
//    }
//
//    public Double getError() {
//        return targetAngle - getAngle();
//    }
//
//    public Double getTargetAngle() {
//        return targetAngle;
//    }
//
//}