//package org.firstinspires.ftc.teamcode.commandbase.subsystems.turret;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.RobotLog;
//import com.seattlesolvers.solverslib.command.SubsystemBase;
//
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
//import org.firstinspires.ftc.teamcode.constant.Constant;
//import org.firstinspires.ftc.teamcode.util.controller.PIDFController;
//import org.firstinspires.ftc.teamcode.util.controller.ProfiledPIDController;
//
//@Config
//public class TestTurret extends SubsystemBase {
//    // 定義安全電壓範圍
//    private static final double SAFE_MIN_V = 0.5;
//    private static final double SAFE_MAX_V = 2.8;
//
//    // === 1. PIDF 調參 (Profiled 模式下 P 通常可以給大一點) ===
//    public static double kP = 0.002; // 建議重新調校
//    public static double kI = 0.05;
//    public static double kD = 0.0001;
//
//    // [新增] Motion Profile 參數 (單位: 度/秒, 度/秒^2)
//    public static double MAX_VEL = 360.0;   // 假設一秒轉一圈
//    public static double MAX_ACCEL = 360.0; // 一秒內加速到滿速
//    public static double kV = 0.008;        // 速度前饋 (需實驗: 1 / MaxSpeed)
//    public static double kA = 0;       // 加速度前饋
//
//    // Auto 專用參數
//    public static double kP_Auto = 0.002;
//    public static double kI_Auto = 0.05;
//    public static double kD_Auto = 0.0001;
//
//    // 正反向靜摩擦力
//    public static double kStatic_Pos = 0.06;
//    public static double kStatic_Neg = 0.06;
//    public static double Izone = 10;
//
//    // === 2. 物理參數 ===
//    public static double REAL_MAX_V = 3.25;
//    public static double GEAR_RATIO = 27.0 / 50.0;
//    public static double ANGLE_OFFSET = 140.4;
//    public static double LR_PHASE_OFFSET = -106;
//
//    // === 3. 軟體限位 ===
//    public static double MAX_ANGLE = 90.0;
//    public static double MIN_ANGLE = -90.0;
//    public static double TOLERANCE = 1.0;
//    public static double FILTER_ALPHA = 0.2;
//    // [修改] 換成 Profiled 控制器
//    public final ProfiledPIDController controller;
//    // --- 硬體與感測器 ---
//    private final CRServo servoDriveL, servoDriveR;
//    private final AnalogInput feedbackSensorL, feedbackSensorR;
//    private final Servo servoHoodL, servoHoodR;
//    private final VoltageMonitor voltageMonitor;
//    private final IMU imu;
//    // --- Debug ---
//    public double debug_currentAngle = 0;
//    public double debug_kStatic = 0;
//    public double debug_finalPower = 0;
//    public double debug_looptime = 0;
//    public double maxLoopTime = 0; // [新增] 監控最大延遲
//
//    // --- 狀態變數 ---
//    public double lastChassisVel = 0;
//    public double lastTime = 0;
//    public double accelFilterAlpha = 0.2;
//    public double currentChassisAccel = 0;
//    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//    private double targetAngle = 0.0;
//    private double lastTargetAngle = -999.0; // 用於檢測目標是否改變
//    private double lastServoAngle = 0;
//    private int rotationCount = 0;
//    private boolean isInitialized = false;
//    private double activePhaseOffset = LR_PHASE_OFFSET;
//    private double filteredAngle = 0;
//    private boolean filterInitialized = false;
//
//    // 速度計算
//    public double currentTurretVelocity = 0;
//    private double lastTurretAngleForVel = 0;
//    private double velFilterAlpha = 0.3;
//
//    public TestTurret(HardwareMap hwMap, VoltageMonitor voltageMonitor, IMU imu) {
//        this.voltageMonitor = voltageMonitor;
//        this.imu = imu;
//
//        servoDriveL = hwMap.get(CRServo.class, "turretL");
//        servoDriveR = hwMap.get(CRServo.class, "turretR");
//        servoDriveL.setDirection(CRServo.Direction.FORWARD);
//        servoDriveR.setDirection(CRServo.Direction.FORWARD);
//
//        feedbackSensorL = hwMap.get(AnalogInput.class, "turretEncoder");
//        feedbackSensorR = hwMap.get(AnalogInput.class, "turretEncoder2");
//        servoHoodL = hwMap.get(Servo.class, "RCS0");
//        servoHoodR = hwMap.get(Servo.class, "RCS1");
//
//        // [修改] 初始化 ProfiledPIDController
//        controller = new ProfiledPIDController(
//                kP, kI, kD, 0, // kF 在 update 裡動態給，這裡先給 0
//                MAX_VEL, MAX_ACCEL, kV, kA
//        );
//
//        // 設定 PID 內部積分限制 (透過 public pidController 訪問)
//        controller.pidController.setIntegrationControl(new PIDFController.IntegrationControl(
//                PIDFController.IntegrationBehavior.CLEAR_AT_SP,
//                0.8, // Decay
//                -1.0, 1.0
//        ));
//
//        tryAutoCalibrateOffset();
//        lastTurretAngleForVel = getAngle();
//    }
//
//    public boolean tryAutoCalibrateOffset() {
//        double vL = feedbackSensorL.getVoltage();
//        double vR = feedbackSensorR.getVoltage();
//        boolean isL_Safe = (vL > SAFE_MIN_V && vL < SAFE_MAX_V);
//        boolean isR_Safe = (vR > SAFE_MIN_V && vR < SAFE_MAX_V);
//
//        if (isL_Safe && isR_Safe) {
//            double degL = (vL / REAL_MAX_V) * 360.0;
//            double degR = (vR / REAL_MAX_V) * 360.0;
//            double calculatedOffset = degL - degR;
//            while (calculatedOffset > 180) calculatedOffset -= 360;
//            while (calculatedOffset < -180) calculatedOffset += 360;
//            this.activePhaseOffset = calculatedOffset;
//            return true;
//        } else {
//            this.activePhaseOffset = LR_PHASE_OFFSET;
//            return false;
//        }
//    }
//
//    public double getAngle() {
//        double vL = feedbackSensorL.getVoltage();
//        double vR = feedbackSensorR.getVoltage();
//        double degL = (Range.clip(vL, 0.01, REAL_MAX_V) / REAL_MAX_V) * 360.0;
//        double degR = (Range.clip(vR, 0.01, REAL_MAX_V) / REAL_MAX_V) * 360.0;
//        double angleL = degL;
//        double angleR = (degR + activePhaseOffset + 360.0) % 360.0;
//        double currentRawAngle;
//
//        if (!isInitialized) {
//            currentRawAngle = angleL;
//        } else {
//            double diffL = Math.abs(angleL - lastServoAngle);
//            double diffR = Math.abs(angleR - lastServoAngle);
//            if (diffL > 180) diffL = 360 - diffL;
//            if (diffR > 180) diffR = 360 - diffR;
//            double PHYSICAL_LIMIT = 20.0;
//            if (diffL > PHYSICAL_LIMIT && diffR < PHYSICAL_LIMIT) {
//                currentRawAngle = angleR;
//            } else if (diffR > PHYSICAL_LIMIT && diffL < PHYSICAL_LIMIT) {
//                currentRawAngle = angleL;
//            } else {
//                if (degL > 45 && degL < 280) {
//                    currentRawAngle = angleL;
//                } else {
//                    currentRawAngle = angleR;
//                }
//            }
//        }
//        if (!isInitialized) {
//            lastServoAngle = currentRawAngle;
//            filteredAngle = currentRawAngle;
//            rotationCount = 0;
//            isInitialized = true;
//            return (currentRawAngle * GEAR_RATIO) - ANGLE_OFFSET;
//        }
//
//        double delta = currentRawAngle - lastServoAngle;
//        if (delta < -260) rotationCount++;
//        else if (delta > 260) rotationCount--;
//
//        double continuousAngle = (rotationCount * 360.0) + currentRawAngle;
//
//        if (!filterInitialized) {
//            filteredAngle = continuousAngle;
//            filterInitialized = true;
//        } else {
//            filteredAngle = FILTER_ALPHA * continuousAngle + (1 - FILTER_ALPHA) * filteredAngle;
//        }
//
//        lastServoAngle = currentRawAngle;
//        RobotLog.vv("Turret Angle: " , String.valueOf((filteredAngle * GEAR_RATIO) - ANGLE_OFFSET));
//        return (filteredAngle * GEAR_RATIO) - ANGLE_OFFSET;
//    }
//
//    public void update() {
//        Robot.getInstance().profiler.start("Turret Loop");
//        double currentTime = timer.seconds();
//        if (lastTime == 0) lastTime = currentTime;
//        double dt = currentTime - lastTime;
//
//        // [優化] 限制最小 dt，並記錄最大延遲
//        dt = Math.max(dt, 0.002);
//        debug_looptime = dt * 1000.0;
//        maxLoopTime = Math.max(maxLoopTime, debug_looptime);
//
//        // 1. 底盤運動補償計算
//        double chassisAngularVel = imu.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
//        double rawAccel = (chassisAngularVel - lastChassisVel) / dt;
//        currentChassisAccel = (accelFilterAlpha * rawAccel) + ((1.0 - accelFilterAlpha) * currentChassisAccel);
//
//        // 2. 讀取當前角度
//        debug_currentAngle = getAngle();
//
//        // 3. 計算炮塔速度
//        double deltaAngle = debug_currentAngle - lastTurretAngleForVel;
//        double rawTurretVel = deltaAngle / dt;
//        currentTurretVelocity = (velFilterAlpha * rawTurretVel) + ((1.0 - velFilterAlpha) * currentTurretVelocity);
//        lastTurretAngleForVel = debug_currentAngle;
//
//        // 4. 動態 PIDF 參數設定
//        // 計算誤差來決定 kStatic 方向 (注意: Profiled 模式下這通常基於 Setpoint 速度，但基於 Error 也可以)
//        double error = targetAngle - debug_currentAngle;
//
//        double baseKStatic;
//        if (error > 0) baseKStatic = kStatic_Pos;
//        else baseKStatic = kStatic_Neg;
//
//        // 高速移動時降低靜摩擦補償
//        if (Math.abs(currentTurretVelocity) > 10.0) {
//            baseKStatic = 0.04;
//        }
//        debug_kStatic = baseKStatic;
//
//        // 設置 PID 係數 (注意: 這裡只更新 PIDF 的 F 部分為 kStatic)
//        if (Constant.OpModeType.AUTO == Constant.OP_MODE_TYPE) {
//            controller.setPIDF(kP_Auto, kI_Auto, kD_Auto, baseKStatic);
//        } else {
//            controller.setPIDF(kP, kI, kD, baseKStatic);
//        }
//        controller.pidController.setIZone(Izone);
//        controller.setTolerance(TOLERANCE);
//        controller.setFeedForward(kV, kA);
//
//        // 5. [關鍵] 設定 Profile 目標
//        // 只有當目標改變時，才呼叫 setGoal，這樣 Profile 才能連續規劃
//        if (Math.abs(targetAngle - lastTargetAngle) > 1) {
//            controller.setGoal(targetAngle);
//            lastTargetAngle = targetAngle;
//        }
//
//        // 6. 計算輸出 (PID + Feedforward)
//        double pidOutput = controller.calculate(debug_currentAngle);
//
//        // 7. 電壓補償
//        double compensationFactor = voltageMonitor.getCompensationFactor();
//        double totalPower = pidOutput * compensationFactor;
//
//        // 8. 軟體限位
//        if ((debug_currentAngle > MAX_ANGLE && totalPower > 0) ||
//                (debug_currentAngle < MIN_ANGLE && totalPower < 0)) {
//            totalPower = 0;
//        }
//
//        // 9. 容許誤差內停止
//        if (controller.atGoal()) {
//            totalPower = 0;
//        }
//
//        // 10. 輸出
//        debug_finalPower = Range.clip(totalPower, -1, 1);
//        servoDriveL.setPower(debug_finalPower);
//        servoDriveR.setPower(debug_finalPower);
//
//        lastTime = currentTime;
//        lastChassisVel = chassisAngularVel;
//        Robot.getInstance().profiler.end("Turret Loop");
//    }
//
//    @Override
//    public void periodic() {
//        update();
//    }
//
//    // ... 其他 getter/setter ...
//    public void adjustTargetAngle(double delta) {
//        setTargetAngle(targetAngle + delta);
//    }
//
//    public boolean atTarget(double tolerance) {
//        return Math.abs(debug_currentAngle - targetAngle) < tolerance;
//    }
//
//    // ... (Hood 相關代碼保持不變) ...
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
//    public CRServo getServoL() {
//        return servoDriveL;
//    }
//
//    public CRServo getServoR() {
//        return servoDriveR;
//    }
//
//    public double getError() {
//        return targetAngle - getAngle();
//    }
//
//    public double getTargetAngle() {
//        return targetAngle;
//    }
//
//    // --- 對外介面 ---
//    public void setTargetAngle(double angle) {
//        this.targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
//    }
//
//    public double[] getDualSensorData() { return new double[]{0,0,0,0,0}; /* 簡化 */ }
//}