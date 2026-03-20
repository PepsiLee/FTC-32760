//package org.firstinspires.ftc.teamcode.commandbase.subsystems.turret;
//
//import static java.lang.Math.abs;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.command.SubsystemBase;
//
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.VoltageMonitor;
//import org.firstinspires.ftc.teamcode.constant.Constant;
//import org.firstinspires.ftc.teamcode.util.controller.PIDFController;
//
//@Config
//public class TwoAxonTurret extends SubsystemBase {
//    // 定義安全電壓範圍 (避開死區)
//    private static final double SAFE_MIN_V = 0.5;
//    private static final double SAFE_MAX_V = 2.8;
//
//    // === 1. PIDF 調參 ===
//    public static double kP = 0.0033;
//    public static double kI = 0;
//    public static double kD = 0.00065;
//
//    // === Auto 專用 PIDF 參數 ===
//    public static double kP_Auto = 0.003;
//    public static double kI_Auto = 0;
//    public static double kD_Auto = 0.00065;
//
//    // [修改] 拆分正反向靜摩擦力
//    public static double kStatic_Pos = 0.06;
//    public static double kStatic_Neg = 0.065;
//    public static double Izone = 10;
//
//    // === 2. 物理與硬體校準參數 ===
//    public static double REAL_MAX_V = 3.25;
//    public static double GEAR_RATIO = 27.0 / 50.0;
//    public static double ANGLE_OFFSET = 145.4;
//    public static double LR_PHASE_OFFSET = -106;
//
//    // === 3. 軟體限位 ===
//    public static double MAX_ANGLE = 90.0;
//    public static double MIN_ANGLE = -90.0;
//    public static double TOLERANCE = 1.0;
//    public static double FILTER_ALPHA = 0.2;
//
//    // --- 硬體物件 ---
//    private final CRServo servoDriveL, servoDriveR;
//    private final AnalogInput feedbackSensorL, feedbackSensorR;
//    private final Servo servoHoodL, servoHoodR;
//    private final VoltageMonitor voltageMonitor;
//    private final IMU imu;
//    public final PIDFController controller; //TODO: FOR debug
//
//    // --- Debug 變數 ---
//    public double debug_currentAngle = 0;
//    public double debug_kStatic = 0;
//    public double debug_finalPower = 0;
//
//    // [新增] Loop Time 監控變數 (單位: ms)
//    public double debug_looptime = 0;
//
//    public double lastChassisVel = 0;
//    public double lastTime = 0;
//    public double accelFilterAlpha = 0.2;
//    public double currentChassisAccel = 0;
//    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//    // --- 狀態變數 ---
//    private double targetAngle = 0.0;
//    private double lastServoAngle = 0;
//    private int rotationCount = 0;
//    private boolean isInitialized = false;
//    private double activePhaseOffset = LR_PHASE_OFFSET;
//    private double filteredAngle = 0;
//    private boolean filterInitialized = false;
//
//    // --- 速度計算變數 ---
//    private double currentTurretVelocity = 0;
//    private double lastTurretAngleForVel = 0;
//    private double velFilterAlpha = 0.8;
//
//    public TwoAxonTurret(HardwareMap hwMap, VoltageMonitor voltageMonitor, IMU imu) {
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
//        // 預設先給 Pos 的值，反正 update 會覆蓋
//        controller = new PIDFController(kP, kI, kD, kStatic_Pos);
//
//        tryAutoCalibrateOffset();
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
//            System.out.println("✅ Turret Auto-Calibrated! New Offset: " + this.activePhaseOffset);
//            return true;
//        } else {
//            System.out.println("⚠️ Turret not in overlap zone. Using default offset: " + LR_PHASE_OFFSET);
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
//        return (filteredAngle * GEAR_RATIO) - ANGLE_OFFSET;
//    }
//
//    public void update() {
//        Robot.getInstance().profiler.start("Turret Loop");
//        double currentTime = timer.seconds();
//        if (lastTime == 0) lastTime = currentTime;
//        double dt = currentTime - lastTime;
//
//        // [新增] 計算 Loop Time (轉成毫秒方便閱讀)
//        debug_looptime = dt * 1000.0;
//
//        if (dt < 0.002) return;
//
//        // 1. 底盤運動補償計算 (保持不變)
//        double chassisAngularVel = imu.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
//        double rawAccel = (chassisAngularVel - lastChassisVel) / dt;
//        currentChassisAccel = (accelFilterAlpha * rawAccel) + ((1.0 - accelFilterAlpha) * currentChassisAccel);
//
//        // 2. 讀取當前角度
//        debug_currentAngle = getAngle();
//
//        // 3. 計算炮塔速度與濾波 (保持不變)
//        double deltaAngle = debug_currentAngle - lastTurretAngleForVel;
//        double rawTurretVel = deltaAngle / dt;
//        currentTurretVelocity = (velFilterAlpha * rawTurretVel) + ((1.0 - velFilterAlpha) * currentTurretVelocity);
//        lastTurretAngleForVel = debug_currentAngle;
//
//        // 步驟 A: 先計算誤差，決定我們「想往哪邊轉」
//        double error = targetAngle - debug_currentAngle;
//
//        // 步驟 B: 根據誤差方向，選擇對應的 kStatic
//        double baseKStatic;
//        if (error > 0) {
//            baseKStatic = kStatic_Pos; // 往正向轉 (例如往左)
//        } else {
//            baseKStatic = kStatic_Neg; // 往反向轉 (例如往右，線可能比較緊)
//        }
//
//        // 步驟 C: 動態調整 (如果正在高速移動，降低靜摩擦力補償)
//        if (Math.abs(currentTurretVelocity) > 10.0) {
//            // 當動起來時，摩擦力變小，切換成一個較小的動摩擦值 (例如 0.04)
//            // 你也可以設定為 baseKStatic * 0.6 之類的比例
//            baseKStatic = 0.04;
//        }
//        debug_kStatic = baseKStatic;
//        // 4. 將計算好的 baseKStatic 傳入 PID 控制器
//        if (Constant.OpModeType.AUTO == Constant.OP_MODE_TYPE) {
//            // 自動模式：使用 Auto 參數
//            controller.setPIDF(kP_Auto, kI_Auto, kD_Auto, baseKStatic);
//        } else {
//            // 手動模式 / Tuning：使用一般參數
//            controller.setPIDF(kP, kI, kD, baseKStatic);
//        }
//        controller.setIZone(Izone);
//        controller.setPoint(targetAngle);
//        controller.setIntegrationControl(new PIDFController.IntegrationControl(
//                PIDFController.IntegrationBehavior.CLEAR_AT_SP,
//                0.2,
//                -1.0,  // 放大到 1.0 (代表允許積分累積到滿功率)
//                1.0
//        ));
//
//        controller.setTolerance(1.5);
//
//        // 5. 計算 PID 輸出
//        double pidOutput = controller.update(debug_currentAngle);
//
//        // 6. 電壓補償
//        double compensationFactor = voltageMonitor.getCompensationFactor();
//        double totalPower = pidOutput * compensationFactor;
//
//        // 7. 軟體限位檢查
//        if ((debug_currentAngle > MAX_ANGLE && totalPower > 0) ||
//                (debug_currentAngle < MIN_ANGLE && totalPower < 0)) {
//            totalPower = 0;
//        }
//        // 2. 如果在容許誤差範圍內 (TOLERANCE)
//        if (Math.abs(error) < TOLERANCE) {
//            // A. 徹底停止動力，讓馬達放鬆
//            totalPower = 0;
//
//            // B. 重設 PID 內部狀態，防止切換目標時的爆發
//            controller.reset();
//        }
//
//        // 8. 最終輸出
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
//    // --- 對外介面 ---
//    public void setTargetAngle(double angle) {
//        this.targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
//    }
//    // ... 其他 getter/setter 保持不變 ...
//    public void adjustTargetAngle(double delta) { setTargetAngle(targetAngle + delta); }
//    public boolean atTarget(double tolerance) { return abs(debug_currentAngle - targetAngle) < tolerance; }
//    public double getHoodPosition() { return servoHoodL.getPosition(); }
//    public void setHoodPosition(double pos) { pos = Range.clip(pos, 0, 1); servoHoodL.setPosition(pos); servoHoodR.setPosition(pos); }
//    public CRServo getServoR() { return servoDriveR; }
//    public CRServo getServoL() { return servoDriveL; }
//    public Double getError() { return targetAngle - getAngle(); }
//    public Double gettargetAngle() { return targetAngle; }
//    public double[] getDualSensorData() {
//        double degL = (Range.clip(feedbackSensorL.getVoltage(), 0.01, REAL_MAX_V) / REAL_MAX_V) * 360.0;
//        double degR = (Range.clip(feedbackSensorR.getVoltage(), 0.01, REAL_MAX_V) / REAL_MAX_V) * 360.0;
//
//        return new double[]{
//                degL,
//                degR,
//                activePhaseOffset,
//                lastServoAngle,
//                debug_currentAngle
//        };
//    }
//
//}