package org.firstinspires.ftc.teamcode.util.motion;

public class TrapezoidProfile {
    private final double maxVel;
    private final double maxAccel;

    // 狀態變數
    private double startPos, startVel;
    private double endPos; // 假設結束速度總是 0
    private double t1, t2, t3; // 三個階段的時間點
    private double distToAccel, distToDecel, cruiseDist;
    private double accelerationTime, decelerationTime, cruiseTime;
    private double peakVel; // 實際達到的最高速度 (可能小於 maxVel)

    public TrapezoidProfile(double maxVel, double maxAccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    /**
     * 設定目標，假設起始速度為 0 (相容舊代碼)
     */
    public void setTarget(double currentPos, double targetPos) {
        setTarget(currentPos, 0, targetPos, 0);
    }

    /**
     * [關鍵修正] 設定目標，包含當前速度與目標速度
     * 這是 ProfiledPIDController.setGoal 需要呼叫的方法
     */
    public void setTarget(double currentPos, double currentVel, double targetPos, double targetVel) {
        this.startPos = currentPos;
        this.startVel = currentVel;
        this.endPos = targetPos;

        // 1. 計算位移與方向
        double displacement = targetPos - currentPos;
        // 為了簡化計算，我們將所有數據轉換到 "正向" 座標系處理，最後再還原
        double direction = Math.signum(displacement);

        // 如果已經到位且速度極小，直接結束
        if (Math.abs(displacement) < 1e-6 && Math.abs(currentVel) < 1e-6) {
            t1 = t2 = t3 = 0;
            return;
        }

        double dist = Math.abs(displacement);
        double vi = currentVel * direction; // 轉換為相對速度
        double vf = targetVel * direction;  // 通常為 0

        // 2. 計算達到 maxVel 所需的距離
        // 公示: vf^2 = vi^2 + 2ad  =>  d = (vf^2 - vi^2) / 2a
        double distToReachMaxVel = (maxVel * maxVel - vi * vi) / (2 * maxAccel);
        double distFromMaxVelToEnd = (maxVel * maxVel - vf * vf) / (2 * maxAccel);

        // 3. 判斷是梯形 (Trapezoid) 還是三角形 (Triangle)
        if (distToReachMaxVel + distFromMaxVelToEnd <= dist) {
            // === 梯形模式 (有巡航階段) ===
            peakVel = maxVel;
            cruiseDist = dist - distToReachMaxVel - distFromMaxVelToEnd;
            cruiseTime = cruiseDist / maxVel;

            accelerationTime = (maxVel - vi) / maxAccel;
            decelerationTime = (maxVel - vf) / maxAccel;
        } else {
            // === 三角形模式 (沒有巡航，達不到 maxVel 就得減速) ===
            cruiseTime = 0;
            cruiseDist = 0;

            // 求解 peakVel (頂點速度)
            // dist = distAcc + distDec
            // dist = (vPeak^2 - vi^2)/2a + (vPeak^2 - vf^2)/2a
            // 2a * dist = 2*vPeak^2 - vi^2 - vf^2
            // vPeak = sqrt( (2a*dist + vi^2 + vf^2) / 2 )
            peakVel = Math.sqrt(Math.abs((2 * maxAccel * dist + vi * vi + vf * vf) / 2.0));

            accelerationTime = (peakVel - vi) / maxAccel;
            decelerationTime = (peakVel - vf) / maxAccel;
        }

        // 4. 設定時間戳記
        // 注意：accelerationTime 可能是負的 (如果初始速度太快，需要先減速)
        // 這裡為了簡單，假設 Profile 是連續的，我們取絕對值或由公式自然處理
        // 但最穩健的寫法是處理 vi > maxVel 的情況，這裡簡化處理：
        if (accelerationTime < 0) accelerationTime = 0;

        t1 = accelerationTime;
        t2 = t1 + cruiseTime;
        t3 = t2 + decelerationTime;
    }

    public State calculate(double t) {
        double pos = 0;
        double vel = 0;
        double accel = 0;

        // 用來還原方向
        double direction = Math.signum(endPos - startPos);
        double vi = startVel * direction; // 相對起始速度

        if (t < t1) {
            // === 加速階段 ===
            accel = maxAccel;
            vel = vi + maxAccel * t;
            pos = vi * t + 0.5 * maxAccel * t * t;
        } else if (t < t2) {
            // === 巡航階段 ===
            accel = 0;
            vel = peakVel;
            double dt = t - t1;
            // 先算出第一階段結束的位置
            double pos1 = vi * t1 + 0.5 * maxAccel * t1 * t1;
            pos = pos1 + peakVel * dt;
        } else if (t <= t3) {
            // === 減速階段 ===
            accel = -maxAccel;
            vel = peakVel - maxAccel * (t - t2);
            double dt = t - t2;

            // 第一二階段總位移
            double pos1 = vi * t1 + 0.5 * maxAccel * t1 * t1;
            double pos2 = peakVel * cruiseTime;

            pos = pos1 + pos2 + peakVel * dt - 0.5 * maxAccel * dt * dt;
        } else {
            // === 結束 ===
            accel = 0;
            vel = 0; // 假設終點速度為 0
            pos = Math.abs(endPos - startPos);
        }

        // 還原到絕對座標系
        return new State(
                startPos + (pos * direction),
                vel * direction,
                accel * direction
        );
    }

    public double getTotalTime() {
        return t3;
    }

    public static class State {
        public double position;
        public double velocity;
        public double acceleration;

        public State(double p, double v, double a) {
            this.position = p;
            this.velocity = v;
            this.acceleration = a;
        }
    }
}