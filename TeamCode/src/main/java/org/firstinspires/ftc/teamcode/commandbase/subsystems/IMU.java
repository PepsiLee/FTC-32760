package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class IMU extends SubsystemBase {
    public static double PINPOINT_TELEOP_POLLING_RATE = 50;
    private final GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer;


    public IMU(HardwareMap hardwareMap) {
        // 初始化 Pinpoint 硬件
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0.01, 6.2, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        timer = new ElapsedTime();

        // 重置角度
//        resetHeading();
    }

    /**
     * 获取当前航向角 (Degrees)
     */
    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /**
     * 获取当前角度 (Radians)
     */
    public double getHeadingDegree() {
        return pinpoint.getHeading(AngleUnit.DEGREES); // 或者使用 AngleUnit 转换
    }

    /**
     * 获取角速度 (Degrees/Second)
     */
    public double getYawVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    /**
     * 重置 IMU 角度为 0
     */
    public void resetHeading() {
        pinpoint.resetPosAndIMU();
    }

    public void resetIMU() {
        pinpoint.resetPosAndIMU();
    }

    public Pose2d getPose() {
        return new Pose2d(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), new Rotation2d(pinpoint.getHeading(AngleUnit.RADIANS)));
    }

    public void setPose(Pose2d pose) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
    }

    public Vector2d getVelocity() {
        return new Vector2d(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH));
    }

    public double getHeadingVelocity(UnnormalizedAngleUnit unit) {
        return pinpoint.getHeadingVelocity(unit);
    }

    @Override
    public void periodic() {
        update();
    }

    public void update() {
        if (timer.milliseconds() > (1000 / PINPOINT_TELEOP_POLLING_RATE)) {
            pinpoint.update();
            timer.reset();
        }
    }
}
