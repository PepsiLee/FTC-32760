package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IMU;
import org.firstinspires.ftc.teamcode.constant.Constant;

@Config
public class PoseFusion extends SubsystemBase {
    // Vision correction tuning
    public static double VISION_BLEND = 0.1; // 0 = no correction, 1 = snap to vision
    public static double VISION_MAX_DISTANCE_IN = 12.0; // reject large jumps
    public static double VISION_MIN_INTERVAL_MS = 50.0; // limit correction rate
    public static boolean WRITE_BACK_TO_IMU = true; // keep IMU pose corrected
    public static boolean USE_VISION_HEADING = true; // use MT1 heading if available
    public static double VISION_HEADING_BLEND = 0.05; // 0 = no heading correction
    public static double VISION_MAX_HEADING_DIFF_DEG = 25.0; // reject large heading jumps

    private final IMU imu;
    private final Vision vision;
    private final ElapsedTime visionTimer = new ElapsedTime();

    private Pose2d fusedPose;
    private boolean usingVision = false;

    public PoseFusion(IMU imu, Vision vision) {
        this.imu = imu;
        this.vision = vision;
        this.fusedPose = imu.getPose();
    }

    public void update() {
        Pose2d odomPose = imu.getPose();
        fusedPose = odomPose;
        usingVision = false;

        if (visionTimer.milliseconds() < VISION_MIN_INTERVAL_MS) {
            return;
        }
        visionTimer.reset();
        Pose2d visionPose = vision.getRobotPoseMT2WithHeading(imu.getHeadingDegree());

        if (visionPose == null) {
            return;
        }

        //Convert to Pedro
        if(Constant.OP_MODE_TYPE == Constant.OpModeType.AUTO){
            Pose converterPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, visionPose.getX(), visionPose.getY(), AngleUnit.RADIANS, visionPose.getHeading()), InvertedFTCCoordinates.INSTANCE);
            converterPose = converterPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            visionPose = new Pose2d(converterPose.getX(), converterPose.getY(), new Rotation2d(converterPose.getHeading()));
        }



        double dist = odomPose.getTranslation().getDistance(visionPose.getTranslation());
        if (dist > VISION_MAX_DISTANCE_IN) {
            return;
        }

        double newX = lerp(odomPose.getX(), visionPose.getX(), VISION_BLEND);
        double newY = lerp(odomPose.getY(), visionPose.getY(), VISION_BLEND);

        double newHeading = odomPose.getHeading();
        if (USE_VISION_HEADING) {
            double headingDiffDeg = Math.toDegrees(Math.abs(wrapRadians(visionPose.getHeading() - odomPose.getHeading())));
            if (headingDiffDeg <= VISION_MAX_HEADING_DIFF_DEG) {
                newHeading = lerpAngle(odomPose.getHeading(), visionPose.getHeading(), VISION_HEADING_BLEND);
            }
        }

        fusedPose = new Pose2d(newX, newY, new Rotation2d(newHeading));
        usingVision = true;

        if (WRITE_BACK_TO_IMU) {
            imu.setPose(fusedPose);
        }
    }

    public Pose2d getPose() {
        return fusedPose;
    }

    public boolean isUsingVision() {
        return usingVision;
    }

    public void reset(Pose2d pose) {
        fusedPose = pose;
        usingVision = false;
        visionTimer.reset();
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double lerpAngle(double a, double b, double t) {
        double diff = wrapRadians(b - a);
        return wrapRadians(a + diff * t);
    }

    private static double wrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
