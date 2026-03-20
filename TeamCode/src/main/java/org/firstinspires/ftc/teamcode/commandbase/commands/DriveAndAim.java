package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.constant.Constant.GOAL_POSE;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;

public class DriveAndAim extends ParallelDeadlineGroup {

    public DriveAndAim(Robot robot, PathChain path) {
        super(
                new FollowPathCommand(robot.follower, path),
                new RunCommand(() -> {
                    // 使用手動( TeleOp )相同的 Pose 來源：IMU + Vision 融合
                    Pose2d robotPose = Robot.getInstance().getPose();

                    robot.shooterSystem.aimAtRunning(robotPose, new Vector2d(), GOAL_POSE());
                }));
        addRequirements(robot.shooterSystem);
    }
}
