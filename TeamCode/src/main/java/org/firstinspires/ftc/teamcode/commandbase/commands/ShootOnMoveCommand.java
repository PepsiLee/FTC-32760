package org.firstinspires.ftc.teamcode.commandbase.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSystem;

import java.util.function.Supplier;

public class ShootOnMoveCommand extends CommandBase {
    private final ShooterSystem shooterSystem;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Vector2d> velocitySupplier;
    private final Translation2d targetLocation;

    /**
     * @param system 射擊系統
     * @param pose   提供機器人目前的座標 (通常來自 OdometrySubsystem::getPose)
     * @param vel    提供機器人目前的速度 (通常來自 DriveSubsystem::getVelocity)
     * @param target 目標的場地座標
     */
    public ShootOnMoveCommand(ShooterSystem system, Supplier<Pose2d> pose, Supplier<Vector2d> vel, Translation2d target) {
        this.shooterSystem = system;
        this.poseSupplier = pose;
        this.velocitySupplier = vel;
        this.targetLocation = target;

        addRequirements(); // 注意：這裡是 System Wrapper，不需要 require 內部的 Turret/Shooter
    }

    @Override
    public void execute() {
        // 每一次循環都重新計算向量與補償
        shooterSystem.aimAtRunning(
                poseSupplier.get(),
                velocitySupplier.get(),
                targetLocation
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooterSystem.stopAll();
    }
}