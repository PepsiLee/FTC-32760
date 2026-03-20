package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;

public class FireSequenceCommand extends SequentialCommandGroup {

    public FireSequenceCommand(Robot robot, boolean precise) {
        addRequirements(robot.shooter, robot.intake);

        addCommands(
                // 1. 如果是精確模式，先等到飛輪轉速達標 (isReady)
                new ConditionalCommand(
                        new WaitUntilCommand(() -> robot.shooterSystem.isReady()),
                        new InstantCommand(), // 非精確模式則直接跳過等待
                        () -> precise
                ),

                // 2. 智慧開門：只有狀態改變才等 100ms
                new SmartGateCommand(robot.shooter, true),

                // 3. 開始進彈
                new RunCommand(
                        () -> robot.intake.setState(Intake.IntakeState.INTAKE),
                        robot.intake
                )
        );
    }
}