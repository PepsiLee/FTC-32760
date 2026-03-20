package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.shooter.Shooter;

// 1. 改繼承 SequentialCommandGroup，確保動作一定會發生
public class SmartGateCommand extends SequentialCommandGroup {

    public SmartGateCommand(Shooter shooter, boolean targetState) {
        // 2. 直接加入指令序列，不再做條件判斷
        addCommands(
                // 不管現在狀態是什麼，強制再送一次訊號確保到位
                new InstantCommand(() -> shooter.setGateOpen(targetState)),

                // 強制等待 150ms，給伺服機物理轉動的時間
                // 這能防止機器人在門還沒關好前就開始移動
                new WaitCommand(100)
        );

        addRequirements(shooter);
    }
}