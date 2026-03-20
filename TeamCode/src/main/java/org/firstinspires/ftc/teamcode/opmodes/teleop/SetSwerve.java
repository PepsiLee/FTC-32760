package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class SetSwerve extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = Robot.getInstance();
        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            robot.drive.lf.setTarget(0,0);
            robot.drive.lb.setTarget(0,0);
            robot.drive.rf.setTarget(0,0);
            robot.drive.rb.setTarget(0,0);

            robot.drive.lf.apply();
            robot.drive.lb.apply();
            robot.drive.rf.apply();
            robot.drive.rb.apply();

            robot.turret.setTargetAngle(0);
            robot.shooter.setGateOpen(true);
        }
    }
}
