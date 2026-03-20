package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSystem.HOOD_MAX;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSystem.HOOD_MIN;
import static org.firstinspires.ftc.teamcode.constant.Constant.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.constant.Constant.ANGLE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.DISTANCE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.END_POSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.commands.FullAimCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.SmartGateCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpInputHandler;
import org.firstinspires.ftc.teamcode.util.Drawing;

@Config
@TeleOp(name = "ShooterTuner1")
public class ShooterTunerTest extends CommandOpMode {

    // ==================== 硬體與輸入 ====================
    private Robot robot;
    private GamepadEx driverOp;
    private TeleOpInputHandler inputHandler;
    public static double TARGET_RPM = 0;

    @Override
    public void initialize() {
        // Set the op mode type
        Constant.OP_MODE_TYPE = Constant.OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot
        robot = Robot.getInstance();
        robot.init(hardwareMap);

        // End Pose
        robot.imu.setPose(new Pose2d(END_POSE.getX(DISTANCE_UNIT), END_POSE.getY(DISTANCE_UNIT), END_POSE.getHeading(ANGLE_UNIT)));
        robot.drive.setTargetHeading(END_POSE.getHeading(ANGLE_UNIT));

        //Initialize Gamepad
        driverOp = new GamepadEx(gamepad1);

        // Initialize Input Handler
        inputHandler = new TeleOpInputHandler();


        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Alliance Color: ", ALLIANCE_COLOR);
    }

    public static double  kComp = 0;
    @Override
    public void run() {
        robot.profiler.start("Full Loop");
        super.run();
        robot.shooterSystem.setCalculatedBaseRpm(TARGET_RPM);
        double error = TARGET_RPM - robot.shooter.getCurrentRPM();
        double compHood = Range.clip(1 - (kComp * error), HOOD_MIN, HOOD_MAX);
        robot.turret.setHoodPosition(compHood);

        telemetry.addData("rpm", robot.shooter.getCurrentRPM());
        telemetry.addData("target", TARGET_RPM);
        telemetry.addData("error", error);
        telemetry.update();

        robot.updateTeleOp();
        driverOp.readButtons();

        updateVisualization();
        robot.profiler.end("Full Loop");
    }


    private void updateVisualization() {
        Drawing.sendPacket();
    }

    @Override
    public void end() {
        robot.exportProfiler(robot.file);
    }
}