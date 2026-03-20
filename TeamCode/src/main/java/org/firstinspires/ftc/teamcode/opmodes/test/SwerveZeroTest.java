package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.constant.Constant.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.constant.Constant.ANGLE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.DISTANCE_UNIT;
import static org.firstinspires.ftc.teamcode.constant.Constant.END_POSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constant.Constant;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpInputHandler;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name = "Swerve Zero Test")
public class SwerveZeroTest extends CommandOpMode {
    // ==================== 硬體與輸入 ====================
    private Robot robot;
    private GamepadEx driverOp;
    private TeleOpInputHandler inputHandler;

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

        initializeCommands();
        setupDriverControls();
        setupToolControls();

        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Alliance Color: ", ALLIANCE_COLOR);
    }

    private void initializeCommands() {

    }

    private void setupDriverControls() {
        // ==== Default Command ====
        // Drive Control
        robot.drive.setDefaultCommand(new RunCommand(() -> {
            TeleOpInputHandler.DriveInputs inputs = inputHandler.getDriveInputs(driverOp);
            robot.drive.arcadeDrive(inputs.forward, inputs.strafe, inputs.rotation);
        }, robot.drive));

        // Back: Reset Pose
        new GamepadButton(driverOp, GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    robot.imu.resetIMU();
                }));

    }

    private void setupToolControls() {
    }


    @Override
    public void run() {
        robot.profiler.start("Full Loop");
        super.run();

        robot.drive.rb.setServoBias(Constant.RB_BIAS);
        robot.drive.rf.setServoBias(Constant.RF_BIAS);
        robot.drive.lb.setServoBias(Constant.LB_BIAS);
        robot.drive.lf.setServoBias(Constant.LF_BIAS);

        robot.drive.rb.setTarget(0,0);
        robot.drive.rf.setTarget(0,0);
        robot.drive.lb.setTarget(0,0);
        robot.drive.lf.setTarget(0,0);

        robot.updateTeleOp();
        driverOp.readButtons();

        updateTelemetry();
        updateVisualization();
        robot.profiler.end("Full Loop");
    }

    private void updateTelemetry() {


        TelemetryPacket packet = getSwerveModuleStatus();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


        telemetry.update();
    }

    @NonNull
    private TelemetryPacket getSwerveModuleStatus() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Swerve/LF/Target", robot.drive.lf.getTargetAngle());
        packet.put("Swerve/LF/Current", robot.drive.lf.getCurrentAngle());
        packet.put("Swerve/RF/Target", robot.drive.rf.getTargetAngle());
        packet.put("Swerve/RF/Current", robot.drive.rf.getCurrentAngle());
        packet.put("Swerve/LB/Target", robot.drive.lb.getTargetAngle());
        packet.put("Swerve/LB/Current", robot.drive.lb.getCurrentAngle());
        packet.put("Swerve/RB/Target", robot.drive.rb.getTargetAngle());
        packet.put("Swerve/RB/Current", robot.drive.rb.getCurrentAngle());
        return packet;
    }

    private void updateVisualization() {
        Drawing.sendPacket();
    }

    @Override
    public void end() {
        robot.exportProfiler(robot.file);
    }
}