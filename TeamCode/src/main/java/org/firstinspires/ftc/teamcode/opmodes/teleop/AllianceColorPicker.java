package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constant.Constant;

@TeleOp
public class AllianceColorPicker extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Cross / Triangle", "Blue");
        telemetry.addData("Circle / Square", "Red");
        telemetry.addData("Alliance Color", Constant.ALLIANCE_COLOR);

        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.cross || gamepad2.cross || gamepad1.triangle || gamepad2.triangle) {
                Constant.ALLIANCE_COLOR = Constant.AllianceColor.BLUE;
            } else if (gamepad1.circle || gamepad2.circle || gamepad1.square || gamepad2.square) {
                Constant.ALLIANCE_COLOR = Constant.AllianceColor.RED;
            }

            telemetry.addData("Cross / Triangle", "Blue");
            telemetry.addData("Circle / Square", "Red");
            telemetry.addData("Alliance Color", Constant.ALLIANCE_COLOR);

            telemetry.update();
        }
    }
}