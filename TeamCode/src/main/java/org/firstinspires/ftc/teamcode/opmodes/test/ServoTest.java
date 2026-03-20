package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    public static double servoPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        ServoEx servo = new ServoEx(hardwareMap, "Gate");
        servo.setPwm(new PwmControl.PwmRange(500,2500));
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a) servo.set(servoPos);
            else {
                servo.disable();
            }

        }
    }
}
