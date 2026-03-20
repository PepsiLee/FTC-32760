package org.firstinspires.ftc.teamcode.opmodes.debugging;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Test: Wiring & Zero Point", group = "Test")
public class SwerveWiringTest extends LinearOpMode {

    private DcMotorEx[] driveMotors = new DcMotorEx[4];
    private Servo[] turnServos = new Servo[4];
    private AbsoluteAnalogEncoder[] encoders = new AbsoluteAnalogEncoder[4];


    private String[] moduleDisplayNames = {"左前 (FL)", "右前 (FR)", "左後 (RL)", "右後 (RR)"};
    private int currentSelection = 0;

    // 按鈕狀態記錄
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastButtonA = false;
    private boolean lastButtonB = false;
    private boolean lastButtonY = false;

    // ★ PID 參數 (可在 Dashboard 即時調整)
    // P: 比例，主要出力。因為輸入是角度(誤差可達180)，所以 P 要很小 (0.01~0.03)
    // I: 積分，用來消除最後那一丁點的誤差
    // D: 微分，用來煞車，防止衝過頭 (震盪)
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.001;

    private PIDFController turnPID;

    @Override
    public void runOpMode() throws InterruptedException {
        String[] driveNames = {"fl", "fr", "rl", "rr"};
        String[] servoNames = {"fls", "frs", "rls", "rrs"};
        String[] encNames =   {"fle", "fre", "rle", "rre"};

        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, driveNames[i]);
            turnServos[i] = hardwareMap.get(Servo.class, servoNames[i]);
            AnalogInput rawInput = hardwareMap.get(AnalogInput.class, encNames[i]);
            encoders[i] = new AbsoluteAnalogEncoder(hardwareMap, encNames[i], 3.205, AngleUnit.DEGREES).setReversed(true);
            driveMotors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        // 初始化 PID 控制器
        turnPID = new PIDFController(new PIDCoefficients(kP, kI, kD));

        telemetry.addLine("初始化完成！");
        telemetry.addLine("操作：");
        telemetry.addLine("  D-Pad 上/下：切換輪子");
        telemetry.addLine("  右搖桿 X：手動轉動");
        telemetry.addLine("★ 按住 Y 鍵：PID 自動歸零");
        telemetry.addLine("  按 A 鍵：設定 Offset (歸零)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // === 1. 切換輪子 ===
            if (gamepad1.dpad_up && !lastDpadUp) currentSelection = (currentSelection + 1) % 4;
            if (gamepad1.dpad_down && !lastDpadDown) currentSelection = (currentSelection - 1 + 4) % 4;
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            AbsoluteAnalogEncoder currentEnc = encoders[currentSelection];
            DcMotorEx currentMotor = driveMotors[currentSelection];

            // === 2. 歸零設定 (Offset) ===
            if (gamepad1.a && !lastButtonA) currentEnc.getCurrentPosition();
            if (gamepad1.b && !lastButtonB) currentEnc.zero(0);

            lastButtonA = gamepad1.a;
            lastButtonB = gamepad1.b;

            // === 3. 控制邏輯 ===
            double drivePower = -gamepad1.left_stick_y * 1;
            double servoPower = 0;

            if (gamepad1.y) {
                // ★ PID 自動歸零模式 ★

                // 如果是剛按下 Y 鍵，重置 PID 內部狀態 (清除積分累積)
                if (!lastButtonY) {
                    turnPID.reset();
                    // 確保使用最新的參數 (如果 Dashboard 有改)
                    turnPID = new PIDFController(new PIDCoefficients(kP, kI, kD));
                }

                // 1. 取得絕對角度
                double currentAngle = currentEnc.getCurrentPosition();

                // 2. 計算誤差 (目標 0 - 當前角度)
                double error = 0 - currentAngle;

                // 3. 最短路徑處理 (Wrap around)
                while (error > 180) error -= 360;
                while (error < -180) error += 360;

                // 4. 設定 PID 目標
                // 技巧：告訴 PID 目標是 error，當前位置是 0
                // 這樣 PID 只需要努力把 0 變成 error 即可
                turnPID.setTargetPosition(error);
                servoPower = turnPID.update(0);

                // 5. 限制最大出力
                servoPower = Range.clip(servoPower, -0.6, 0.6);

                telemetry.addData("模式", "PID 歸零中");
                telemetry.addData("誤差", "%.2f", error);
                telemetry.addData("PID Output", "%.2f", servoPower);
            } else {
                // 手動模式
                servoPower = gamepad1.right_stick_x * 0.4;
                telemetry.addData("模式", "手動控制");
            }
            lastButtonY = gamepad1.y;

            // 輸出動力
            for (int i = 0; i < 4; i++) {
                if (i == currentSelection) {
                    turnServos[i].setPosition(servoPower);
                    driveMotors[i].setPower(drivePower);
                } else {
                    turnServos[i].setPosition(0);
                    driveMotors[i].setPower(0);
                }
            }
            int position = currentMotor.getCurrentPosition();
            // === 4. 顯示數據 ===
            telemetry.addData(">> 編輯模組", moduleDisplayNames[currentSelection]);
            telemetry.addData("Raw Angle", "%.2f", currentEnc.getCurrentPosition());
            telemetry.addData("校正後角度", "%.2f", currentEnc.getCurrentPosition());

            telemetry.update();
        }
    }
}