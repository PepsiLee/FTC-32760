package org.firstinspires.ftc.teamcode.constant;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveConfig {
    // === 左前輪 (LF) ===
    public static double LF_P = 0.007;
    public static double LF_I = 0;
    public static double LF_D = 0.00025;
    public static double LF_kStatic = 0.07;
    // === 右前輪 (RF) ===
    public static double RF_P = 0.007;
    public static double RF_I = 0;
    public static double RF_D = 0.00025;
    public static double RF_kStatic = 0.07;
    // === 左後輪 (LB) ===
    public static double LB_P = 0.007;
    public static double LB_I = 0;
    public static double LB_D = 0.00025;
    public static double LB_kStatic = 0.07;
    // === 右後輪 (RB) ===
    public static double RB_P = 0.0065;
    public static double RB_I = 0;
    public static double RB_D = 0.00025;
    public static double RB_kStatic = 0.07;

    public static double[] LUT_LF = {0.100, 0.106, 0.100, 0.087, 0.096, 0.079, 0.085, 0.095, 0.108, 0.110, 0.106, 0.096, 0.086, 0.084, 0.090, 0.086, 0.087, 0.098, 0.091, 0.092, 0.080, 0.083, 0.076, 0.077, 0.080, 0.085, 0.086, 0.092, 0.088, 0.086, 0.085, 0.083, 0.082, 0.086, 0.090, 0.101};
    public static double[] LUT_RF = {0.083, 0.084, 0.084, 0.083, 0.085, 0.087, 0.086, 0.086, 0.083, 0.084, 0.083, 0.086, 0.089, 0.092, 0.091, 0.098, 0.110, 0.096, 0.085, 0.089, 0.098, 0.106, 0.109, 0.114, 0.096, 0.109, 0.087, 0.083, 0.091, 0.095, 0.097, 0.089, 0.098, 0.079, 0.085, 0.083};
    public static double[] LUT_LB = {0.084, 0.085, 0.079, 0.091, 0.103, 0.111, 0.095, 0.096, 0.072, 0.089, 0.086, 0.090, 0.103, 0.111, 0.096, 0.112, 0.103, 0.096, 0.082, 0.087, 0.090, 0.087, 0.088, 0.093, 0.076, 0.084, 0.082, 0.085, 0.082, 0.079, 0.082, 0.083, 0.090, 0.088, 0.085, 0.084};
    public static double[] LUT_RB = {0.081, 0.080, 0.087, 0.085, 0.099, 0.091, 0.084, 0.088, 0.082, 0.080, 0.085, 0.093, 0.100, 0.096, 0.085, 0.081, 0.078, 0.083, 0.079, 0.082, 0.082, 0.086, 0.084, 0.082, 0.082, 0.079, 0.081, 0.079, 0.081, 0.081, 0.084, 0.079, 0.084, 0.074, 0.082, 0.079};
}
