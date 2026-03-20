package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

public class InterpolationLUT {
    // 使用 TreeMap 因為它會自動根據 Key (距離) 排序
    private final TreeMap<Double, Double> table = new TreeMap<>();

    private String name;

    public InterpolationLUT(String name) {
        this.name = name;
    }

    // 加入數據點 (例如: 距離 100cm -> 轉速 1500rpm)
    public void add(double x, double y) {
        table.put(x, y);
    }

    // 核心邏輯：輸入 x (距離)，輸出 y (轉速)
    public double get(double x) {
        if (table.isEmpty()) return 0;

        // 1. 處理邊界情況：比最小距離還近，或比最遠距離還遠
        Map.Entry<Double, Double> floor = table.floorEntry(x);   // 找比 x 小的點
        Map.Entry<Double, Double> ceiling = table.ceilingEntry(x); // 找比 x 大的點

        if (floor == null) return ceiling.getValue(); // 比最小值還小，回傳最小值
        if (ceiling == null) return floor.getValue(); // 比最大值還大，回傳最大值
        if (floor.getKey().equals(ceiling.getKey())) return floor.getValue(); // 剛好命中 Key

        // 2. 線性插值公式 (Linear Interpolation)
        // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double x1 = floor.getKey();
        double y1 = floor.getValue();
        double x2 = ceiling.getKey();
        double y2 = ceiling.getValue();

        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    public boolean isEmpty() {
        return table.isEmpty();
    }
}