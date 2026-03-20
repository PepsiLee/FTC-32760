package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MedianFilter {
    private final int size;
    private final List<Double> buffer = new ArrayList<>();

    public MedianFilter(int size) {
        this.size = size;
    }

    public double calculate(double input) {
        buffer.add(input);
        if (buffer.size() > size) {
            buffer.remove(0);
        }

        List<Double> sorted = new ArrayList<>(buffer);
        Collections.sort(sorted);

        if (sorted.isEmpty()) return 0.0;
        int middle = sorted.size() / 2;
        if (sorted.size() % 2 == 1) {
            return sorted.get(middle);
        } else {
            return (sorted.get(middle - 1) + sorted.get(middle)) / 2.0;
        }
    }
}