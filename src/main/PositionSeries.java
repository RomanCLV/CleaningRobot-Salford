package main;

import javafx.scene.paint.Color;
import java.util.ArrayList;
import java.util.List;

public class PositionSeries {
    private final String name;
    private final Color color;
    private final List<PositionSample> samples;

    public PositionSeries(String name, Color color) {
        this.name = name;
        this.color = color;
        this.samples = new ArrayList<>();
    }

    public String getName() {
        return name;
    }

    public Color getColor() {
        return color;
    }

    public List<PositionSample> getSamples() {
        return samples;
    }

    public void addSample(PositionSample sample) {
        this.samples.add(sample);
    }

    public void clearSamples() {
        this.samples.clear();
    }
}
