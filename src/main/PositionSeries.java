package main;

import javafx.scene.paint.Color;
import java.util.ArrayList;
import java.util.List;

public class PositionSeries {
    private String name;
    private Color color;
    private List<PositionSample> samples;

    public PositionSeries(String name, Color color) {
        this.name = name;
        this.color = color;
        this.samples = new ArrayList<>();
    }

    // Ajouter un échantillon
    public void addSample(PositionSample sample) {
        this.samples.add(sample);
    }

    // Vider la liste d'échantillons
    public void clearSamples() {
        this.samples.clear();
    }

    // Getters
    public String getName() {
        return name;
    }

    public Color getColor() {
        return color;
    }

    public List<PositionSample> getSamples() {
        return samples;
    }
}
