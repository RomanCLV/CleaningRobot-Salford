package main;

public class PositionSample {
    private final double x;
    private final double y;
    private final double theta;
    private final long time;

    public PositionSample(double x, double y, double theta, long time) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.time = time;
    }

    // Getters
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public long getTime() {
        return time;
    }
}
