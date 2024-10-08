package main;

public interface Behavior
{
    public int getPriority();
    public boolean getInhibitor();
    public boolean getSuppressor();
    public double[] getNormSensorVec(double sensor[]);

    public void run();
    public int fsm(Entry inputEntry[], Entry outputEntry[], boolean reset);
}
