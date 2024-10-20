package main;

import javafx.fxml.FXML;
import javafx.scene.chart.LineChart;
import java.util.ArrayList;
import java.util.List;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.scene.chart.XYChart;
import javafx.util.Duration;

import utils.Utils;


public class PositionChartsController extends BaseController {

    //region Variables declaration

    //region GUI variables declaration
    @FXML
    private LineChart<Double, Double> chartX;
    @FXML
    private LineChart<Double, Double> chartY;
    @FXML
    private LineChart<Double, Double> chartTheta;
    @FXML
    private LineChart<Double, Double> chartXY;
    //endregion

    private final ArrayList<PositionSeries> positionSeriesList = new ArrayList<>();
    private int updateDelayMS = 1000; // 1 second
    private int displayWindowMS = 10000; // 10 * 1000 = 10 seconds

    private boolean isRunning = false;
    private Timeline updateTimeline;

    private final ArrayList<Integer> lastIndexsAdded = new ArrayList<>();
    //endregion

    @FXML
    private void initialize()
    {
        chartX.getXAxis().setAutoRanging(true);
        chartX.getYAxis().setAutoRanging(true);

        chartY.getXAxis().setAutoRanging(true);
        chartY.getYAxis().setAutoRanging(true);

        chartTheta.getXAxis().setAutoRanging(true);
        chartTheta.getYAxis().setAutoRanging(false);

        chartXY.getXAxis().setAutoRanging(true);
        chartXY.getYAxis().setAutoRanging(true);
    }

    public void setUpdateDelayMS(int delayMs)
    {
        updateDelayMS = delayMs;
    }

    public boolean setDisplayWindowSeconds(int seconds)
    {
        boolean set = false;
        if (!isRunning)
        {
            displayWindowMS = seconds * 1000;
            set = true;
        }
        return set;
    }

    public boolean setPositionSeriesList(PositionSeries... positionSeriesList)
    {
        boolean set = false;
        if (!isRunning)
        {
            this.positionSeriesList.clear();
            chartX.getData().clear();
            chartY.getData().clear();
            chartTheta.getData().clear();
            chartXY.getData().clear();
            lastIndexsAdded.clear();

            for (PositionSeries series : positionSeriesList)
            {
                this.positionSeriesList.add(series);

                XYChart.Series<Double, Double> xS = new XYChart.Series<>();
                XYChart.Series<Double, Double> yS = new XYChart.Series<>();
                XYChart.Series<Double, Double> tS = new XYChart.Series<>();
                XYChart.Series<Double, Double> xyS = new XYChart.Series<>();

                chartX.getData().add(xS);
                chartY.getData().add(yS);
                chartTheta.getData().add(tS);
                chartXY.getData().add(xyS);

                lastIndexsAdded.add(-1);

                String colorStyle = "-fx-stroke: " + Utils.toHexString(series.getColor()) + ";";
                xS.getNode().setStyle(colorStyle); // pb ici NullPointer
                yS.getNode().setStyle(colorStyle);
                tS.getNode().setStyle(colorStyle);
                xyS.getNode().setStyle(colorStyle);

                xS.setName(series.getName() + " - X over time");
                yS.setName(series.getName() + " - Y over time");
                tS.setName(series.getName() + " - θ over time");
                xyS.setName(series.getName() + " - XY");

            }

            set = true;
        }
        return set;
    }

    public void start()
    {
        if (!isRunning)
        {
            isRunning = true;
            updateTimeline = new Timeline(new KeyFrame(Duration.millis(updateDelayMS), event -> updateCharts()));
            updateTimeline.setCycleCount(Timeline.INDEFINITE);
            updateTimeline.play();
        }
    }

    public void stop()
    {
        if (isRunning)
        {
            isRunning = false;
            if (updateTimeline != null) {
                updateTimeline.stop();
                updateTimeline = null;
            }
        }
    }

    private void updateCharts()
    {
        long currentTime = System.currentTimeMillis();
        int totalSamples;
        List<PositionSample> samples;
        XYChart.Series<Double, Double> xSeries;
        XYChart.Series<Double, Double> ySeries;
        XYChart.Series<Double, Double> thetaSeries;
        XYChart.Series<Double, Double> xySeries;

        double startTime = currentTime - displayWindowMS; // 10 seconds before now
        int lastIndexAdded;
        PositionSample sample;
        double sampleTimeSec;

        for (int i = 0; i < positionSeriesList.size(); i++) {
            samples = positionSeriesList.get(i).getSamples();

            xSeries = chartX.getData().get(i);
            ySeries = chartY.getData().get(i);
            thetaSeries = chartTheta.getData().get(i);
            xySeries = chartXY.getData().get(i);

            // remove old values
//            while (!xSeries.getData().isEmpty() && xSeries.getData().get(0).getXValue() < startTime)
//            {
//                xSeries.getData().remove(0);
//                ySeries.getData().remove(0);
//                thetaSeries.getData().remove(0);
//            }

            totalSamples = samples.size();

            lastIndexAdded = lastIndexsAdded.get(i);
            while (lastIndexAdded + 1 < totalSamples)
            {
                lastIndexAdded++;
                sample = samples.get(lastIndexAdded);
                sampleTimeSec = sample.getTime() / 1000.;
                xSeries.getData().add(new XYChart.Data<>(sampleTimeSec, sample.getX()));
                ySeries.getData().add(new XYChart.Data<>(sampleTimeSec, sample.getY()));
                thetaSeries.getData().add(new XYChart.Data<>(sampleTimeSec, sample.getTheta()));
                xySeries.getData().add(new XYChart.Data<>(sample.getX(), sample.getY()));
            }
            lastIndexsAdded.set(i, lastIndexAdded);
        }
    }
}
