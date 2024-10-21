package main;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.chart.*;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
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
    private ScatterChart<Double, Double> chartXY;
    @FXML
    private NumberAxis xAxisX;
    @FXML
    private NumberAxis xAxisY;
    @FXML
    private NumberAxis yAxisX;
    @FXML
    private NumberAxis yAxisY;
    @FXML
    private NumberAxis thetaAxisX;
    @FXML
    private NumberAxis thetaAxisY;
    @FXML
    private NumberAxis xyAxisX;
    @FXML
    private NumberAxis xyAxisY;

    //endregion

    private final ArrayList<PositionSeries> positionSeriesList = new ArrayList<>();
    private int updateDelayMS = 1000; // 1 second
    private int displayWindowMS = 10000; // 10 * 1000 = 10 seconds

    private boolean isRunning = false;
    private Timeline updateTimeline;

    private final ArrayList<Integer> lastIndexesAdded = new ArrayList<>();

    private double chartTimeMin = 0.;
    private double chartTimeMax = 10.;
    private final double chartTimeExtend = 5.;

    private final static double DEFAULT_CHART_X_MIN = -.2;
    private final static double DEFAULT_CHART_X_MAX =  .2;
    private final static double DEFAULT_CHART_Y_MIN = -.2;
    private final static double DEFAULT_CHART_Y_MAX =  .2;
    private final static double DEFAULT_CHART_THETA_MIN = -180.;
    private final static double DEFAULT_CHART_THETA_MAX =  180.;
    private final static double DEFAULT_CHART_XY_X_MIN = -0.5;
    private final static double DEFAULT_CHART_XY_X_MAX =  4.5;
    private final static double DEFAULT_CHART_XY_Y_MIN = -4.5;
    private final static double DEFAULT_CHART_XY_Y_MAX =  1.5;

    private double chartXMin = DEFAULT_CHART_X_MIN;
    private double chartXMax = DEFAULT_CHART_X_MAX;
    private double chartYMin = DEFAULT_CHART_Y_MIN;
    private double chartYMax = DEFAULT_CHART_Y_MAX;

    private long startSimulationTimestamp;
    //endregion

    @FXML
    private void initialize()
    {
        xAxisX.setAutoRanging(false);
        xAxisX.setLowerBound(chartTimeMin);
        xAxisX.setUpperBound(chartTimeMax);

        xAxisY.setAutoRanging(false);
        xAxisY.setLowerBound(chartXMin);
        xAxisY.setUpperBound(chartXMax);

        yAxisX.setAutoRanging(false);
        yAxisX.setLowerBound(chartTimeMin);
        yAxisX.setUpperBound(chartTimeMax);

        yAxisY.setAutoRanging(false);
        yAxisY.setLowerBound(chartXMin);
        yAxisY.setUpperBound(chartXMax);

        thetaAxisX.setAutoRanging(false);
        thetaAxisX.setLowerBound(chartTimeMin);
        thetaAxisX.setUpperBound(chartTimeMax);

        thetaAxisY.setAutoRanging(false);
        thetaAxisY.setLowerBound(DEFAULT_CHART_THETA_MIN);
        thetaAxisY.setUpperBound(DEFAULT_CHART_THETA_MAX);

        xyAxisX.setAutoRanging(false);
        xyAxisX.setLowerBound(DEFAULT_CHART_XY_X_MIN);
        xyAxisX.setUpperBound(DEFAULT_CHART_XY_X_MAX);

        xyAxisY.setAutoRanging(false);
        xyAxisY.setLowerBound(DEFAULT_CHART_XY_Y_MIN);
        xyAxisY.setUpperBound(DEFAULT_CHART_XY_Y_MAX);
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

    public void setStartSimulationTimestamp(long startTimestamp)
    {
        startSimulationTimestamp = startTimestamp;
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
            lastIndexesAdded.clear();

            for (PositionSeries series : positionSeriesList)
            {
                this.positionSeriesList.add(series);

                // LineChart
                XYChart.Series<Double, Double> xS = new XYChart.Series<>();
                XYChart.Series<Double, Double> yS = new XYChart.Series<>();
                XYChart.Series<Double, Double> tS = new XYChart.Series<>();

                // ScatterChart
                XYChart.Series<Double, Double> xyS = new XYChart.Series<>();

                chartX.getData().add(xS);
                chartY.getData().add(yS);
                chartTheta.getData().add(tS);
                chartXY.getData().add(xyS);

                lastIndexesAdded.add(-1);

                String colorHex = Utils.toHexString(series.getColor());
                String colorStyle = "-fx-stroke: " + colorHex + ";";

                xS.getNode().setStyle(colorStyle);
                yS.getNode().setStyle(colorStyle);
                tS.getNode().setStyle(colorStyle);

                //xyS.getNode().setStyle("-fx-background-color: " + colorHex + ";"); // pb ici

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
        List<PositionSample> samples;
        XYChart.Series<Double, Double> xSeries;
        XYChart.Series<Double, Double> ySeries;
        XYChart.Series<Double, Double> thetaSeries;
        XYChart.Series<Double, Double> xySeries;

        long currentTime = System.currentTimeMillis();
        double newChartTimeMax = Math.max(10., (currentTime - startSimulationTimestamp) / 1000.);
        if (newChartTimeMax > chartTimeMax)
        {
            chartTimeMax += chartTimeExtend;
        }
        chartTimeMin = Math.max(0., chartTimeMax - ((displayWindowMS / 1000.)));

        int lastIndexAdded;
        PositionSample sample;
        double sampleTimeSec;
        int totalSamples;

        double tmp;

        for (int i = 0; i < positionSeriesList.size(); i++) {
            samples = positionSeriesList.get(i).getSamples();

            xSeries = chartX.getData().get(i);
            ySeries = chartY.getData().get(i);
            thetaSeries = chartTheta.getData().get(i);
            xySeries = chartXY.getData().get(i);

            // remove old values
            while (!xSeries.getData().isEmpty() && xSeries.getData().get(0).getXValue() < chartTimeMin)
            {
                xSeries.getData().remove(0);
                ySeries.getData().remove(0);
                thetaSeries.getData().remove(0);
            }

            totalSamples = samples.size();
            lastIndexAdded = lastIndexesAdded.get(i);

            // add new values
            while (lastIndexAdded + 1 < totalSamples)
            {
                lastIndexAdded++;
                sample = samples.get(lastIndexAdded);
                sampleTimeSec = sample.getTime() / 1000.;
                if (sampleTimeSec >= chartTimeMin)
                {
                    xSeries.getData().add(new XYChart.Data<>(sampleTimeSec, sample.getX()));
                    ySeries.getData().add(new XYChart.Data<>(sampleTimeSec, sample.getY()));
                    thetaSeries.getData().add(new XYChart.Data<>(sampleTimeSec, sample.getTheta()));
                }
                xySeries.getData().add(new XYChart.Data<>(sample.getX(), sample.getY()));
            }
            lastIndexesAdded.set(i, lastIndexAdded);

            // find new min and max of chart X.
            if (xSeries.getData().isEmpty())
            {
                chartXMin = DEFAULT_CHART_X_MIN;
                chartXMax = DEFAULT_CHART_X_MAX;
            }
            else
            {
                tmp = getMinValue(xSeries);
                if (tmp < chartXMin) chartXMin = tmp;
                tmp = getMaxValue(xSeries);
                if (tmp > chartXMax) chartXMax = tmp;
            }

            // find new min and max of chart Y.
            if (ySeries.getData().isEmpty())
            {
                chartYMin = DEFAULT_CHART_Y_MIN;
                chartYMax = DEFAULT_CHART_Y_MAX;
            }
            else
            {
                tmp = getMinValue(ySeries);
                if (tmp < chartYMin) chartYMin = tmp;
                tmp = getMaxValue(ySeries);
                if (tmp > chartYMax) chartYMax = tmp;
            }
        }

        // update timelines
        xAxisX.setLowerBound(chartTimeMin);
        xAxisX.setUpperBound(chartTimeMax);

        yAxisX.setLowerBound(chartTimeMin);
        yAxisX.setUpperBound(chartTimeMax);

        thetaAxisX.setLowerBound(chartTimeMin);
        thetaAxisX.setUpperBound(chartTimeMax);

        tmp = 0.1 * (chartXMax - chartXMin);
        if (tmp == 0.) tmp = 0.1;
        xAxisY.setLowerBound(chartXMin - tmp);
        xAxisY.setUpperBound(chartXMax + tmp);

        tmp = 0.1 * (chartYMax - chartYMin);
        if (tmp == 0.) tmp = 0.1;
        yAxisY.setLowerBound(chartYMin - tmp);
        yAxisY.setUpperBound(chartYMax + tmp);
    }

    private Double getMinValue(XYChart.Series<Double, Double> series) {
        return series.getData().stream().map(XYChart.Data::getYValue).min(Double::compare).orElse(null);
    }

    private Double getMaxValue(XYChart.Series<Double, Double> series) {
        return series.getData().stream().map(XYChart.Data::getYValue).max(Double::compare).orElse(null);
    }
}


