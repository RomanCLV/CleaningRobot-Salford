package main;

import javafx.fxml.FXML;
import javafx.scene.chart.LineChart;
import javafx.scene.paint.Color;
import java.util.ArrayList;
import java.util.List;

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

    private final List<PositionSeries> positionSeriesList;
    //endregion

    public PositionChartsController() {
        positionSeriesList = new ArrayList<>();
    }

    public PositionSeries createPositionSeries(String name, Color color) {
        PositionSeries newSeries = new PositionSeries(name, color);
        positionSeriesList.add(newSeries);
        return newSeries;
    }

}
