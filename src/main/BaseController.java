package main;

import javafx.stage.Stage;

public abstract class BaseController implements IController {
    protected Stage primaryStage;

    @Override
    public void init() {
    }

    @Override
    public void setStage(Stage stage) {
        primaryStage = stage;
    }
}
