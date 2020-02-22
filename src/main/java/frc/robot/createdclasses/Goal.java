package frc.robot.createdclasses;

import java.util.ArrayList;

import frc.robot.Constants;

import java.awt.Point;

public class Goal {

    private double objectID;
    private double centerX;
    private double centerY;
    private double endX;
    private double endY;
    private double area;
    private double confidence;

    public Goal() {

        this.objectID = 0;
        this.centerX = 0;
        this.centerY = 0;
        this.endX = 0;
        this.endY = 0;
        this.area = 0;
        this.confidence = 0;

    }

    public Goal(ArrayList<Double> evsData) {

        this.objectID = evsData.get(0);
        this.centerX = evsData.get(1);
        this.centerY = evsData.get(2);
        this.endX = evsData.get(3);
        this.endY = evsData.get(4);
        this.area = evsData.get(5);
        this.confidence = evsData.get(6);

    }

    public double getObjectID() {

        return objectID;

    }

    public double getDistance() {

        return Math.sqrt((Math.pow(Constants.GOAL_DISTANCE, 2) * Constants.GOAL_AREA) / getArea());

    }

    public double getHeight() {

        return 2 * (centerX - endY);

    }

    public double getCenterX() {

        return centerX;

    }

    public double getCenterY() {

        return centerY;

    }

    public double getEndX() {

        return endX;

    }

    public double getEndY() {

        return endY;

    }

    public Point getCenter() {

        Point p = new Point();
        p.x = (int) centerX;
        p.y = (int) (endY + getHeight());
        return p;

    }

    public Point getEnd() {

        Point p = new Point();
        p.x = (int) endX;
        p.y = (int) endY;
        return p;

    }

    public double getArea() {

        return area;

    }

    public double getConfidence() {

        return confidence;

    }

    public void update(ArrayList<Double> evsData) {

        this.objectID = evsData.get(0);
        this.centerX = evsData.get(1);
        this.centerY = evsData.get(2);
        this.endX = evsData.get(3);
        this.endY = evsData.get(4);
        this.area = evsData.get(5);
        this.confidence = evsData.get(6);

    }

}