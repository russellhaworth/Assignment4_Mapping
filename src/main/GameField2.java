package main;

import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;

public class GameField2 {
    Line[] lineArray = new Line[1];
    float length = (float)142.24;
    float width = (float) 114.3;
    float spacing = (float) 21.59;
    Rectangle bounds = new Rectangle(0,0,length, width);


    public GameField2() {
        drawLines();
    }

    public Line[] getLineArray() {
        return lineArray;
    }

    private void drawLines(){
        Line newLine = new Line((float)53.34,(float)0.0,(float)53.34,(float)27.94);
        lineArray[0] = newLine;

    }


    public void setLineArray(Line[] lineArray) {
        this.lineArray = lineArray;
    }

    public float getLength() {
        return length;
    }

    public void setLength(float length) {
        this.length = length;
    }

    public float getWidth() {
        return width;
    }

    public void setWidth(float width) {
        this.width = width;
    }

    public float getSpacing() {
        return spacing;
    }

    public void setSpacing(int spacing) {
        this.spacing = spacing;
    }

    public Rectangle getBounds() {
        return bounds;
    }

    public void setBounds(Rectangle bounds) {
        this.bounds = bounds;
    }

    public LineMap createLineMap(){
        LineMap lineMap = new LineMap(lineArray, bounds);
        return lineMap;
    }
}
