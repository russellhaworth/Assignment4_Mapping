import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;

import java.util.ArrayList;
import java.util.List;

public class GameField {
    List<Line>lineList = new ArrayList<Line>();
    Line[] lineArray = lineList.toArray(new Line[0]);
    float length = 142;
    float width = 114;
    int spacing = 14;
    Rectangle bounds = new Rectangle(0,0,142,114);


    public GameField() {
        drawLines();
    }

    public Line[] getLineArray() {
        return lineArray;
    }

    private void drawLines(){
        Line newLine;
        int p = spacing;
        //todo: forms the colored verticle lines
        for(int i = 0; i < 7; i ++){
            newLine = new Line(p,0, p, 114);
            lineList.add(newLine);
            p += spacing;
        }
         //todo: figure horizontal line placement
        p = spacing;
        for(int i = 0; i < length/spacing; i++){
            newLine = new Line(0,p, 0, 142);
            lineList.add(newLine);
            p += spacing;
        }
    }

    public List<Line> getLineList() {
        return lineList;
    }

    public void setLineList(List<Line> lineList) {
        this.lineList = lineList;
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

    public int getSpacing() {
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
