import lejos.robotics.navigation.Waypoint;

public class Plotter {
    float equationX;
    float equationY;
    float angle;
    float value;
    float robotX;
    float robotY;

    public float plotX(){
        float x = (float) ((value*Math.cos(angle)) + robotX);
        setEquationX((float) (x - 12));
        return (float) (x - 12);
    }

    public float plotY(){
        float y = (float) ((value*Math.sin(angle)) + robotY);
        setEquationY(y);
        return y ;
    }

    public Waypoint createWayPoint(){
        Waypoint waypoint = new Waypoint(plotX(), plotY());
        return waypoint;
    }


    public float getEquationX() {
        return equationX;
    }

    public void setEquationX(float equationX) {
        this.equationX = equationX;
    }

    public float getEquationY() {
        return equationY;
    }

    public void setEquationY(float equationY) {
        this.equationY = equationY;
    }

    public float getAngle() {
        return angle;
    }

    public void setAngle(float angle) {
        this.angle = angle;
    }

    public float getValue() {
        return value;
    }

    public void setValue(float value) {
        this.value = value;
    }

    public float getRobotX() {
        return robotX;
    }

    public void setRobotX(float robotX) {
        this.robotX = robotX;
    }

    public float getRobotY() {
        return robotY;
    }

    public void setRobotY(float robotY) {
        this.robotY = robotY;
    }

    public Plotter() {
    }

    public Plotter(float angle, float robotX, float robotY) {
        this.angle = angle;
        this.robotX = robotX;
        this.robotY = robotY;
    }
}
