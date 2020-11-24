package main;


public class SampleSet {
    private float lastIRDistance;
    private float lastUltrasonicDistance;
    private float lastGyroRate;
    private float lastGyroAngle;
    private String lastColor;
    private final EV3Robot robot;

    public SampleSet(EV3Robot robot) {
        this.robot = robot;
    }

    public void takeSamples(){
        setLastIRDistance(robot.getIRDistance());
        setLastUltrasonicDistance(robot.getUltrasonicDistance());

    }
    
    public float getLastUltrasonicDistance() {
        return lastUltrasonicDistance;
    }

    public void setLastUltrasonicDistance(float lastUltrasonicDistance) {
        this.lastUltrasonicDistance = lastUltrasonicDistance;
    }
    
    public float getLastGyroRate() {
        return lastGyroRate;
    }

    public void setLastGyroRate(float lastGyroRate) {
        this.lastGyroRate = lastGyroRate;
    }
    
    public float getLastGyroAngle() {
        return lastGyroAngle;
    }

    public void setLastGyroAngle(float lastGyroAngle) {
        this.lastGyroAngle = lastGyroAngle;
    }    
    

    public float getLastIRDistance() {
        return lastIRDistance;
    }

    public void setLastIRDistance(float lastIRDistance) {
        this.lastIRDistance = lastIRDistance;
    }

    public String getLastColor() {
        return lastColor;
    }

    public void setLastColor(String lastColor) {
        this.lastColor = lastColor;
    }
}
