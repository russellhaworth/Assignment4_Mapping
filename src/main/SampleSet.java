package main;


public class SampleSet {
    private float lastIRDistance;
    private float lastUltrasonicDistance;
    private float lastGyroRate;
    private float lastGyroAngle;
    private float[] lastColor;
    private final EV3Robot robot;
    private float lastRedColor;
    private float lastGreenColor;
    private float lastBlueColor;
    private String lastColorString;
    private String currentColorString;

    public SampleSet(EV3Robot robot) {
        this.robot = robot;
    }

    public void takeSamples(){
        setLastIRDistance(robot.getIRDistance());
        setLastUltrasonicDistance(robot.getUltrasonicDistance());
        setLastColor(robot.getLastColor());
        setLastColorString();
        

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

    public float[] getLastColor() {
        return lastColor;
    }
    
    public float getLastRedColor() {
    	return lastRedColor*100;
    }
    
    public float getLastGreenColor() {
    	return lastGreenColor*100;
    }
    
    public float getLastBlueColor() {
    	return lastBlueColor*100;
    }
    
    public void setLastColorString() {
    	if(getLastBlueColor() <= 1.5 && getLastGreenColor() <= 1.5 && getLastRedColor() <= 1.5) {
    		this.currentColorString = "BLACK";
    	} 
    	else if(getLastBlueColor() <= 1.5 && getLastGreenColor() >= 3 && getLastRedColor() <= 1.5) {
    		this.currentColorString = "GREEN";
    	}
    	else if(getLastBlueColor() >= 4 && getLastGreenColor() <= 3.0 && getLastRedColor() <= 1.8) {
    		this.currentColorString = "BLUE";
    	}
    	else if(getLastBlueColor() >= 5 && getLastGreenColor() >= 5.0 && getLastRedColor() >= 5.0) {
    		this.currentColorString = "WHITE";
    	}
    }
    
    public String getCurrentColorString() {
    	return currentColorString;
    }

    public void setLastColor(float[] lastColor) {
        this.lastColor = lastColor;
        this.lastRedColor = lastColor[0];
        this.lastGreenColor = lastColor[1];
        this.lastBlueColor = lastColor[2];
    }
    
}
