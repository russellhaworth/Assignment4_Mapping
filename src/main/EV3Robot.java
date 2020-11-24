package main;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;

public class EV3Robot {
	public static final String OFFENSE = "OFFENSE";
	public static final String INITIAL = "INITIAL";
	public static final String FOUND_BALL = "FOUND_BALL";
	public static final String FOUND_OBJECT = "FOUND_OBJECT";
	public static final String FOUND_OBSTACLE = "FOUND OBSTACLE";
	private String robotState;
	EV3LargeRegulatedMotor motorA; //Left motor
	EV3LargeRegulatedMotor motorB; //Right motor
	EV3MediumRegulatedMotor motorC; //Claw Motor
	
	double diam = MovePilot.WHEEL_SIZE_EV3;
    double trackwidth = 23;
	MovePilot pilot;
	
	private EV3IRSensor irSensor;
	private SampleProvider rangeSampler;
	private float[] lastRange;
	
	private EV3UltrasonicSensor ultSensor;
	private SampleProvider ultSampler;
	private float[] lastUltRange;
	
	
	
	public EV3Robot() {
	    motorA = new EV3LargeRegulatedMotor(MotorPort.B);
	    motorB = new EV3LargeRegulatedMotor(MotorPort.C);
	    motorC = new EV3MediumRegulatedMotor(MotorPort.D);
	    pilot = new MovePilot(diam, trackwidth, motorA, motorB);
	    
	    irSensor = new EV3IRSensor(SensorPort.S3);
	    rangeSampler = irSensor.getDistanceMode();
	    lastRange = new float[rangeSampler.sampleSize()];
	    
	    ultSensor = new EV3UltrasonicSensor(SensorPort.S4);
	    ultSampler = ultSensor.getDistanceMode();
	    lastUltRange = new float[ultSampler.sampleSize()];
	    
	   
	}
	
	
	public float getIRDistance() {
	    rangeSampler.fetchSample(lastRange, 0);
	    return lastRange[0];
	}
	
	
	public float getUltrasonicDistance() {
		ultSampler.fetchSample(lastUltRange, 0);
		return lastUltRange[0]*100;
	}
	
	
	public void setRobotState(String robotState){
		this.robotState = robotState;
	}
	
	public String getRobotState() {
		return this.robotState;
	}
	
	public void playBeep() {
	    Sound.beep();
	}
	
	public void robotForward() {
		motorA.forward();
		motorB.forward();
	}
	
	public void robotRotateLeft() {
		motorA.backward();
		motorB.forward();
	}
	
	public void robotRotateRight() {
		motorA.forward();
		motorB.backward();
	}
	
	public void robotReverse() {
		motorA.backward();
		motorB.backward();
	}
	
	public void robotStop() {
		motorA.stop();
		motorB.stop();
	}
	
	public void robotHonk() {
		Sound.setVolume(100);
		Sound.beep();
	}
	
	public void robotExitRemote() {
		Sound.setVolume(100);
		Sound.buzz();
		motorA.close();
		motorB.close();
	}
	
	public void closeClaw() {
		motorC.backward();
		if(motorC.isStalled()) {
			motorC.stop();
		}
	}
	
	public void openClaw() {
		motorC.forward();
		if(motorC.isStalled()) {
			motorC.stop();
		}
	}
	
	public void sentient() {
		pilot.stop();
		pilot.travel(-5);
		pilot.rotate(180);
	}
}
