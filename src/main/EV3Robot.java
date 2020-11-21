package main;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.navigation.MovePilot;

public class EV3Robot {
	private String robotState;
	EV3LargeRegulatedMotor motorA = new EV3LargeRegulatedMotor(MotorPort.B); //Left motor
	EV3LargeRegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.C); //Right motor
	EV3MediumRegulatedMotor motorC = new EV3MediumRegulatedMotor(MotorPort.D);
	double diam = MovePilot.WHEEL_SIZE_EV3;
    double trackwidth = 23;
	MovePilot pilot = new MovePilot(diam, trackwidth, motorA, motorB);
	
	
	public EV3Robot() {
		
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
		motorA.forward();;
		motorB.backward();;
	}
	
	public void robotRotateRight() {
		motorA.backward();
		motorB.forward();
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
	}
	
	public void openClaw() {
		motorC.forward();
	}
}
