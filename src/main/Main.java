package main;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.Battery;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;
import lejos.robotics.*;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class Main {
	
	public static void main(String[] args) throws IOException {
		final GameField gameField = new GameField();
		final EV3Robot robot = new EV3Robot();
		
		EV3IRSensor ir1 = new EV3IRSensor(SensorPort.S3);
		final SampleProvider spIR = ir1.getDistanceMode();
		int distanceValueIR = 0;
		
		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
	    ultrasonicSensor.setCurrentMode(0);
	    final SampleProvider spUltrasonic = ultrasonicSensor.getDistanceMode();
	    double distanceValueUltrasonic = 0;
	    
		System.out.println("Press Left Button for Autonomous Mode.");
		System.out.println("Press Right Button for Remote Control Mode.");
		Button.waitForAnyPress();
		//LCD.clearDisplay();
		if(Button.LEFT.isDown()) {
			robot.setRobotState("INITIAL");
			robot.playBeep();
			System.out.println("STATE: INITIAL");
			System.out.println("PRESS ANY BUTTON TO BEGIN OFFENSE");
			Button.waitForAnyPress();
			LCD.clearDisplay();
			robot.setRobotState("OFFENSE");
			while(robot.getRobotState() == "OFFENSE") {
				
				float [] sampleIR = new float[spIR.sampleSize()];
	        	spIR.fetchSample(sampleIR, 0);
	        	distanceValueIR = (int)sampleIR[0];
	        	
	        	float [] sampleUltrasonic = new float[spUltrasonic.sampleSize()];
	        	spUltrasonic.fetchSample(sampleUltrasonic, 0);
	        	distanceValueUltrasonic = (double)sampleUltrasonic[0]*100;
	        	
	        	System.out.println("IR Distance = " + distanceValueIR);
	        	System.out.println("Ultrasonic Distance = " + distanceValueUltrasonic);
	        	Delay.msDelay(1000);
			}
			
			
			
		}
		else if(Button.RIGHT.isDown()) {
			System.out.println("Remote Control Mode");
			int input;
			ServerSocket server = new ServerSocket(1111);
			IsEscapeDownChecker isEscapeDown = new IsEscapeDownChecker(server);
			isEscapeDown.setDaemon(true);
			isEscapeDown.start();
			while (true) {
				Socket socket;
				try {
					socket = server.accept();
				} catch (IOException e) {
					break;
				}
				DataInputStream in = new DataInputStream(socket.getInputStream());
				input = in.readInt();
				System.out.println("Input: " + input);
				
				switch(input) {
				case 1: robot.robotForward();
				case 2: robot.robotReverse();
				case 3: robot.robotRotateRight();
				case 4: robot.robotRotateLeft();
				case 5: robot.robotStop();
				case 6: {
					robot.robotExitRemote();
					Sound.setVolume(100);
					Sound.buzz();
					server.close();
					}
				case 7: robot.robotHonk();
				case 8: robot.closeClaw();
				case 9: robot.openClaw();
				}
				
			
			}
		
		}

	}
}
