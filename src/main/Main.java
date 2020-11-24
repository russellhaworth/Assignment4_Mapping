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
	
	private static GameField gameField = new GameField();
	private static EV3Robot robot = new EV3Robot();
	private static SampleSet sampleSet = new SampleSet(robot);
	private static RobotState robotState = new RobotState();
	private static int mode = 0;
	
	public static void main(String[] args) throws IOException, InterruptedException {

		
		Thread mt = new Thread(new MotorThread(robotState, robot));
		Thread st = new Thread(new SensorThread(robotState, robot ,sampleSet));
		Thread ct = new Thread(new ControlThread(robotState, robot, sampleSet));
		
		
	    if(mode == 0) {
	    	System.out.println("Press Left Button for Autonomous Mode.");
			System.out.println("Press Right Button for Remote Control Mode.");
			Button.waitForAnyPress();
			//LCD.clearDisplay();
			if(Button.LEFT.isDown()) {
				mode = 1; //Set Autonomous mode
			} else if(Button.RIGHT.isDown()) {
				mode = 2; //Remote Control Mode
			}
	    }
		
		if(mode == 1) {
			robotState.state = State.INITIAL;
			robot.playBeep();
			//System.out.println("STATE: " + robot.getRobotState());
			System.out.println("PRESS ANY BUTTON TO BEGIN OFFENSE");
			Button.waitForAnyPress();
			LCD.clearDisplay();
			mt.start();
			st.start();
			ct.start();		
			//robotState.state = State.GO_TO_MIDDLE_AND_ROTATE;
			System.out.println("Current State: " + robotState.state);
			
		}
		else if(mode == 2) {
			st.start(); //Start sensor thread to ensure robot doesn't run into wall.
			System.out.println("Remote Control Mode");
			int input;
			ServerSocket server = new ServerSocket(1111);
			IsEscapeDownChecker isEscapeDown = new IsEscapeDownChecker(server);
			isEscapeDown.setDaemon(true);
			isEscapeDown.start();
			while (mode == 2) {
				if(sampleSet.getLastUltrasonicDistance() <= 15) {
					input = 999;
				} else {
					Socket socket;
					try {
						socket = server.accept();
					} catch (IOException e) {
						break;
					}
					DataInputStream in = new DataInputStream(socket.getInputStream());
					input = in.readInt();
					System.out.println("Input: " + input);
				}	
				
				switch(input) {
				case 1: 
					robot.robotForward();
					break;
				case 2: 
					robot.robotReverse();
					break;
				case 3: robot.robotRotateRight();
					break;
				case 4: 
					robot.robotRotateLeft();
					break;
				case 5: 
					robot.robotStop();
					break;
				case 6: 
					robot.robotExitRemote();
					Sound.setVolume(100);
					Sound.buzz();
					server.close();
					mode = 0;
					break;
				case 7: 
					robot.robotHonk();
					break;
				case 8: 
					robot.closeClaw();
					break;
				case 9: 
					robot.openClaw();
					break;
				case 999:
					robot.sentient(); //Robot saves itself from harm
				}
				
			
			}
		
		}

	}
}
