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
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;
import lejos.hardware.Battery;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;
import lejos.robotics.*;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;



public class Main {
	
	//private static GameField gameField = new GameField();
	private static GameField2 gameField = new GameField2();
	private static EV3Robot robot = new EV3Robot();
	private static SampleSet sampleSet = new SampleSet(robot);
	private static RobotState robotState = new RobotState();
	private static PathFollower pFollow = new PathFollower(robot, gameField);
	private static int mode = 0;
	
	public static void main(String[] args) throws IOException, InterruptedException, DestinationUnreachableException {

		
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
		
	    //Autonomous Mode
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
			//robotState.state = State.GO_TO_MIDDLE;
			//System.out.println("Current State: " + robotState.state);
			LineMap map = new LineMap(gameField.lineArray, gameField.bounds);
	        Pose start = new Pose((float) 12.7, (float) 11.43, 0);
       
	        
	        pFollow.newPose(start);
	        //Waypoint goal = new Waypoint(44.45, 27.94);
	        Waypoint goal = new Waypoint(71.12,57.15);
	        ShortestPathFinder finder = new ShortestPathFinder(map);
	        finder.lengthenLines(20);
	        Path path = finder.findRoute(start, goal);
	        pFollow.newPath(path);
	        pFollow.navigate();
	        
	        
	        if(pFollow.isPathComplete()) {
	        	Sound.beepSequenceUp();
	        }
	        Plotter plotter = new Plotter();
	        float angle = (float) pFollow.getAngle();
	        System.out.println("Angle is " + angle);
	        plotter.setRobotY((float) pFollow.getY());
	        plotter.setRobotX((float)pFollow.getX());
	        plotter.setAngle((float) pFollow.getAngle());
	        pFollow.setAngle(plotter.getAngle());
	        System.out.println("Angle is " + angle);
	        plotter.setEquationX((float) pFollow.getX());
	        plotter.setEquationY((float) pFollow.getY());
	       
	        float value = sampleSet.getLastUltrasonicDistance();
	        System.out.println("hypotenues: " + value);
	        plotter.setValue(value);
	        
	        Waypoint object = plotter.createWayPoint();
	        System.out.println("X value: " +object.x);
	        System.out.println("Y value: " + object.y);
	        Path path2 = finder.findRoute(pFollow.getPose(), object);
	        pFollow.newPath(path2);
	        pFollow.navigate();
	        //Waypoint goal = new Waypoint(71.12, 57.15);
	        
	        
	        //pFollow.newPose(start);
	        
	        
	        if (pFollow.isPathComplete()) {
	            Sound.beepSequenceUp();
	            pFollow.score(pFollow.getPose());
	        }
			
		}
		//Remote Control
		else if(mode == 2) {
			st.start(); //Start sensor thread to ensure robot doesn't run into wall.
			System.out.println("Remote Control Mode");
			int input;
			ServerSocket server = new ServerSocket(1111);
			IsEscapeDownChecker isEscapeDown = new IsEscapeDownChecker(server);
			isEscapeDown.setDaemon(true);
			isEscapeDown.start();
			while (mode == 2) {
				if(sampleSet.getLastIRDistance() <= 17) {
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
