package main;

import lejos.utility.Stopwatch;

import java.awt.Color;

import lejos.utility.Delay;




public class ControlThread implements Runnable {

	private EV3Robot robot;
	private SampleSet sampleSet;
	private RobotState robotState;



	public ControlThread(RobotState robotState, EV3Robot robot, SampleSet sampleSet) {
		this.robot = robot;
		this.sampleSet = sampleSet;
		this.robotState	= robotState;
	}


	@Override
	public void run() {
		while(robotState.shouldRun) {
			
			/*
			System.out.println("RED: " + sampleSet.getLastRedColor());
			System.out.println("GREEN: " + sampleSet.getLastGreenColor());
			System.out.println("BLUE " + sampleSet.getLastBlueColor());
			Delay.msDelay(1000);
			*/
			
			System.out.println("CURRENT COLOR " + sampleSet.getCurrentColorString());
			
			
			System.out.println("CURRENT STATE: " + robotState.state);
			
			if(robotState.state == State.GO_TO_MIDDLE && sampleSet.getCurrentColorString()=="BLACK") {
				robotState.state = State.ROTATE_360;
			}
			if(robotState.state == State.ROTATE_360 && sampleSet.getLastUltrasonicDistance() <= 20) {
				robotState.state = State.FOUND_OBJECT;
			}
			if(robotState.state == State.FOUND_OBJECT) {
				robotState.state = State.GO_TO_OBJECT;
			}
			if(robotState.state == State.GO_TO_OBJECT && sampleSet.getLastUltrasonicDistance() <= 10) {
				robotState.state = State.STOPPED_IN_FRONT_OF_OBJECT;
			}
			if(robotState.state == State.STOPPED_IN_FRONT_OF_OBJECT) {
				if(sampleSet.getLastUltrasonicDistance() <= 10 && sampleSet.getLastIRDistance() >= 35) {
					robotState.state = State.FOUND_BALL;
				}
				else if(sampleSet.getLastUltrasonicDistance() <= 10 && sampleSet.getLastIRDistance() < 35) {
					robotState.state = State.FOUND_OBSTACLE;
				}
			}
			
			
		}
	}
}
