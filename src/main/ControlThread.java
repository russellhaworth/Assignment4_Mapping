package main;

import lejos.utility.Stopwatch;
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
			if(robotState.state == State.GO_TO_MIDDLE_AND_ROTATE) {
				if(sampleSet.getLastUltrasonicDistance() < 30) {
					robotState.state = State.FOUND_OBJECT;
				}
			}
			if(robotState.state == State.FOUND_OBJECT && sampleSet.getLastUltrasonicDistance() <= 10) {
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
