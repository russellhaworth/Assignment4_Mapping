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
				//System.out.println("ENTER CONTROL THREAD");
		while(robotState.shouldRun) {
			if(sampleSet.getLastUltrasonicDistance() < 50) {
				robot.setRobotState(robot.FOUND_OBJECT);
				System.out.println("ROBOT STATE: FOUND OBJECT");
			}
			//System.out.println("IR Distance: " + sampleSet.getLastIRDistance());
			//System.out.println("ULT Distance: " + sampleSet.getLastUltrasonicDistance());
		}
		while(robot.getRobotState() == robot.FOUND_OBJECT) {
			if(sampleSet.getLastUltrasonicDistance() < 10) {
				if(sampleSet.getLastIRDistance() < 30) {
					robot.setRobotState(robot.FOUND_OBSTACLE);
					System.out.println("ROBOT STATE: FOUND OBSTACLE");
				} else {
					robot.setRobotState(robot.FOUND_BALL);
					System.out.println("ROBOT STATE: FOUND BALL");
				}
			}
		}
				
	}
}
