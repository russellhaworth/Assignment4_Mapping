package main;

import lejos.robotics.navigation.MovePilot;

public class MotorThread implements Runnable {

	private EV3Robot robot;
	private RobotState robotState;

	public MotorThread(RobotState robotState, EV3Robot robot) {
		this.robot = robot;
		this.robotState = robotState;
	}

	@Override
	public void run() {
		while(robotState.shouldRun) {
			if(robotState.state != null) {
				if(robotState.state == State.GO_TO_MIDDLE_AND_ROTATE) {
					robot.pilot.travel(70);
					robot.pilot.rotate(360);
				}
				if(robotState.state == State.ROTATE_360) {
					robot.pilot.rotate(360);
				}
				if(robotState.state == State.FOUND_OBJECT) {
					robot.pilot.stop();
					robot.pilot.forward();
				}
				if(robotState.state == State.FOUND_BALL) {
					robot.pilot.stop();
				}
				if(robotState.state == State.FOUND_OBSTACLE) {
					robot.pilot.stop();
				}
			}
		}
		
	}
	
}

