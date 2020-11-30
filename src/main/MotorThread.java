package main;

import lejos.robotics.navigation.MovePilot;

public class MotorThread implements Runnable {

	private EV3Robot robot;
	private RobotState robotState;
	private boolean threadInterrupted = false;

	public MotorThread(RobotState robotState, EV3Robot robot) {
		this.robot = robot;
		this.robotState = robotState;
	}

	@Override
	public void run() {
		while(robotState.shouldRun) {
			if(robotState.state != null) {
				if(robotState.state == State.GO_TO_MIDDLE) {
					if(!robot.pilot.isMoving()) {
						robot.pilot.setLinearSpeed(10);
						robot.pilot.forward();
					}					
				}
				if(robotState.state == State.ROTATE) {
					robot.pilot.rotate(5);
				}
				if(robotState.state == State.FOUND_OBJECT) {
					robot.pilot.stop();
				}
				if(robotState.state == State.GO_TO_OBJECT) {
					if(!robot.pilot.isMoving()) {
						robot.pilot.forward();
					}
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

