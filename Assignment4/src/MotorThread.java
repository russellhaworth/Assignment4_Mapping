
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
			
		}
		
	}
	
}

