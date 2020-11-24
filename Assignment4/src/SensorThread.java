

public class SensorThread implements Runnable {

	private EV3Robot robot;
	private SampleSet sampleSet;
	private RobotState robotState;

	public SensorThread(RobotState robotState, EV3Robot robot, SampleSet sampleSet) {
		this.robot = robot;
		this.sampleSet = sampleSet;
		this.robotState = robotState;
	}

	@Override
	public void run() {
		while(robotState.shouldRun) {
			sampleSet.takeSamples();
		}
	}
}

