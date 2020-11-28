import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.*;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;

public class PathFollower{
    @SuppressWarnings("deprecation")
    final Waypoint scoreSquare = new Waypoint(137.16,22.32);
    private GameField gameField = new GameField();
    final Waypoint scoreNode = new Waypoint(125.92,22.86);
    private DifferentialPilot pilot = new DifferentialPilot(3.25, 19.8, Motor.C, Motor.B);
    private EV3MediumRegulatedMotor claw = new EV3MediumRegulatedMotor(MotorPort.D);
    private Navigator navigator = new Navigator(pilot);
    private Waypoint next;
    public void newWaypoint(int x, int y){
        navigator.addWaypoint(x, y);
    }

    public void newPose(Pose pose){
        navigator.getPoseProvider().setPose(pose);
    }
    public Pose getPose(){ return navigator.getPoseProvider().getPose();}
    public void navigate(){
        while(!navigator.pathCompleted()){
            navigator.followPath();
            next = navigator.getWaypoint();
            LCD.drawString("Moving to...", 0, 0);
            LCD.drawString("(" + (int)next.getX() +
                    "," + (int)next.getY() + ")", 0, 1);
        }
    }
    //need to feed in the current position of robot
    public void score (Pose start) throws DestinationUnreachableException, InterruptedException {
        LineMap map = new LineMap(gameField.lineArray, gameField.bounds);
        ShortestPathFinder finder = new ShortestPathFinder(map);
        finder.lengthenLines(30);
        newPose(start);
        Path path = finder.findRoute(start, scoreNode);
        newPath(path);
        navigate();
        if(navigator.pathCompleted()) {
            Sound.beepSequenceUp();
            turnToGoal();
            claw.backward();
            wait(6);
            claw.stop();
        }
    }

    public void newPath(Path path) {navigator.setPath(path);
    }

    public void turnToGoal(){
        float turnAngle = getPose().angleTo(scoreSquare);
        navigator.rotateTo(turnAngle);
    }
    public boolean isPathComplete() {
        return navigator.pathCompleted();
    }
}

