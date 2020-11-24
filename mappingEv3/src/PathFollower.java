import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;

public class PathFollower{
    private DifferentialPilot pilot = new DifferentialPilot(3.25, 19.8, Motor.C, Motor.B);
    private Navigator navigator = new Navigator(pilot);
    private Waypoint next;
    public void newWaypoint(int x, int y){
        navigator.addWaypoint(x, y);
    }

    public void newPose(Pose pose){
        navigator.setPoseProvider((PoseProvider) pose);
    }
    public void navigate(){
        while(!navigator.pathCompleted()){
            navigator.followPath();
            next = navigator.getWaypoint();
            LCD.drawString("Moving to...", 0, 0);
            LCD.drawString("(" + (int)next.getX() +
                    "," + (int)next.getY() + ")", 0, 1);
        }
    }

    public void newPath(Path path) {
        navigator.setPath(path);
    }
}
