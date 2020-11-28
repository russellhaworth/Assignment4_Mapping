import lejos.hardware.Sound;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;

public class test {
    public static void main(String [] args) throws DestinationUnreachableException, InterruptedException {

        //travel to middle of field then score
        GameField gameField = new GameField();
        LineMap map = new LineMap(gameField.lineArray, gameField.bounds);
        Pose start = new Pose((float) 12.7, (float) 11.43, 0);
        PathFollower pFollow = new PathFollower();

        Waypoint goal = new Waypoint(71.12, 57.15);
        ShortestPathFinder finder = new ShortestPathFinder(map);
        finder.lengthenLines(30);
        pFollow.newPose(start);
        Path path = finder.findRoute(start, goal);
        pFollow.newPath(path);
        pFollow.navigate();
        if (pFollow.isPathComplete()) {
            Sound.beepSequenceUp();
            pFollow.score(pFollow.getPose());
        }
    }

}
