import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;

public class test {
    public static void main(String [] args) throws DestinationUnreachableException {
        GameField gameField = new GameField();
        LineMap map = new LineMap(gameField.lineArray, gameField.bounds);
        Pose start = new Pose(0, 0, 270);
        Waypoint goal = new Waypoint(125, 150);
        ShortestPathFinder finder = new ShortestPathFinder(map);
        finder.lengthenLines(5);
        PathFollower pFollow = new PathFollower();
        pFollow.newPose(start);
        Path path = finder.findRoute(start, goal);
        pFollow.newPath(path);
        pFollow.navigate();

    }

}
