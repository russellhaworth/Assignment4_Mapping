import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;
import mapping.Plotter;

public class test {
	
	  public static void main(String[] args) throws DestinationUnreachableException, InterruptedException {

	        //travel to middle of field then score
	        GameField gameField = new GameField();
	        LineMap map = new LineMap(gameField.lineArray, gameField.bounds);
	        Pose start = new Pose((float) 12.7, (float) 11.43, 0);
	        PathFollower pFollow = new PathFollower();
	        EV3UltrasonicSensor ultSensor = new EV3UltrasonicSensor(SensorPort.S4);
	        SampleProvider ultSampler  = ultSensor.getDistanceMode();
	        float[] lastUltRange = new float[ultSampler.sampleSize()];
	        
	        
	        pFollow.newPose(start);
	        //Waypoint goal = new Waypoint(44.45, 27.94);
	        Waypoint goal = new Waypoint(71.12,57.15);
	        ShortestPathFinder finder = new ShortestPathFinder(map);
	        finder.lengthenLines(20);
	        Path path = finder.findRoute(start, goal);
	        pFollow.newPath(path);
	        pFollow.navigate();
	        
	        
	        if(pFollow.isPathComplete()) {
	        	Sound.beepSequenceUp();
	        }
	        Plotter plotter = new Plotter();
	        float angle = (float) pFollow.getAngle();
	        System.out.println("Angle is " + angle);
	        plotter.setRobotY((float) pFollow.getY());
	        plotter.setRobotX((float)pFollow.getX());
	        plotter.setAngle((float) pFollow.getAngle());
	        pFollow.setAngle(plotter.getAngle());
	        System.out.println("Angle is " + angle);
	        plotter.setEquationX((float) pFollow.getX());
	        plotter.setEquationY((float) pFollow.getY());
	       
	        float value = getUltrasonicDistance(ultSampler, lastUltRange);
	        System.out.println("hypotenues: " + value);
	        plotter.setValue(value);
	        
	        Waypoint object = plotter.createWayPoint();
	        System.out.println("X value: " +object.x);
	        System.out.println("Y value: " + object.y);
	        Path path2 = finder.findRoute(pFollow.getPose(), object);
	        pFollow.newPath(path2);
	        pFollow.navigate();
	        //Waypoint goal = new Waypoint(71.12, 57.15);
	        
	        
	        //pFollow.newPose(start);
	        
	        
	        if (pFollow.isPathComplete()) {
	            Sound.beepSequenceUp();
	            pFollow.score(pFollow.getPose());
	        }
	    }
	    public static float getUltrasonicDistance(SampleProvider ultSampler, float[] lastUltRange) {
	        ultSampler.fetchSample(lastUltRange, 0);
	        return lastUltRange[0]*100;
	    }

//    public static void main(String [] args) throws DestinationUnreachableException, InterruptedException {
//        GameField gameField = new GameField();
//        LineMap map = new LineMap(gameField.lineArray, gameField.bounds);
//        Pose start = new Pose((float) 12.7, (float) 11.43, 0);
//        PathFollower pFollow = new PathFollower();
//        
//        
//        
//        
//        Waypoint goal = new Waypoint(71.12,57.15);
//      ShortestPathFinder finder = new ShortestPathFinder(map);
//      finder.lengthenLines(30);
//      pFollow.newPose(start);
//      Path path = finder.findRoute(start, goal);
//      pFollow.newPath(path);
//      pFollow.navigate();
//      if(pFollow.isPathComplete()) {
//    	  Sound.beepSequenceUp();
//    	  pFollow.score(pFollow.getPose());
//      }
//      
//    }

}
