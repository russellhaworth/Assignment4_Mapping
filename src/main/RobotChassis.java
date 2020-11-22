package main;

import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.utility.Matrix;

public class RobotChassis extends WheeledChassis {
	//docs
	//https://github.com/bdeneuter/lejos-ev3/blob/master/src/main/java/lejos/robotics/chassis/WheeledChassis.java
	
	//implementation example
	//https://github.com/gloomyandy/surveyor/blob/master/Surveyor/src/LineFollowerChassis.java
	

    Wheel[] wheels;
    RegulatedMotor[] motors;
    Matrix reverse2;
    Matrix forward2;
	
	
    public RobotChassis(Wheel[] wheels, int dim) {
		super(wheels, dim);
		// TODO Auto-generated constructor stub
	}

}
