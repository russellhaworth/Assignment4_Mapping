import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Pose;
import sensors.RoboChassis;

import java.io.IOException;

public class testRobo {
    Brick brick;
    Wheel[] wheels;
    private String robotState;
    EV3LargeRegulatedMotor motorA;
    EV3LargeRegulatedMotor motorB;
    EV3MediumRegulatedMotor motorC;
    EV3GyroSensor gyro;
    RoboChassis chassis;
    double diam;
    double trackwidth;
    MovePilot pilot = new MovePilot(diam, trackwidth, motorA, motorB);
    PoseProvider poseByGyroProvider, poseProvider;
    Pose currentPoseByGyro, currentPose;

    public void init() {
        //The distance between the robots yPose-axis and the center of the wheel
        int offset = 0;

        Brick brick = BrickFinder.getLocal();
        RegulatedMotor left = new EV3LargeRegulatedMotor(brick.getPort("B")); //Left motor
        RegulatedMotor right = new EV3LargeRegulatedMotor(brick.getPort("C")); //Right motor
        RegulatedMotor claw = new EV3LargeRegulatedMotor(brick.getPort("D")); //Right motor
        EV3GyroSensor gyro = new EV3GyroSensor(brick.getPort("?????"));
        wheels = new Wheel[] { WheeledChassis.modelWheel(left, MovePilot.WHEEL_SIZE_EV3).offset(offset),
                WheeledChassis.modelWheel(right, 56.0).offset(0-offset)};
        chassis = new RoboChassis(wheels, gyro);
        diam = MovePilot.WHEEL_SIZE_EV3;
        poseByGyroProvider = chassis.getGyroOdomPoseProvider();
        poseProvider = chassis.getPoseProvider();

    }

    public void printPose() {
        currentPoseByGyro = poseByGyroProvider.getPose();
        currentPose = poseProvider.getPose();
        System.out.println("Current position: ");
        System.out.printf("with gyro: (%.2f, %.2f) at %3.2f degrees\n", currentPoseByGyro.getX(), currentPoseByGyro.getY(), currentPoseByGyro.getHeading());
        System.out.printf("w/o  gyro: (%.2f, %.2f) at %3.2f degrees\n", currentPose.getX(), currentPose.getY(), currentPose.getHeading());

    }


    public static void main(String[] args) throws IOException {
        testRobo robo = new testRobo();
        robo.init();
        robo.printPose();
        RoboChassis chassis = robo.chassis;
        System.out.println("Moving 5x Wheel Diamater.");
        chassis.travel(5);
        chassis.waitComplete();
        robo.printPose();
        System.out.println("Rotating 180 degrees.");
        chassis.rotate(180);
        chassis.waitComplete();
        robo.printPose();
        System.out.println("Moving 5x Wheel Diamater.");
        chassis.travel(5);
        chassis.waitComplete();
        robo.printPose();



    }
}
