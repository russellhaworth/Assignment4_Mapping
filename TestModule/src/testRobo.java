import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Pose;
import sensors.RoboChassis;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class testRobo {
    //location of the wheel along the y-axis.  Width / 2.
    static final double WIDTH = 21.59;
    static final double OFFSET = 21.59/2;
    static final double DIAMETER = MovePilot.WHEEL_SIZE_EV3; //4.32cm

    static final String MOTOR_PORT_LEFT = "B";
    static final String MOTOR_PORT_RIGHT = "C";
    static final String MOTOR_PORT_CLAW = "D";
    static final String SENSOR_PORT_GYRO = "???";
    static final String SENSOR_PORT_COLOR = "???";
    static final String SENSOR_PORT_IR = "???";
    static final String SENSOR_PORT_ULTRASONIC = "???";
    static final String SENSOR_PORT_TOUCH = "???";


    public static void main(String[] args) throws IOException {
        Brick brick = BrickFinder.getLocal();
        RegulatedMotor left = new EV3LargeRegulatedMotor(brick.getPort(MOTOR_PORT_LEFT)); //Left motor
        RegulatedMotor right = new EV3LargeRegulatedMotor(brick.getPort(MOTOR_PORT_RIGHT)); //Right motor
        RegulatedMotor claw = new EV3LargeRegulatedMotor(brick.getPort(MOTOR_PORT_CLAW)); //Right motor
        EV3GyroSensor gyro = new EV3GyroSensor(brick.getPort(SENSOR_PORT_GYRO));
        Wheel[] wheels = new Wheel[] {
                WheeledChassis.modelWheel(left, DIAMETER).offset(OFFSET),
                WheeledChassis.modelWheel(right, DIAMETER).offset(0-OFFSET)  };
        File f = new File("data.txt");
        FileOutputStream fos = null;
        try{
            fos = new FileOutputStream(f);
        }catch(IOException e){
            System.err.println("Failed to create output stream");
            Button.waitForAnyPress();
            System.exit(1);
        }
        RoboChassis chassis = new RoboChassis(wheels, gyro, fos);

        PaceTest(chassis, 5, 4);

        System.out.println("Test complete.");
        fos.close();
        System.exit(0);
    }

    public static void PaceTest(RoboChassis chassis, int distance, int laps) {
        //
        // total distance = distance * DIAMETER
        PoseProvider gyroProv = chassis.getGyroOdomPoseProvider();
        PoseProvider poseProv = chassis.getPoseProvider();

        for (int i=0; i<laps;i++) {
            System.out.printf("Lap %d - Moving %.2f cm\n", i, distance*DIAMETER);
            printPose("Robot", gyroProv);
            printPose("Robot", poseProv);
            chassis.travel(distance);
            chassis.waitComplete();
            printPose("Robot", gyroProv);
            printPose("Robot", poseProv);
            System.out.println("Rotating 180 degrees.");
            chassis.rotate(180);
            chassis.waitComplete();
            System.out.printf("Lap %d - Moving %.2f cm\n", i, distance*DIAMETER);
            printPose("Robot", gyroProv);
            printPose("Robot", poseProv);
            chassis.travel(distance);
            chassis.waitComplete();
            printPose("Robot", gyroProv);
            printPose("Robot", poseProv);
            System.out.println("Rotating 180 degrees.");
            chassis.rotate(180);
            chassis.waitComplete();
        }
        printPose("Robot", gyroProv);
        printPose("Robot", poseProv);
    }

    public static void SpinTest(RoboChassis chassis, int degrees, int times) {
        PoseProvider gyroProv = chassis.getGyroOdomPoseProvider();
        PoseProvider poseProv = chassis.getPoseProvider();
        for (int i=0;i<times;i++) {

        }
    }

    public static void RandomSpinTest(RoboChassis chassis, int times) {

    }

    public static void writeString(String name, String value) {

    }

    public static void printPose(String name, PoseProvider p) {
        Pose pose = p.getPose();
        String str = String.format("%f, %10s: (%.2f, %.2f) at %.2f degrees.\f", System.currentTimeMillis(), name, pose.getX(), pose.getY(), pose.getHeading());
    }
}
