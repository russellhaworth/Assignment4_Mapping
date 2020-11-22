package main;

import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.GyroscopeAdapter;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;
import lejos.utility.Matrix;

//holds 2 different odomenters.
//OdometerByGyro: calculates position by distance traveled and heading taken
//                from gyroscope.
//Odometer: calculates position by distance traveled and heading calculated by
//          motor movement.


//docs
//https://github.com/bdeneuter/lejos-ev3/blob/master/src/main/java/lejos/robotics/chassis/WheeledChassis.java

//implementation example
//https://github.com/gloomyandy/surveyor/blob/master/Surveyor/src/LineFollowerChassis.java

public class RoboChassis extends WheeledChassis {
    protected double baseDistance = 0;
    Wheel[] wheels;
    RegulatedMotor[] trackMotors;
    RegulatedMotor[] clawMotor;
    EV3GyroSensor gyro;
    Matrix reverse2;
    Matrix forward2;

	
    public RoboChassis(Wheel[] wheels) {

        super(wheels, TYPE_DIFFERENTIAL);
        forward2 = forward.copy();
        reverse2 = forward2.inverse();
        if (reverse2 == null)
            System.out.println("Reverse2 null");
        else
            System.out.println("Reverse2 created");
        this.wheels = wheels;
        trackMotors = new RegulatedMotor[wheels.length];
        for(int i = 0; i < wheels.length; i++)
        {
            trackMotors[i] = wheels[i].getMotor();
        }
	}
    public RoboChassis(Wheel[] wheels, EV3GyroSensor gyro) {
        super(wheels, TYPE_DIFFERENTIAL);
        this.wheels = wheels;
        this.gyro = gyro;
        forward2 = forward.copy();
        reverse2 = forward2.inverse();
        if (reverse2 == null)
            System.out.println("Reverse2 null");
        else
            System.out.println("Reverse2 created");
        trackMotors = new RegulatedMotor[wheels.length];
        for(int i = 0; i < wheels.length; i++)
        {
            trackMotors[i] = wheels[i].getMotor();
        }
    }

	protected synchronized Matrix getPosition() {
        Matrix m = new Matrix(trackMotors.length+dummyWheels, 1);
        master.startSynchronization();
        for (int i = 0; i < trackMotors.length; i++) {
            m.set(i, 0, ((BaseRegulatedMotor)motor[i]).getPosition());
        }
        if (dummyWheels==1) m.set(trackMotors.length, 0, 0);
        master.endSynchronization();
        return m;
    }

    private class OdometerByGyro implements PoseProvider{
        Matrix lastTacho;
        double xPose;
        double yPose;
        double aPose;
        double aBase;
        int time;

        private OdometerByGyro() {
            this.time = 64;
            this.lastTacho = getAttribute(TACHOCOUNT);
            PoseTracker tracker = new PoseTracker();
            tracker.setDaemon(true);
            tracker.start();
        }

        private double getYaw() {
            //get gyro x-axes euler vector angle in deg

        }

        @Override
        public synchronized Pose getPose() {
            return new Pose((float)this.xPose, (float)this.yPose, (float)this.aPose);
        }

        @Override
        public synchronized void setPose(Pose pose) {
            xPose = (double)pose.getX();
            yPose = (double)pose.getY();
            aPose = (double)pose.getHeading();
            aBase = getYaw() + aPose;
        }

        private synchronized void updatePoseAvg() {
            double newPose = getYaw() - aBase;
            //Matrix currentTacho = getAttribute(TACHOCOUNT);
            Matrix currentTacho = getPosition();
            Matrix delta = currentTacho.minus(lastTacho);
            while (newPose < -180)
                newPose += 360;
            while (newPose > 180)
                newPose -= 360;
            double diff =  ( ( (newPose + 180) - (aPose + 180) + 180 + 360 ) % 360 ) - 180;
            double avPose = (360 + (newPose + 180) + ( diff / 2 ) ) % 360 - 180;
            //System.out.println("ap " + aPose + " np " + newPose + " avp " + avPose);
            int max = (int) getMax(delta);

            delta = reverse.times(delta);
            //double sin = Math.sin(Math.toRadians(aPose));
            //double cos = Math.cos(Math.toRadians(aPose));
            double sin = Math.sin(Math.toRadians(avPose));
            double cos = Math.cos(Math.toRadians(avPose));
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);

            xPose += cos * x - sin * y;
            yPose += sin * x + cos * y;
            //aPose += delta.get(2, 0);

            //aPose = getYaw() - aBase;
            aPose = newPose;
            /*
            while (aPose < -180)
              aPose += 360;
            while (aPose > 180)
              aPose -= 360;
            */
            // adjust loop speed (between 4 and 64 msec);

            //
            //set loop to gyro freq.
            //
            long prevTime = time;
            if (max > 20) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            time = 64;
            //if (cnt++ % 100 == 0)
            //  System.out.println("max is " + max);
            if (prevTime != time)
                System.out.println("Set time to " + time);
            lastTacho = currentTacho;

        }


        private synchronized void updatePose() {
            Matrix currentTacho = getAttribute(TACHOCOUNT);
            double newPose = getYaw() - aBase;
            Matrix delta = currentTacho.minus(this.lastTacho);

            int max = (int) getMax(delta);

            delta = reverse.times(delta);
            double sin = Math.sin(Math.toRadians(this.aPose));
            double cos = Math.cos(Math.toRadians(this.aPose));
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);

            this.xPose += cos * x - sin * y;
            this.yPose += sin * x + cos * y;
            //aPose += delta.get(2, 0);

            //aPose = getYaw() - aBase;
            aPose = newPose;

            while (aPose < -180)
                aPose += 360;
            while (aPose > 180)
                aPose -= 360;

            // adjust loop speed (between 4 and 64 msec);
            //
            //set loop to gyro freq.
            //
            if (max > 10) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            lastTacho = currentTacho;
        }

        private synchronized void updatePosePos() {
            Matrix currentTacho = getAttribute(TACHOCOUNT);
            //double newPose = getYaw() - aBase;
            Matrix delta = currentTacho.minus(lastTacho);

            //System.out.println("ap " + aPose + " np " + newPose + " avp " + avPose);
            int max = (int) getMax(delta);

            delta = reverse.times(delta);
            double sin = Math.sin(Math.toRadians(aPose));
            double cos = Math.cos(Math.toRadians(aPose));
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);

            xPose += cos * x - sin * y;
            yPose += sin * x + cos * y;
            aPose += delta.get(2, 0);

            //aPose = getYaw() - aBase;
            //aPose = newPose;

            while (aPose < -180)
                aPose += 360;
            while (aPose > 180)
                aPose -= 360;

            // adjust loop speed (between 4 and 64 msec);
            if (max > 10) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            time = 64;
            lastTacho = currentTacho;
        }

        private class PoseTracker extends Thread {
            public void run() {
                while (true) {
                    //updatePosePos();
                    updatePoseAvg();
                    Delay.msDelay(time);
                }
            }
        }
    }

    private OdometerByGyro odometerByGyro = null;

    public PoseProvider getGyroOdomPoseProvider() {
        if (odometerByGyro == null) odometerByGyro = new OdometerByGyro();
        return  odometerByGyro;
    }


    private Odometer odometer = null;

    @Override
    public PoseProvider getPoseProvider() {
        if (odometer == null) odometer = new Odometer();
        return  odometer;
    }

    /** The odometer keeps track of the robot pose based on odometry using the encoders of the regulated motors of the wheels.
     * @author Aswin Bouwmeester
     *
     */
    private class Odometer implements PoseProvider {
        Matrix lastTacho;
        double xPose, yPose, aPose;

        int    time = 64;

        private Odometer() {
            lastTacho = getAttribute(TACHOCOUNT);
            PoseTracker tracker = new PoseTracker();
            tracker.setDaemon(true);
            tracker.start();
        }

        @Override
        public Pose getPose() {
            return new Pose((float) xPose, (float) yPose, (float) aPose);
        }

        @Override
        public synchronized void setPose(Pose pose) {
            xPose = pose.getX();
            yPose = pose.getY();
            aPose = pose.getHeading();
        }

        private synchronized void updatePose() {
            Matrix currentTacho = getAttribute(TACHOCOUNT);
            Matrix delta = currentTacho.minus(lastTacho);

            int max = (int) getMax(delta);
            delta = reverse2.times(delta);

            double newPose = aPose + delta.get(2, 0);
            while (newPose < -180)
                newPose += 360;
            while (newPose > 180)
                newPose -= 360;
            double diff =  ( ( (newPose + 180) - (aPose + 180) + 180 + 360 ) % 360 ) - 180;
            double avPose = (360 + (newPose + 180) + ( diff / 2 ) ) % 360 - 180;
            //System.out.println("ap " + aPose + " np " + newPose + " avp " + avPose);
            //double sin = Math.sin(Math.toRadians(aPose));
            //double cos = Math.cos(Math.toRadians(aPose));
            double sin = Math.sin(Math.toRadians(avPose));
            double cos = Math.cos(Math.toRadians(avPose));

        /*
        delta = reverse2.times(delta);
        double sin = Math.sin(Math.toRadians(aPose));
        double cos = Math.cos(Math.toRadians(aPose));
        */
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);

            xPose += cos * x - sin * y;
            yPose += sin * x + cos * y;
            aPose = newPose;
        /*
        aPose += delta.get(2, 0);
        while (aPose < -180)
          aPose += 360;
        while (aPose > 180)
          aPose -= 360;
        */
            // adjust loop speed (between 4 and 64 msec);
            if (max > 10) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            lastTacho = currentTacho;
        }

        private class PoseTracker extends Thread {
            public void run() {
                while (true) {
                    updatePose();
                    Delay.msDelay(time);
                }
            }
        }
    }

    public void setWheelParams(double diameter, double offset)
    {
        for (int row = 0; row < wheels.length; row++) {
            Matrix factors = wheels[row].getFactors();
            factors.set(0, 2, -(2.0 * (row > 0 ? -offset : offset) / diameter));
            forward2.setMatrix(row, row, 0, 2, factors);
        }
        reverse2 = forward2.inverse();
    }

}
