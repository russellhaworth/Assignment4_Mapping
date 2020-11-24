package sensors;

import lejos.hardware.Button;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.UARTPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;
import lejos.utility.Matrix;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

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
    protected OdometerByGyro odometerByGyro;
    protected Odometer odometer;
    protected SampleProvider gyroProvider;
    protected EV3GyroSensor gyro;
    protected Wheel[] wheels;
    protected RegulatedMotor[] motors;
    protected Matrix reverse2;
    protected Matrix forward2;
    protected double baseDistance = 0;
    FileOutputStream out;
    DataOutputStream dataOut;


    public RoboChassis(Wheel[] wheels) {
        super(wheels, TYPE_DIFFERENTIAL);
        forward2 = forward.copy();
        reverse2 = forward2.inverse();
        if (reverse2 == null)
            System.out.println("Reverse2 null");
        else
            System.out.println("Reverse2 created");
        this.wheels = wheels;
        motors = new RegulatedMotor[wheels.length];
        for(int i = 0; i < wheels.length; i++)
        {
            motors[i] = wheels[i].getMotor();
        }
        odometerByGyro = null;
        odometer = null;
        this.gyro = null;
        this.dataOut = null;
	}

    public RoboChassis(Wheel[] wheels, EV3GyroSensor gyro) {
        this(wheels);
        this.gyro = gyro;
        this.gyroProvider = gyro.getAngleMode();
    }

    public RoboChassis(Wheel[] wheels, EV3GyroSensor gyro, FileOutputStream fio) {
        this(wheels, gyro);
        this.out = fio;
        this.dataOut = new DataOutputStream(out);
    }

    public void writePose(Pose p, String name) {
        if (!dataOut.equals(null)) {
            try{
                String s = String.format("%d, %s: (%.2f, %.2f) at %f.2 degrees.\n", System.currentTimeMillis(), name, p.getX(), p.getY(), p.getHeading());
                dataOut.writeChars(s);
            }catch(IOException e){
                System.err.println("Failed to write to output stream");
                Button.waitForAnyPress();
                System.exit(1);
            }
        }
    }

    public void writeString(String data) {
        if (!dataOut.equals(null)) {
            try{
                String str = String.format("%f: %d\n", System.currentTimeMillis(), data);
                dataOut.writeChars(str);
            }catch(IOException e){
                System.err.println("Failed to write to output stream");
                Button.waitForAnyPress();
                System.exit(1);
            }
        }
    }


    public RoboChassis(Wheel[] wheels, EV3GyroSensor gyro, int i) {
        this(wheels);
    }

    private class GyroTracker extends EV3GyroSensor {

        public GyroTracker(Port port) {
            super(port);
        }

        public GyroTracker(UARTPort port) {
            super(port);
        }
    }
    public Matrix getForward() {
        return this.forward.copy();
    }

    public Matrix getReverse() {
        return this.reverse.copy();
    }


    private class Odometer implements PoseProvider {
        Matrix lastTacho;
        double xPose, yPose, aPose;
        int time;

        private Odometer() {
            this.time = 64;
            this.lastTacho = getAttribute(0);
            PoseTracker tracker = new PoseTracker();
            tracker.setDaemon(true);
            tracker.start();
        }

        @Override
        public Pose getPose() {
            return new Pose((float)this.xPose, (float)this.yPose, (float)this.aPose);
        }

        @Override
        public synchronized void setPose(Pose pose) {
            this.xPose = (double)pose.getX();
            this.yPose = (double)pose.getY();
            this.aPose = (double)pose.getHeading();
        }

        private synchronized void updatePose() {
            Matrix currentTacho = getAttribute(0);
            Matrix delta = currentTacho.minus(this.lastTacho);
            int max = (int)getMax(delta);
            delta = reverse.times(delta);
            double sin = Math.sin(Math.toRadians(this.aPose));
            double cos = Math.cos(Math.toRadians(this.aPose));
            double x = delta.get(0, 0);
            double y = delta.get(1, 0);
            this.xPose += cos * x - sin * y;
            this.yPose += sin * x + cos * y;

            for(this.aPose += delta.get(2, 0); this.aPose < 180.0D; this.aPose += 360.0D) {
            }

            while(this.aPose > 180.0D) {
                this.aPose -= 360.0D;
            }

            if (max > 10) {
                this.time /= 2;
            }

            if (max < 10) {
                this.time *= 2;
            }

            this.time = Math.max(Math.min(this.time, 64), 4);
            if (!lastTacho.equals(currentTacho)){
                writePose(getPose(), "     Odom");
            }
            this.lastTacho = currentTacho;
        }

        private class PoseTracker extends Thread {
            private PoseTracker() {
            }

            public void run() {
                while(true) {
                    Odometer.this.updatePose();
                    Delay.msDelay((long) Odometer.this.time);
                }
            }
        }
    }

    private class OdometerByGyro implements PoseProvider{
        Matrix lastTacho;
        double xPose, yPose, aPose, aBase;
        int time = 64;

        private OdometerByGyro() {
            this.lastTacho = getAttribute(TACHOCOUNT);
            PoseTracker tracker = new PoseTracker();
            tracker.setDaemon(true);
            tracker.start();
        }

        private double getYaw() {
            //get gyro x-axes euler vector angle in deg
            float [] sample = new float[gyroProvider.sampleSize()];
            gyroProvider.fetchSample(sample, gyroProvider.sampleSize());
            return sample[0];
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
            long prevTime = time;
            if (max > 20) time=time / 2;
            if (max < 10) time=time * 2;
            time = Math.max(Math.min(time, 64), 4);
            time = 64;
            //if (cnt++ % 100 == 0)
            //  System.out.println("max is " + max);
            if (prevTime != time)
                System.out.println("Set time to " + time);
            if (!lastTacho.equals(currentTacho)){
                writePose(getPose(), "Gyro Odom");
            }
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
            if (!lastTacho.equals(currentTacho)){
                writePose(getPose(), "Gyro Odom");
            }
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
            if (!lastTacho.equals(currentTacho)){
                writePose(getPose(), "Gyro Odom");
            }
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

    @Override
    public PoseProvider getPoseProvider() {
        if (odometer == null) odometer = new Odometer();
        return  odometer;
    }
    public PoseProvider getGyroOdomPoseProvider() {
        if (odometerByGyro == null) odometerByGyro = new OdometerByGyro();
        return  odometerByGyro;
    }
	protected synchronized Matrix getPosition() {
        Matrix m = new Matrix(motors.length+dummyWheels, 1);
        master.startSynchronization();
        for (int i = 0; i < motors.length; i++) {
            m.set(i, 0, ((BaseRegulatedMotor)motor[i]).getPosition());
        }
        if (dummyWheels==1) m.set(motors.length, 0, 0);
        master.endSynchronization();
        return m;
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
