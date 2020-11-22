package main;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;

//docs
//https://github.com/bdeneuter/lejos-ev3/blob/master/src/main/java/lejos/robotics/chassis/WheeledChassis.java

//implementation example
//https://github.com/gloomyandy/surveyor/blob/master/Surveyor/src/MapTracker.java

public class MapTracker implements LineTracker
{
    public static enum PlanState{NONE, DIRECT, PLAN};
    final static float[] speedMult = {1.0f, 0.75f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    PoseProvider gyroPose;
    PoseProvider odomPose;
    public Pose targetPose = new Pose();
    float heading = 0.0f;
    public float targetSpeed = 0.0f;
    public PlanState planState;

    //used in slam.
    Pose deltaPose = new Pose();

    public MapTracker(RoboChassis chassis)
    {
        gyroPose = chassis.getGyroOdomPoseProvider();
        odomPose = chassis.getPoseProvider();
    }

    @Override
    public int sampleSize()
    {
        return 2;
    }

    @Override
    public synchronized void fetchSample(float[] sample, int offset)
    {
        Pose slamPose = addPose(gyroPose.getPose(), deltaPose);
        //System.out.println("IMU heading " + imuPose.getPose().getHeading());
        //float error = normalize(heading - imuPose.getPose().getHeading());
        //System.out.println("target " + targetPose + " imu " + imuPose.getPose() + " slam " + slamPose);
        float error = 0;
        switch(planState)
        {
            case NONE:
                break;
            case DIRECT:
                error = normalize(slamPose.angleTo(targetPose.getLocation()) - slamPose.getHeading());
                break;
            case PLAN:
                error = normalize(heading - slamPose.getHeading());
                break;
        }
        if (planState == PlanState.NONE)
        {
            sample[offset] = 0.0f;
            sample[offset+1] = 0.0f;
        }
        else
        {
            //try and find heuristic to choose pose and offset from
            // either gyro or odom pose.

            //slam
            /*
            sample[offset] = error/100;
            if (slamPose.distanceTo(targetPose.getLocation()) < 100)
            {
                sample[offset+1] = 0.0f;
                planState = PlanState.NONE;
            }
            else
                sample[offset+1] = targetSpeed*speedMult[(int)(Math.abs(error)/22.5f)];
             */
        }
        System.out.printf("Error %f speed %f\n", sample[0], sample[1]);
    }

    @Override
    public boolean startCalibration()    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void stopCalibration()    {
        // TODO Auto-generated method stub

    }

    @Override
    public void calibrate()    {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean marker()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void close()    {
        // TODO Auto-generated method stub

    }

    @Override
    public void setEdge(int edge)    {
        // TODO Auto-generated method stub

    }

    public void setHeading(float newHeading)   {
        heading = newHeading;
    }

    protected float normalize(float h) {
        while (h < -180) h += 360;
        while (h > 180) h -= 360;
        return h;
    }

    protected Pose poseDelta(Pose op, Pose np)  {
        Pose ret = new Pose();
        ret.setLocation(np.getX() - op.getX(), np.getY() - op.getY());
        ret.setHeading(normalize(np.getHeading() - op.getHeading()));
        return ret;
    }

    protected Pose addPose(Pose op, Pose np) {
        Pose ret = new Pose();
        ret.setLocation(np.getX() + op.getX(), np.getY() + op.getY());
        ret.setHeading(normalize(np.getHeading() + op.getHeading()));
        return ret;

    }

    public synchronized void setTarget(PlanState state, Pose newTargetPose, float speed, float heading)
    {
        if (state == PlanState.NONE)
        {
            if (planState != PlanState.NONE)
            {
                targetSpeed = 0;
                targetPose = new Pose();
                planState = PlanState.NONE;
            }
            return;
        }

        if (newTargetPose.getX() != targetPose.getX() || newTargetPose.getY() != targetPose.getY() || newTargetPose.getHeading() != targetPose.getHeading())
        {
            Pose pnew = new Pose(newTargetPose.getX(), newTargetPose.getY(), newTargetPose.getHeading());
            targetPose = pnew;
            planState = state;
        }
        targetSpeed = speed;
        this.heading = heading;
    }

    public synchronized void setPoseDelta(Pose p) {
        //slam
        Pose pnew = new Pose(p.getX(), p.getY(), p.getHeading());
        deltaPose = pnew;
        System.out.println("new delta " + pnew);
    }

    public synchronized Pose getSlamPose() {
        //slam
        return addPose(gyroPose.getPose(), deltaPose);
    }

}

