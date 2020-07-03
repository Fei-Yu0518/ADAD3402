 private class MyFluidData implements DwFluid2D.FluidData{
    
    // update() is called during the fluid-simulation update step.
    @Override
    public void update(DwFluid2D fluid) {
    //get the skeletons as an Arraylist of KSkeletons
    ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonDepthMap();
      float plx, ply, prx, pry, vx, vy, radius, vscale, temperature;
        for (int i = 0; i < skeletonArray.size(); i++) {
          KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
        //if the skeleton is being tracked compute the skleton joints
        if (skeleton.isTracked()) {
          KJoint[] joints = skeleton.getJoints();
          vscale = 15;
          plx = map(joints[KinectPV2.JointType_HandTipLeft].getX(), 0, kinect.getDepthImage().width, 0, width);
          ply = height - map(joints[KinectPV2.JointType_HandTipLeft].getY(), 0, kinect.getDepthImage().height, 0, height);
          prx = map(joints[KinectPV2.JointType_HandTipRight].getX(), 0, kinect.getDepthImage().width, 0, width);
          pry = height - map(joints[KinectPV2.JointType_HandTipRight].getY(), 0, kinect.getDepthImage().height, 0, height);
          //println(px);
          vx = random(-3.5, 3.5) * +vscale; // note the random(-5, 5) produces a number 
          vy = random(-3.5, 3.5) * -vscale;
          radius = 8;
          fluid.addDensity(plx, ply, radius, 1, 1, 1f, 1.0f);
          fluid.addDensity(prx, pry, radius, 1, 1, 1f, 1.0f);
          radius = 15;
          fluid.addVelocity(plx, ply, radius, vx, vy);
          fluid.addVelocity(prx, pry, radius, vx, vy);
          
        }  
    }
  }    
}    
