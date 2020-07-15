/*
Thomas Sanchez Lengeling.
 http://codigogenerativo.com/

 KinectPV2, Kinect for Windows v2 library for processing

 Skeleton color map example.
 Skeleton (x,y) positions are mapped to match the color Frame
 */

import KinectPV2.KJoint;
import KinectPV2.*;
import processing.video.Capture;
KinectPV2 kinect;
Capture video;

void setup() {
  size(1920, 1080, P3D);

  kinect = new KinectPV2(this);

  kinect.enableSkeletonColorMap(true);
  kinect.enableColorImg(true);

  kinect.init();
    // This the default video input, see the GettingStartedCapture 
  // example if it creates an error
  video = new Capture(this, 640, 480);
  
  // Start capturing the images from the camera
  video.start();  
  
  background(0);
}

void draw() {
  image(kinect.getColorImage(), 0, 0, 320, 240);
  if (video.available()) {
    video.read();

    ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();

  //individual JOINTS
    for (int i = 0; i < skeletonArray.size(); i++) {
      KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
      if (skeleton.isTracked()) {
        KJoint[] joints = skeleton.getJoints();

        color col  = skeleton.getIndexColor();
        fill(col);
        stroke(col);
        drawBody(joints);

        //draw different color for each hand state
        drawHandState(joints[KinectPV2.JointType_HandRight]);
        drawHandState(joints[KinectPV2.JointType_HandLeft]);
      }
    }

    fill(255, 0, 0);
    text(frameRate, 50, 50);
  }
}

//DRAW BODY
void drawBody(KJoint[] joints) {

  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);

  // Left Arm

  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);

}


//draw bone
void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  pushMatrix();
  translate(joints[jointType1].getX(), joints[jointType1].getY(), joints[jointType1].getZ());
  ellipse(0, 0, 25, 25);
  popMatrix();
  line(joints[jointType1].getX(), joints[jointType1].getY(), joints[jointType1].getZ(), joints[jointType2].getX(), joints[jointType2].getY(), joints[jointType2].getZ());
}

//draw hand state
void drawHandState(KJoint joint) {
  noStroke();
  handState(joint.getState());
  pushMatrix();
  translate(joint.getX(), joint.getY(), joint.getZ());
  ellipse(0, 0, 70, 70);
  popMatrix();
}

/*
Different hand state
 KinectPV2.HandState_Open
 KinectPV2.HandState_Closed
 KinectPV2.HandState_Lasso
 KinectPV2.HandState_NotTracked
 */
void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    fill(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    fill(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    fill(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    fill(255, 255, 255);
    break;
  }
}
