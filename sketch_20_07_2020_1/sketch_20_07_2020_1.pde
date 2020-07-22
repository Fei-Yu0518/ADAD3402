/*
Thomas Sanchez Lengeling.
 http://codigogenerativo.com/

 KinectPV2, Kinect for Windows v2 library for processing

 Skeleton depth tracking example
 
 Fluid_StreamLines is an experiment, to visualize streamlines of the fluid.
 At regular gridpoints polylines are placed and their vertices are aligned 
 to the fluid vectors.

 This is pretty critical for performance. So the real challenge is to handle
 the amount of data to render but even more its realtime updates.
 A software solution turned out to be way to insufficient.

 The trick is, to allocate one ping-pong-texture, GL_RG32F, that stores
 for each line one vertex position (x,y).
 
 The rendering pass is then an iterative execution of an update/render step.

 e.g. 500 x 500 streamlines a 30 vertices:
 the ping-pong-texture has a size of 500 x 500.
 the update/render-step is executed 30 times, each frame.


 controls:

 LMB: add Velocity
 RMB: add Density
  
 */
//import java.io.FilenameFilter;
//import processing.sound.*;
import java.awt.Rectangle;
import java.util.Map;
import java.util.Iterator;

import com.thomasdiewald.pixelflow.java.DwPixelFlow;
import com.thomasdiewald.pixelflow.java.fluid.DwFluid2D;
import com.thomasdiewald.pixelflow.java.fluid.DwFluidStreamLines2D;

import controlP5.Accordion;
import controlP5.ControlP5;
import controlP5.Group;
import controlP5.RadioButton;
import controlP5.Toggle;
import processing.core.*;
import processing.opengl.PGraphics2D;
//import processing.video.Capture;

import java.util.ArrayList;
import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;
int viewport_w = 1920;
int viewport_h = 1080;
int viewport_x = 100;
int viewport_y = 0;

int gui_w = 200;
int gui_x = 20;
int gui_y = 280;
int jointType1;
int jointType2;
int fluidgrid_scale = 1;



// library
DwPixelFlow context;

// Fluid simulation
DwFluid2D fluid;

// streamline visualization
DwFluidStreamLines2D streamlines;

// render targets
PGraphics2D pg_fluid;
//texture-buffer, for adding obstacles
PGraphics2D pg_obstacles;

// some state variables for the GUI/display
int     BACKGROUND_COLOR           = 0;
boolean UPDATE_FLUID               = true;
boolean DISPLAY_FLUID_TEXTURES     = true;
boolean DISPLAY_FLUID_VECTORS      = !true;
int     DISPLAY_fluid_texture_mode = 0;
boolean DISPLAY_STREAMLINES        = true;
int     STREAMLINE_DENSITY         = 10;
float   OFFSET                     = 1.0; // if using Kinect V2 use keys 4/5 to toggle
int handVecListSize = 20;
float[] fluid_velocity;
//Capture video;
Map<Integer, ArrayList<PVector>>  
handPathList = new HashMap<Integer, ArrayList<PVector>>();

  public void settings() {
    size(viewport_w, viewport_h, P3D);
    //fullScreen(P2D);
    smooth(2); //8
  }

  public void setup() {
    //noCursor();
    //surface.setLocation(viewport_x, viewport_y);
    
    // main library context
    context = new DwPixelFlow(this);
    context.print();
    context.printGL();

  kinect = new KinectPV2(this);

  //Enables depth and Body tracking (image)
  kinect.enableDepthImg(true);
  kinect.enableSkeletonDepthMap(true);

  kinect.enableSkeletonColorMap(true);
  kinect.enableColorImg(true);

  kinect.init();
   // visualization of the velocity field
    streamlines = new DwFluidStreamLines2D(context);
    
    // fluid simulation
    fluid = new DwFluid2D(context, viewport_w, viewport_h, fluidgrid_scale);
    
    // set some simulation parameters
    fluid.param.dissipation_density     = 0.999f;
    fluid.param.dissipation_velocity    = 0.99f;
    fluid.param.dissipation_temperature = 0.50f;
    fluid.param.vorticity               = 0.10f;
    
    
    // interface for adding data to the fluid simulation
    MyFluidData cb_fluid_data = new MyFluidData();
    fluid.addCallback_FluiData(cb_fluid_data);
   

    // pgraphics for fluid
    pg_fluid = (PGraphics2D) createGraphics(viewport_w, viewport_h, P2D);
    pg_fluid.smooth(2); //4
    
    // This the default video input, see the GettingStartedCapture 
    // example if it creates an error
    //video = new Capture(this, 640, 480);
  
    // Start capturing the images from the camera
    //video.start();  
  
    
    
    createGUI(); //call GUI
    background(0);
    frameRate(60);
//    frameRate(120);
  }


void draw() {
 
  ////get the skeletons as an Arraylist of KSkeletons
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonDepthMap();

  //individual joints
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    //if the skeleton is being tracked compute the skleton joints
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      color col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);

      drawBody(joints);
      //drawHandState(joints[KinectPV2.JointType_HandRight]);
      //drawHandState(joints[KinectPV2.JointType_HandLeft]);
    }
  }

 
      // update simulation
    if(UPDATE_FLUID){
      //fluid.addObstacles(pg_obstacles);
      fluid.update();
      println("update set called" +millis());
    }
    
    // clear render target
    pg_fluid.beginDraw();
    pg_fluid.background(BACKGROUND_COLOR); // this is conflicying with the render point to standard gxf
    pg_fluid.endDraw();
    
    // render fluid stuff
    if(DISPLAY_FLUID_TEXTURES){
      // render: density (0), temperature (1), pressure (2), velocity (3)
      fluid.renderFluidTextures(pg_fluid, DISPLAY_fluid_texture_mode);
    }
    
    if(DISPLAY_FLUID_VECTORS){
      // render: velocity vector field
      fluid.renderFluidVectors(pg_fluid, 10);
    }

    if(DISPLAY_STREAMLINES){
      streamlines.render(pg_fluid, fluid, STREAMLINE_DENSITY);
    }
    

    // RENDER
    // display textures
    image(pg_fluid, 0, 0);
    //image(pg_obstacles, 0, 0);
    image(kinect.getDepthImage(), 0, 0, 1920, 1080);
    //image(kinect.getColorImage(), 0, 0, 1920, 1080);

    // info
    String txt_fps = String.format(getClass().getName()+ "   [size %d/%d]   [frame %d]   [fps %6.2f]", fluid.fluid_w, fluid.fluid_h, fluid.simulation_step, frameRate);
    surface.setTitle(txt_fps);
}

//draw the body
void drawBody(KJoint[] joints) {
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
}


//draw a bone from two joints
void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  pushMatrix();
  translate(map(joints[jointType2].getX(), 0, kinect.getDepthImage().width, 0, width), map(joints[jointType2].getY(), 0, kinect.getDepthImage().height, 0, height), joints[jointType2].getZ());
  //translate(map(joints[jointType1].getX(), 0, 320, 0, width), map(joints[jointType1].getY(), 0, 240, 0, height), joints[jointType1].getZ());
  //println(map(joints[jointType1].getX(), 0, kinect.getDepthImage().width, 0, width));
  //fill(0, 255, 0);
  ellipse(0, 0, 100, 100);
  
  popMatrix();
  //line(joints[jointType1].getX(), joints[jointType1].getY(), joints[jointType1].getZ(), joints[jointType2].getX(), joints[jointType2].getY(), joints[jointType2].getZ());
}

//draw a ellipse depending on the hand state
void drawHandState(KJoint joint) {
  noStroke();
  handState(joint.getState());
  pushMatrix();
  translate(joint.getX(), joint.getY(), joint.getZ());
  //ellipse(0, 0, 70, 70);
  popMatrix();
}

/*
Different hand state
 KinectPV2.HandState_Open
 KinectPV2.HandState_Closed
 KinectPV2.HandState_Lasso
 KinectPV2.HandState_NotTracked
 */

//Depending on the hand state change the color
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
    fill(100, 100, 100);
    break;
  }
}
