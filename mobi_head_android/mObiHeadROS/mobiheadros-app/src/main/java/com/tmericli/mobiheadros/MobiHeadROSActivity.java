package com.tmericli.mobiheadros;


import android.content.Intent;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.RelativeLayout;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

import java.util.Random;

/**
 * The main activity that instantiates individual engines to handle the
 * drawing and animation of the facial expressions, speech interaction,
 * and visual object tracking.
 *
 * @author Tekin Mericli
 * @version 1.0 2015
 */
public class MobiHeadROSActivity extends RosActivity {

    private RosCameraPreviewView rosCameraPreviewView;
    // Subscriber for the commands received from the dashboard
    private MobiHeadROSSubscriberPublisher mobiHeadROSSubscriberPublisher;

    private MoodEngine moodEngine;
    private FacialExpressionEngine facialExpressionEngine;
    private ConversationEngine conversationEngine;

    private ImageView [] sphereImages = new ImageView[facialExpressionEngine.NUM_SPHERES];

    private RelativeLayout layout;
    private RelativeLayout.LayoutParams params;

    private Handler engineUpdateHandler;
    private Handler expressionTransitionDemoHandler;
    private Handler naturalBehaviorHandler;
    private Handler expressionTransitionHandler;

    private final int updateFrequency = 20; // 20ms ~50fps
    private final int expressionTransitionFrequency = 3000;
    private final int expressionTransitionCheckFrequency = 500; // half a second

    //what the comment buttons correspond to

    public String intro1 = "This is a test of Intro 1.";
    public String intro2 = "This is a test of Intro 2.";
    public String good1 = "This is a test of Good Clue 1.";
    public String good2 = "This is a test of Good Clue 2.";
    public String good3 = "This is a test of Good Clue 3.";
    public String good4 = "This is a test of Good Clue 4.";
    public String good5 = "This is a test of Good Clue 5.";
    public String good6 = "This is a test of Good Clue 6.";
    public String good7 = "This is a test of Good Clue 7.";
    public String bad1 = "This is a test of Bad Clue 1.";
    public String bad2 = "This is a test of Bad Clue 2.";
    public String bad3 = "This is a test of Bad Clue 3.";
    public String bad4 = "This is a test of Bad Clue 4.";
    public String bad5 = "This is a test of Bad Clue 5.";
    public String bad6 = "This is a test of Bad Clue 6.";
    public String bad7 = "This is a test of Bad Clue 7.";
    public String whatsup = "After all this time, I finally do what I'm supposed to do.";


    public MobiHeadROSActivity() {
        super("mObi Head ROS", "mObi Head ROS");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        layout = (RelativeLayout) findViewById(R.id.mainlayout);

        // Keep the screen on all the time to avoid fading and closing of the app
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        final Intent callingIntent = getIntent();

        // Create the sphere image views
        for(int s = 0; s < facialExpressionEngine.NUM_SPHERES; s++) {
            sphereImages[s] = new ImageView(this);
            sphereImages[s].setImageResource(R.drawable.sphere);
            layout.addView(sphereImages[s]);
        }

        // instantiate ros camera preview view
        rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);
        // hide the preview view
        rosCameraPreviewView.setVisibility(View.INVISIBLE);

        // instantiate the mood engine
        moodEngine = new MoodEngine(this);

        // instantiate the facial expression engine
        facialExpressionEngine = new FacialExpressionEngine(this);

        // instantiate the conversation engine
        conversationEngine = new ConversationEngine(this);


        engineUpdateHandler = new Handler();
        engineUpdateHandler.post(runnable);

        // demo showing smooth transitions among various facial expressions
        /*
        expressionTransitionDemoHandler = new Handler();
        expressionTransitionDemoHandler.post(runnableExpressionTransitionDemo);
        */

        expressionTransitionHandler = new Handler();
        expressionTransitionHandler.post(runnableExpressionTransition);

        naturalBehaviorHandler = new Handler();
        //naturalBehaviorHandler.post(runnableNaturalBehavior);

        // Run the GC
        Runtime r = Runtime.getRuntime();
        r.gc();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        //NodeMain node = new SimplePublisherNode();

        // instantiate the head command subscriber
        mobiHeadROSSubscriberPublisher = new MobiHeadROSSubscriberPublisher();

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        rosCameraPreviewView.setCamera(Camera.open(Camera.getNumberOfCameras() - 1));

        //nodeMainExecutor.execute(node, nodeConfiguration);
        nodeMainExecutor.execute(mobiHeadROSSubscriberPublisher, nodeConfiguration);
        nodeMainExecutor.execute(rosCameraPreviewView, nodeConfiguration);

    }

    private Runnable runnable = new Runnable() {
        @Override
        public void run() {

            for (int s = 0; s < facialExpressionEngine.NUM_SPHERES; s++) {

                params = (RelativeLayout.LayoutParams) sphereImages[s].getLayoutParams();
                params.height = (int)facialExpressionEngine.sprites[s].getHeight();
                params.width = (int)facialExpressionEngine.sprites[s].getWidth();
                params.leftMargin = (int) facialExpressionEngine.sprites[s].getX();
                params.topMargin = (int) facialExpressionEngine.sprites[s].getY();
                sphereImages[s].setColorFilter(ColorFilterGenerator.adjustHue(facialExpressionEngine.sprites[s].getHue()));
                sphereImages[s].setLayoutParams(params);
                sphereImages[s].requestLayout();
            }

            // restart the runnable every updateFrequency milliseconds
            engineUpdateHandler.postDelayed(this, updateFrequency);
        }
    };

    /*
    private Runnable runnableExpressionTransitionDemo = new Runnable() {
        @Override
        public void run() {

            FacialExpressionEngine.Expression nextExpression = facialExpressionEngine.getNextExpression();
            facialExpressionEngine.transitionToExpression(nextExpression);

            // restart the runnable every updateFrequency milliseconds
            expressionTransitionDemoHandler.postDelayed(this, expressionTransitionFrequency);

            conversationEngine.speak(nextExpression.toString());

            //Helper.playMedia("moderateLaugh.m4a");
        }
    };
    */

    private Runnable runnableExpressionTransition = new Runnable() {
        @Override
        public void run() {

            if(mobiHeadROSSubscriberPublisher != null) {
                if (mobiHeadROSSubscriberPublisher.requestedExpression.equals("neutral")) {
                    System.out.println("**** transition to neutral facial expression ****");
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals("happy")) {
                    System.out.println("**** transition to happy facial expression ****");
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.HAPPY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals("sad")) {
                    System.out.println("**** transition to sad facial expression ****");
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.SAD);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals("surprised")) {
                    System.out.println("**** transition to surprised facial expression ****");
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.SURPRISED);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals("angry")) {
                    System.out.println("**** transition to angry facial expression ****");
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.ANGRY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals("inlove")) {
                    System.out.println("**** transition to inlove facial expression ****");
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.INLOVE);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals("shy")) {
                    System.out.println("**** transition to shy facial expression ****");
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.SHY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                }

                //INTROS


                else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(intro1)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.HAPPY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(intro2)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.SURPRISED);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                }

                //GOOD CLUES


                else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(good1)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(good2)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(good3)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.HAPPY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(good4)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(good5)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.ANGRY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(good6)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.SURPRISED);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(good7)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.HAPPY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                }


                //BAD CLUES


                else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(bad1)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(bad2)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(bad3)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.HAPPY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(bad4)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(bad5)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.ANGRY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(bad6)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.SURPRISED);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                } else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(bad7)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.HAPPY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                }

                //PLACEHOLDER

                else if (mobiHeadROSSubscriberPublisher.requestedExpression.equals(whatsup)) {
                    System.out.println("**** transition to shy facial expression ****");
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.HAPPY);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                }


                //CUSTOM PHRASE

                if(!mobiHeadROSSubscriberPublisher.requestedExpression.equals((""))) {
                    conversationEngine.speak(mobiHeadROSSubscriberPublisher.requestedExpression);
                    mobiHeadROSSubscriberPublisher.requestedExpression = "";
                }
            }

            // restart the runnable every updateFrequency milliseconds
            expressionTransitionHandler.postDelayed(this, expressionTransitionCheckFrequency);

            //Helper.playMedia("moderateLaugh.m4a");
        }
    };

    private Runnable runnableNaturalBehavior = new Runnable() {
        @Override
        public void run() {

            System.out.println("natural behavior");
            if(new Random().nextDouble() > 0.5) {
                facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.BLINK);
            }
            else {
                facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.NEUTRAL);
            }

            // restart the runnable randomly between 5 and 10 seconds
            naturalBehaviorHandler.postDelayed(this, new Random().nextInt(5000) + 5000);
        }
    };
}
