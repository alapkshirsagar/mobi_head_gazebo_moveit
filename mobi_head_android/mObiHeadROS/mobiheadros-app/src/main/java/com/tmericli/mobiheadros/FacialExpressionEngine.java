package com.tmericli.mobiheadros;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.animation.AnimatorSet;
import android.animation.ObjectAnimator;
import android.animation.PropertyValuesHolder;
import android.animation.TimeInterpolator;
import android.content.Context;
import android.util.DisplayMetrics;
import android.view.animation.AccelerateDecelerateInterpolator;
import android.view.animation.AnticipateOvershootInterpolator;
import android.view.animation.BounceInterpolator;
import android.view.animation.LinearInterpolator;
import android.view.animation.OvershootInterpolator;

import java.util.ArrayList;
import java.util.List;

/**
 * FacialExpressionEngine class defines different facial expressions and
 * handles the transitions between them with proper animations.
 *
 * @author Tekin Mericli
 * @version 1.0 2015
 */
public class FacialExpressionEngine {

    public enum Expression {
        NEUTRAL,
        SURPRISED,
        HAPPY,
        SAD,
        ANGRY,
        //CONFUSED,
        INLOVE,
        SHY,
        BLINK;

        private static String[] expressionNames = {"Neutral", "Surprised", "Happy", "Sad", "Angry", "In love", "Shy", "Blink"};

        private static Expression[] vals = values();

        public Expression next()
        {
            return vals[(this.ordinal()+1) % vals.length];
        }

        public String toString() {
            return expressionNames[this.ordinal()];
        }
    }


    // coordinate encodings of various facial expressions

    private float [] neutralX = {-7.7675347f, -5.9880867f, -2.8368402f, 0.96429396f, 4.54452f, 7.08365f, 8.0f, 7.083648f, 4.544518f, 0.964293f, -2.8368387f, -5.988086f, -7.7675347f};
    private float [] neutralY = { 1.9145254f, 5.3049808f, 7.4801297f, 7.941671f, 6.58387f, 3.7177823f, 0.0f, -3.7177854f, -6.583871f, -7.941671f, -7.48013f, -5.304981f, -1.9145249f};

    private float [] surprisedX = {-8.544f, -6.586f, -3.120f, 1.060f, 4.998f, 7.792f, 8.8f, 7.792f, 4.998f, 1.060f, -3.120f, -6.586f, -8.544f};
    private float [] surprisedY = {2.105f, 5.835f, 8.228f, 8.735f, 7.242f, 4.089f, 0.0f, -4.089f, -7.242f, -8.735f, -8.228f, -5.835f, -2.105f};

    private float [] happyX = {-7.2f, -6.5f, -5.0f, -2.8f, 0.0f, 2.8f, 5.0f, 6.5f, 7.2f, 4.3f, 1.4f, -1.4f, -4.3f};
    private float [] happyY = {1.7f, 3.8f, 5.4f, 6.4f, 6.7f, 6.4f, 5.4f, 3.8f, 1.7f, 3.0f, 3.5f, 3.5f, 3.0f};

    private float [] sadX = {-6.3f, -4.4f, -2.2f, -0.6f, 1.2f, 3.1f, 5.0f, 7.1f, 3.4f, 0.0f, -2.7f, -5.0f, -6.5f};
    private float [] sadY = {5.4f, 6.4f, 5.3f, 3.4f, 1.8f, 0.3f, -1.0f, -2.3f, -2.6f, -2.1f, -0.6f, 0.9f, 3.0f};

    private float [] angryX = {-6.5f, -5.5f, -3.5f, -1.3f, 0.8f, 3.5f, 6.3f, 4.7f, 2.6f, 0.8f, -1.0f, -3.7f, -6.1f};
    private float [] angryY = {-1.3f, 0.5f, 2.2f, 3.4f, 4.1f, 4.6f, 4.7f, 2.9f, 1.5f, -0.4f, -2.2f, -3.3f, -3.2f};

    private float [] inLoveX = {-7.5f, -6.5f, -3.5f, 0.0f, 3.5f, 6.5f, 7.5f, 6.0f, 2.8f, 0.0f, 0.0f, -2.8f, -6.0f};
    private float [] inLoveY = {1.8f, 4.8f, 6.2f, 4.7f, 6.2f, 4.8f, 1.8f, -1.2f, -4.3f, -6.5f, 0.2f, -4.3f, -1.2f};

    private float [] inLovePulseX = {-9.0f, -7.8f, -4.2f, 0.0f, 4.2f, 7.8f, 9.0f, 7.2f, 3.36f, 0.0f, 0.0f, -3.36f, -7.2f};
    private float [] inLovePulseY = {2.16f, 5.76f, 7.44f, 5.64f, 7.44f, 5.76f, 2.16f, -1.44f, -5.16f, -7.8f, 0.24f, -5.16f, -1.44f};

    //private float [] blinkX = {-8.2f, -5.6f, -2.9f, 0.0f, 2.9f, 5.6f, 8.2f, 7.1f, 4.5f, 1.6f, -1.6f, -4.5f, -7.1f};
    //private float [] blinkY = { 1.1f, 0.43f, 0.0f, -0.32f, 0.0f, 0.43f, 1.1f, -0.1f, -1.3f, -1.6f, -1.6f, -1.3f, -0.1f};

    private float [] shyX = {-5.825f, -4.491f, -2.127f, 0.723f, 3.408f, 5.312f, 6.0f, 5.312f, 3.408f, 0.723f, -2.127f, -4.491f, -5.825f};
    private float [] shyY = {1.435f, 3.978f, 5.610f, 5.956f, 4.937f, 2.788f, 0.0f, -2.788f, -4.937f, -5.956f, -5.610f, -3.978f, -1.435f};

    private float [] blinkX = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    private float [] blinkY = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // hue values for different expressions
    private float neutralHue = 90.0f;
    private float surprisedHue = 30.0f;
    private float happyHue = 0.0f; // since the original spheres are already green
    private float sadHue = 130.0f;
    private float angryHue = -100.0f;
    private float inLoveHue = -120.0f;
    private float shyHue = 180.0f;
    private float blinkHue = neutralHue; // assuming the blinking animation will be performed when the expression is neutral

    public static final int SPRITE_WIDTH = 40;
    public static final int SPRITE_HEIGHT = 40;

    public static final int NUM_EYES = 2;
    public static final int NUM_SPHERES = 26;
    public static final int NUM_SPHERES_PER_EYE = NUM_SPHERES / NUM_EYES;

    private static final int COORD_SCALING_FACTOR = 20;

    private final int NEUTRAL_EXPRESSION_EYE_RADIUS = (int)(COORD_SCALING_FACTOR * (Helper.findMax(neutralY) - Helper.findMin(neutralY)) / 2);

    Context mContext;
    DisplayMetrics displayMetrics;

    private int screenWidth;
    private int screenHeight;

    // interpolators for the animations
    private AccelerateDecelerateInterpolator accelerateDecelerateInterpolator = new AccelerateDecelerateInterpolator();
    private LinearInterpolator linearInterpolator = new LinearInterpolator();
    private OvershootInterpolator overshootInterpolator = new OvershootInterpolator();
    private BounceInterpolator bounceInterpolator = new BounceInterpolator();
    private AnticipateOvershootInterpolator anticipateOvershootInterpolator = new AnticipateOvershootInterpolator();

    public Renderable [] sprites = new Renderable[NUM_SPHERES];
    private int spriteIndex = 0;

    private static int expressionIndex = 0;
    private static int transitionCounter = 0;

    private Expression currentExpression;
    private boolean currentExpressionAdditionalAnimationPerformed = false;

    public FacialExpressionEngine(Context context) {
        mContext = context;

        displayMetrics = mContext.getResources().getDisplayMetrics();

        this.screenWidth = displayMetrics.widthPixels;
        this.screenHeight = displayMetrics.heightPixels;

        // for each eye
        for(int e = 0; e < NUM_EYES; e++) {
            // for each sphere in each eye
            for (int s = 0; s < NUM_SPHERES_PER_EYE; s++) {
                spriteIndex = e * NUM_SPHERES_PER_EYE + s;

                sprites[spriteIndex] = new Renderable();
                // initialize the spheres as in blink expression
                sprites[spriteIndex].setX((e == 0 ? -1 : 1) * blinkX[s] * COORD_SCALING_FACTOR + (e == 0 ? 0.25f * screenWidth : 0.75f * screenWidth) - SPRITE_WIDTH / 2);
                sprites[spriteIndex].setY(blinkY[s] * COORD_SCALING_FACTOR + screenHeight / 2 - SPRITE_HEIGHT / 2);
            }
        }

        //transitionToExpression(Expression.HAPPY);
        transitionToExpression(Expression.NEUTRAL);
        //transitionToExpression(Expression.BLINK);
        //transitionToExpression(Expression.INLOVE);
        //giggle(happyY, 1250);
    }


    public void transitionToExpression(Expression expression) {
        currentExpression = expression;

        switch (expression) {
            case NEUTRAL:
                transitionToExpression(neutralX, neutralY, SPRITE_HEIGHT, SPRITE_WIDTH, neutralHue, 300, 0, false, accelerateDecelerateInterpolator);

                break;

            case SURPRISED:
                transitionToExpression(surprisedX, surprisedY, SPRITE_HEIGHT * 1.2f, SPRITE_WIDTH * 1.2f, surprisedHue, 600, 0, false, bounceInterpolator);

                break;

            case HAPPY:
                transitionToExpression(happyX, happyY, SPRITE_HEIGHT, SPRITE_WIDTH, happyHue, 300, 0, false, accelerateDecelerateInterpolator);

                break;

            case SAD:
                transitionToExpression(sadX, sadY, SPRITE_HEIGHT, SPRITE_WIDTH, sadHue, 500, 0, false, accelerateDecelerateInterpolator);

                break;

            case ANGRY:
                transitionToExpression(angryX, angryY, SPRITE_HEIGHT, SPRITE_WIDTH, angryHue, 500, 0, false, accelerateDecelerateInterpolator);

                break;

            case INLOVE:
                transitionToExpression(inLoveX, inLoveY, SPRITE_HEIGHT * 1.25f, SPRITE_WIDTH * 1.25f, inLoveHue, 300, 0, false, accelerateDecelerateInterpolator);
                currentExpressionAdditionalAnimationPerformed = false;

                break;

            case SHY:
                transitionToExpression(shyX, shyY, SPRITE_HEIGHT * 0.8f, SPRITE_WIDTH * 0.8f, shyHue, 300, 0, false, accelerateDecelerateInterpolator);

                break;

            case BLINK:
                transitionToExpression(blinkX, blinkY, SPRITE_HEIGHT, SPRITE_WIDTH, blinkHue, 100, 1, true, accelerateDecelerateInterpolator);

                break;

            default:

                break;
        }
    }

    public void transitionToExpression(float [] expX, float [] expY, float expH, float expW, float expHue, long duration, int repeatCount, boolean reverse, TimeInterpolator interpolator) {
        AnimatorSet animatorSet = new AnimatorSet();
        List<Animator> animatorList = new ArrayList<Animator>();

        PropertyValuesHolder pvhX, pvhY, pvhH, pvhW, pvhHue;
        ObjectAnimator animator;

        animatorSet.addListener(new AnimatorListenerAdapter() {
            @Override
            public void onAnimationEnd(Animator animation) {

                // if the current expression is INLOVE, perform the pulsation animation
                if(currentExpression == Expression.INLOVE && !currentExpressionAdditionalAnimationPerformed) {
                    transitionToExpression(inLovePulseX, inLovePulseY, SPRITE_HEIGHT * 1.5f, SPRITE_WIDTH * 1.5f, inLoveHue, 500, 3, true, accelerateDecelerateInterpolator);

                    currentExpressionAdditionalAnimationPerformed = true;
                }
            }

        });

        for (int e = 0; e < NUM_EYES; e++) {
            for (int s = 0; s < NUM_SPHERES_PER_EYE; s++) {
                pvhX = PropertyValuesHolder.ofFloat("x", (e == 0 ? -1 : 1) * expX[s] * COORD_SCALING_FACTOR + (e == 0 ? 0.25f * screenWidth : 0.75f * screenWidth) - SPRITE_WIDTH / 2);
                pvhY = PropertyValuesHolder.ofFloat("y", screenHeight - (expY[s] * COORD_SCALING_FACTOR + screenHeight / 2 + SPRITE_HEIGHT / 2));

                pvhH = PropertyValuesHolder.ofFloat("width", expW);
                pvhW = PropertyValuesHolder.ofFloat("height", expH);
                pvhHue = PropertyValuesHolder.ofFloat("hue", expHue);

                animator = ObjectAnimator.ofPropertyValuesHolder(sprites[s + e * NUM_SPHERES_PER_EYE], pvhX, pvhY, pvhH, pvhW, pvhHue);

                animator.setDuration(duration);
                animator.setInterpolator(interpolator);
                animator.setRepeatCount(repeatCount);

                if (reverse) {
                    animator.setRepeatMode(ObjectAnimator.REVERSE);
                }

                animatorList.add(animator);
            }
        }

        animatorSet.playTogether(animatorList);
        animatorSet.start();
    }

    public Expression getNextExpression() {
        Expression nextExpression;

        transitionCounter++;

        if(transitionCounter % 2 == 0) {
            expressionIndex = (expressionIndex + 1) % Expression.vals.length;

            if (expressionIndex == 0) {
                expressionIndex++;
            }

            nextExpression = Expression.vals[expressionIndex];

            return nextExpression;
        }
        else {
            return Expression.NEUTRAL;
        }
    }

    /*
    public void giggle(float [] expY, long duration) {
        float currentY;

        for (int e = 0; e < NUM_EYES; e++) {
            for (int s = 0; s < NUM_SPHERES_PER_EYE; s++) {
                //currentY = expY[e == 0 ? s : (NUM_SPHERES_PER_EYE - s - 1)] * COORD_SCALING_FACTOR + screenHeight / 2 - SPRITE_HEIGHT / 2;
                currentY = expY[s] * COORD_SCALING_FACTOR + screenHeight / 2 - SPRITE_HEIGHT / 2;

                PropertyValuesHolder pvhY = PropertyValuesHolder.ofFloat("y", currentY, currentY + 0.05f * currentY);

                ObjectAnimator animator = ObjectAnimator.ofPropertyValuesHolder(spriteArray[s + e * NUM_SPHERES_PER_EYE], pvhY);
                animator.setDuration(duration);
                animator.setInterpolator(new CycleInterpolator(5f));

                animator.start();
            }
        }
    }
    */

}
