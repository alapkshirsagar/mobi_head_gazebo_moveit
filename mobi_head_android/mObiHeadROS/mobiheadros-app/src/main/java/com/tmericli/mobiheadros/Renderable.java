package com.tmericli.mobiheadros;

/**
 * Base class defining the core set of information necessary to render and move
 * an object on the screen.
 *
 * @author Tekin Mericli
 */
public class Renderable {
    // Position.
    public float x;
    public float y;

    // Velocity.
    public float velocityX;
    public float velocityY;

    // Size.
    public float width;
    public float height;

    // color
    public float hue;


    public void setX (float val) {x = val;}
    public float getX () {return x;}

    public void setY (float val) {y = val;}
    public float getY () {return y;}

    public void setWidth (float val) {width = val;}
    public float getWidth () {return width;}

    public void setHeight (float val) {height = val;}
    public float getHeight () {return height;}

    public void setHue(float val) {hue = val;}
    public float getHue() {return hue;}

    public Renderable() {

    }
}
