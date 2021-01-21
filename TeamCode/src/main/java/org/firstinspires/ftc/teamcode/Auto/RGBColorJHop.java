package org.firstinspires.ftc.teamcode.Auto;


public class RGBColorJHop {
    private int red = 0;
    private int green = 0;
    private int blue = 0;

    public RGBColorJHop(){

    }
    public RGBColorJHop(int r, int g, int b){
        red = r;
        green = g;
        blue=  b;
    }

    public void setRed(int r)
    {
        red = r;
    }
    public void setGreen(int g) {
        green = g;
    }
    public void setBlue(int b)
    {
        blue = b;
    }

    public int getRed(){
        return red;
    }
    public int getGreen(){
        return green;
    }
    public int getBlue(){
        return blue;
    }
    public int getValue(){
        return red + green - blue;
    }
}
