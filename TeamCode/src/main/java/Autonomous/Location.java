package Autonomous;

/**
 * Created by root on 8/22/17.
 */

public class Location {
    double x;
    double y;

    public Location(double x1, double y1){
        x = x1;
        y = y1;
    }
    double getX(){
        return x;
    }
    double getY(){
        return y;
    }
    public void setX(double nX){
        x= nX;
    }
    public void setY(double nY){
        y= nY;
    }

    public void updateXY(double x1, double y1){
        x = x1;
        y = y1;
    }

    public double distanceToLocation(Location location){
        double distance = 0;
        Math.sqrt(Math.pow((location.getX() - getX()), 2) + Math.pow((location.getY() - getY()), 2));
        return distance;
    }
}
