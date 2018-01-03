package Autonomous;

/**
 * Created by Jeremy on 12/18/2017.
 */

/*
    A class to store field constants and general autonomous functions
 */
public class RelicRecoveryField {
    public final static int BLUE_ALLIANCE_1 = 0;
    public final static int BLUE_ALLIANCE_2 = 1;
    public final static int RED_ALLIANCE_1 = 2;
    public final static int RED_ALLIANCE_2 = 3;

    public final static int SCORING_COLUMN_1 = 0;
    public final static int SCORING_COLUMN_2 = 1;
    public final static int SCORING_COLUMN_3 = 2;

    //cryptobox designations
    final static double CRYPTOBOX_WIDTH_INCHES = 24; //feet
    final static double CRYPTOBOX_COLORED_COLUMN_WIDTHS = .3;
    final static double CRYPTOBOX_SCORING_COLUMN_WIDTHS = 0;

    public Location determineLocationOfCryptoboxScoringCenter(int team, int col){
        double x = cryptoBoxCenters[team].getX();
        double y = cryptoBoxCenters[team].getY();
        if(team == BLUE_ALLIANCE_1 || team == RED_ALLIANCE_1){
            if(col != SCORING_COLUMN_2){
                x += (col == SCORING_COLUMN_1)? -6:6;
            }
        } else if(team == BLUE_ALLIANCE_2){
            if(col != SCORING_COLUMN_2){
                y += (col == SCORING_COLUMN_1)? 6:-6;
            }
        } else {
            if(col != SCORING_COLUMN_2){
                y += (col == SCORING_COLUMN_1)? -6:6;
            }
        }
        return new Location(x, y);
    }

    /*
        start at balancing beam
        figure out which column we want to go by determining its position
        travel to that position
        do stuff

     */

    final static Location[] cryptoBoxCenters = {new Location(0,3), new Location(7, 0), new Location(0, 9), new Location(7, 12)}; //reference the centers of the cryptoboxes
    public final static double GROUND = 0;
    public final static double ROW1 = 6;
    public final static double ROW2 = 11.5;
    public final static double ROW3 = 17.75;
    public final static double ROW4 = 19.25;




    public final static double RIGHT_COLUMN_DISTANCE_TO_STONE_INCHES = 2.2*12;
    public final static double MIDDLE_COLUMN_DISTANCE_TO_STONE_INCHES = 2.9*12;
    public final static double LEFT_COLUMN_DISTANCE_TO_STONE_INCHES = 3.4*12;
    public final static long CRYPTOBOX_APPROACH_TIME_LIMIT = 1500;

    //balance stones
    final static double BALANCE_STONE_WIDTH = 2.0;
    final static Location[] startLocations = {new Location(4,2), new Location(10, 2), new Location(4, 10), new Location(10, 10)};


    //relics
    static Location[] RELIC_LOCATIONS = {new Location(0,0), new Location(12, 0), new Location(0, 12), new Location(12, 12)};


}
