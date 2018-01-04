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
    final static double CRYPTOBOX_WIDTH_INCHES = 24;
    final static double CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES = 1.375;
    final static double CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES = 6.125;
    public final static double CRYPTOBOX_THICKNESS_INCHES = 4;

    public Location determineLocationOfCryptoboxScoringCenter(int team, int col){
        double x = cryptoBoxCenters[team].getX();
        double y = cryptoBoxCenters[team].getY();
        if(team == BLUE_ALLIANCE_1 || team == RED_ALLIANCE_1){
            switch (col){
                case SCORING_COLUMN_1:
                    x += CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES + CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES;
                    break;
                case SCORING_COLUMN_2:
                    break;
                case SCORING_COLUMN_3:
                    x -= CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES + CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES;
                    break;
            }
        } else if(team == BLUE_ALLIANCE_2){
            switch (col){
                case SCORING_COLUMN_1:
                    y += CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES + CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES;
                    break;
                case SCORING_COLUMN_2:
                    break;
                case SCORING_COLUMN_3:
                    y -= CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES - CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES;
                    break;
            }
        } else {
            switch (col){
                case SCORING_COLUMN_1:
                    y -= CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES + CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES;
                    break;
                case SCORING_COLUMN_2:
                    break;
                case SCORING_COLUMN_3:
                    y += CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES + CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES;
                    break;
            }
        }
        return new Location(x, y);
    }

    public Location determinePitLocation(int team, Location robotLocation){
        switch (team){
            case RED_ALLIANCE_1:
                return new Location(72, 84);
            case RED_ALLIANCE_2:
                return new Location(84, robotLocation.getY());
            case BLUE_ALLIANCE_1:
                return new Location(72, 84);
            case BLUE_ALLIANCE_2:
                return new Location(60, robotLocation.getY());
            default:
                return PIT_CENTER;
        }
    }

    /*
        start at balancing beam
        figure out which column we want to go by determining its position
        travel to that position
        do stuff

     */

    public final static Location[] cryptoBoxCenters = {new Location(108,20), new Location(120, 84), new Location(36, 20), new Location(24, 84)}; //reference the centers of the cryptoboxes
    public final static double GROUND = 0;
    public final static double ROW1 = 6;
    public final static double ROW2 = 11.5;
    public final static double ROW3 = 17.75;
    public final static double ROW4 = 19.25;
    public final static double DIST_FROM_CRYPTOBOX_2_TO_PIT_INCHES = 48;
    public final static double BLUE_CRYPTOBOX_2_TO_PIT_ANGLE = 20;
    public final static double PIT_TO_BLUE_CRYPTOBOX_2_ANGLE = 200;
    public static final Location PIT_CENTER = new Location(72, 84);


    public final static double RIGHT_COLUMN_DISTANCE_TO_STONE_INCHES = 2.2*12;
    public final static double MIDDLE_COLUMN_DISTANCE_TO_STONE_INCHES = 2.9*12;
    public final static double LEFT_COLUMN_DISTANCE_TO_STONE_INCHES = 3.4*12;
    public final static long CRYPTOBOX_APPROACH_TIME_LIMIT = 1500;

    //balance stones
    final static double BALANCE_STONE_WIDTH = 2.0;
    public final static Location[] startLocations = {new Location(120,48), new Location(120, 120), new Location(24, 48), new Location(24, 120)};


    //relics
    static Location[] RELIC_LOCATIONS = {new Location(0,0), new Location(12, 0), new Location(0, 12), new Location(12, 12)};


}
