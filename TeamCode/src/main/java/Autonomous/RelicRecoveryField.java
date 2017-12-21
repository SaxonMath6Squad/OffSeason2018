package Autonomous;

/**
 * Created by Jeremy on 12/18/2017.
 */

public class RelicRecoveryField {
    final static int BLUE_ALLIANCE_1 = 0;
    final static int BLUE_ALLIANCE_2 = 1;
    final static int RED_ALLIANCE_1 = 2;
    final static int RED_ALLIANCE_2 = 3;
    public final int TICKS_PER_REV = 1120;
    public final static int FL_MOTOR = 0;
    public final static int FR_MOTOR = 1;
    public final static int BR_MOTOR = 2;
    public final static int BL_MOTOR = 3;
    public final static int LIFT_MOTOR = 4;
    public final static int GLYPH_MOTOR = 5;

    //cryptobox designations
    final static double CRYPTOBOX_WIDTH = 2;
    final static double CRYPTOBOX_COLORED_COLUMN_WIDTHS = .3;
    final static double CRYPTOBOX_SCORING_COLUMN_WIDTHS = 0;
    public final static int BLUE_CRYPTOBOX_LEFT = 0;
    public final static int BLUE_CRYPTOBOX_RIGHT = 1;
    public final static int RED_CRYPTOBOX_LEFT = 2;
    public final static int RED_CRYPTOBOX_RIGHT = 3;
    final static Location[] cryptoBoxCenters = {new Location(0,3), new Location(7, 0), new Location(0, 9), new Location(7, 12)};
    public final static double GROUND = 0;
    public final static double ROW1 = 6;
    public final static double ROW2 = 11.5;
    public final static double ROW3 = 17.75;
    public final static double ROW4 = 19.25;

    //balance stones
    final static double BALANCE_STONE_WIDTH = 2.0;
    public final static int BLUE_STONE_LEFT = 0;
    public final static int BLUE_STONE_RIGHT = 1;
    public final static int RED_STONE_LEFT = 2;
    public final static int RED_STONE_RIGHT = 3;
    final static Location[] startLocations = {new Location(4,2), new Location(10, 2), new Location(4, 10), new Location(10, 10)};


    //relics
    public final static int BLUE_RELIC_LEFT = 0;
    public final static int BLUE_RELIC_RIGHT = 1;
    public final static int RED_RELIC_LEFT = 2;
    public final static int RED_RELIC_RIGHT = 3;
    static Location[] RELIC_LOCATIONS = {new Location(0,0), new Location(12, 0), new Location(0, 12), new Location(12, 12)};


}
