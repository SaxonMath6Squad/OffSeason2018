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
    public final static int LIFT_MOTOR = 0;
    public final static int GLYPH_MOTOR = 1;
    public final static int FL_MOTOR = 2;
    public final static int FR_MOTOR = 3;
    public final static int BR_MOTOR = 4;
    public final static int BL_MOTOR = 5;

    //cryptobox designations
    final static double CRYPTOBOX_WIDTH = 2;
    final static double CRYPTOBOX_COLORED_COLUMN_WIDTHS = .3;
    final static double CRYPTOBOX_SCORING_COLUMN_WIDTHS = 0;
    final static Location [] cryptoBoxCenters = {new Location(0,0)};
    public final static double ROW1 = 6;
    public final static double ROW2 = 11.5;
    public final static double ROW3 = 17.75;
    public final static double ROW4 = 19.25;

    //balance stones
    final static double BALANCE_STONE_WIDTH = 1.0;
    final static Location [] startLocations = {new Location(0,0)};


    //relics
    static Location [] RELIC_LOCATIONS = {new Location(0,0)};


}
