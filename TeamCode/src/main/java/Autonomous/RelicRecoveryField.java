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

    //cryptobox designations
    final static double CRYPTOBOX_WIDTH = 2;
    final static double CRYPTOBOX_COLORED_COLUMN_WIDTHS = .3;
    final static double CRYPTOBOX_SCORING_COLUMN_WIDTHS = 0;
    final static Location [] cryptoBoxCenters = {new Location(0,0)};
    public final static int ROW1 = 6;
    public final static int ROW2 = 12;
    public final static int ROW3 = 18;
    public final static int ROW4 = 24;

    //balance stones
    final static double BALANCE_STONE_WIDTH = 1.0;
    final static Location [] startLocations = {new Location(0,0)};


    //relics
    static Location [] RELIC_LOCATIONS = {new Location(0,0)};


}
