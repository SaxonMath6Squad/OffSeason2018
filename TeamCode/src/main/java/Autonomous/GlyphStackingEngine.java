package Autonomous;

/**
 * Created by robotics on 9/22/17.
 */
/*
9 10 11
6  7  8
3  4  5
0  1  2
 */
/*
    A class to decide how to put the cryptobox together based off of patterns and glyphs currently in the robot
 */
public class GlyphStackingEngine {
    public enum Glyph {NONE, BROWN, GREY};
//    public spotStatus cryptoBox[3][4];
//
//
//    //return the location (index to the 3 x 4 cryptobox) of where to stack. return a negative value if we should not stack this
//    public Location getLocToStore(GlyphColor color){
//
//    }

    public final Glyph[][] SNAKE_ONE = {{Glyph.BROWN, Glyph.GREY, Glyph.GREY},
            {Glyph.BROWN, Glyph.BROWN, Glyph.GREY},
            {Glyph.GREY, Glyph.BROWN, Glyph.BROWN},
            {Glyph.GREY, Glyph.GREY, Glyph.BROWN}};

    public final Glyph[][] SNAKE_TWO = {{Glyph.GREY, Glyph.BROWN, Glyph.BROWN},
            {Glyph.GREY, Glyph.GREY, Glyph.BROWN},
            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
            {Glyph.BROWN, Glyph.BROWN, Glyph.GREY}};
}
