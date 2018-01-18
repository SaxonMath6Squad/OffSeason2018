package Autonomous;

import java.util.ArrayList;

/**
 * Created by robotics on 9/22/17.
 */
/*
(0,3) | (1,3) | (2,3)
------|-------|------
(0,2) | (1,2) | (2,2)
------|-------|------
(0,1) | (1,1) | (2,1)
------|-------|------
(0,0) | (1,0) | (2,0)
 */
/*
    A class to decide how to put the cryptobox together based off of patterns and glyphs currently in the robot
 */
public class GlyphStackingEngine {
    public enum Glyph {NONE, BROWN, GRAY};
    public static final int BROWN_SNAKE = 0;
    public static final int GRAY_SNAKE = 1;
    public static final int BROWN_BIRD = 2;
    public static final int GRAY_BIRD = 3;
    public static final int BROWN_FROG = 4;
    public static final int GRAY_FROG = 5;

    private Glyph[][] wantedCipher = null;
    private static Glyph[][] cryptobox = new Glyph[3][4];

    private Glyph[][] brownSnake = {
            {Glyph.GRAY, Glyph.GRAY, Glyph.BROWN, Glyph.BROWN},
            {Glyph.GRAY, Glyph.BROWN, Glyph.BROWN, Glyph.GRAY},
            {Glyph.BROWN, Glyph.BROWN, Glyph.GRAY, Glyph.GRAY}
    };
    private Glyph[][] graySnake = {
            {Glyph.BROWN, Glyph.BROWN, Glyph.GRAY, Glyph.GRAY},
            {Glyph.BROWN, Glyph.GRAY, Glyph.GRAY, Glyph.BROWN},
            {Glyph.GRAY, Glyph.GRAY, Glyph.BROWN, Glyph.BROWN}
    };
    private Glyph[][] brownBird = {
            {Glyph.BROWN, Glyph.GRAY, Glyph.GRAY, Glyph.BROWN},
            {Glyph.GRAY, Glyph.BROWN, Glyph.BROWN, Glyph.GRAY},
            {Glyph.BROWN, Glyph.GRAY, Glyph.GRAY, Glyph.BROWN}
    };
    private Glyph[][] grayBird = {
            {Glyph.GRAY, Glyph.BROWN, Glyph.BROWN, Glyph.GRAY},
            {Glyph.BROWN, Glyph.GRAY, Glyph.GRAY, Glyph.BROWN},
            {Glyph.GRAY, Glyph.BROWN, Glyph.BROWN, Glyph.GRAY}
    };
    private Glyph[][] brownFrog = {
            {Glyph.BROWN, Glyph.GRAY, Glyph.BROWN, Glyph.GRAY},
            {Glyph.GRAY, Glyph.BROWN, Glyph.GRAY, Glyph.BROWN},
            {Glyph.BROWN, Glyph.GRAY, Glyph.BROWN, Glyph.GRAY}
    };
    private Glyph[][] grayFrog = {
            {Glyph.GRAY, Glyph.BROWN, Glyph.GRAY, Glyph.BROWN},
            {Glyph.BROWN, Glyph.GRAY, Glyph.BROWN, Glyph.GRAY},
            {Glyph.GRAY, Glyph.BROWN, Glyph.GRAY, Glyph.BROWN}
    };

    public GlyphStackingEngine(ArrayList<Glyph> glyphColors, ArrayList<Location> glyphPositions) {
        boolean shouldContinue = true;
        for(int x = 0; x < 3; x++) {
            for(int y = 0; y < 4; y++) {
                for(int i = 0; i < glyphPositions.size(); i++){
                    if(glyphPositions.get(i).getX() == x && glyphPositions.get(i).getY() == y){
                        cryptobox[x][y] = glyphColors.get(i);
                        shouldContinue = false;
                    }
                    if(shouldContinue) cryptobox[x][y] = Glyph.NONE;
                }
                shouldContinue = true;
            }
        }
        determineCipherToUse();
    }

    public GlyphStackingEngine(int cipher) {
        determineCipherToUse(cipher);
        for(int y = 0; y < 4; y++) {
            for(int x = 0; x < 3; x++) {
                cryptobox[x][y] = Glyph.NONE;
            }
        }
    }

    public GlyphStackingEngine() {
        for(int y = 0; y < 4; y++) {
            for(int x = 0; x < 3; x++) {
                cryptobox[x][y] = Glyph.NONE;
            }
        }
    }

    public boolean placeGlyph(Glyph color, Location location) {
        if(location.getX() != -1 && location.getY() != -1) {
            if(cryptobox[(int)location.getX()][(int)location.getY()] == Glyph.NONE) {
                cryptobox[(int)location.getX()][(int)location.getY()] = color;
                return true;
            }
        }
        return false;
    }

    public Glyph getGlyph(Location location) {
        Glyph placedGlyph = cryptobox[(int)location.getX()][(int)location.getY()];
        return placedGlyph;
    }

    public Location getPlacableLocation() {
        Location placableLocation = new Location(-1, -1);
        for(int y = 0; y < 4; y++) {
            for(int x = 0; x < 3; x++) {
                if(getGlyph(new Location(x, y)) == Glyph.NONE) {
                    placableLocation = new Location(x, y);
                    return placableLocation;
                }
            }
        }
        return placableLocation;
    }

    public Location getPlacableLocation(ArrayList<Location> locationsToExclude) {
        Location placableLocation = new Location(-1, -1);
        boolean shouldSkip = false;
        for(int y = 0; y < 4; y++) {
            for(int x = 0; x < 3; x++) {
                for(int i = 0; i < locationsToExclude.size(); i++) {
                    if(locationsToExclude.get(i).getX() == x && locationsToExclude.get(i).getY() == y) {
                        shouldSkip = true;
                    }
                }
                if(!shouldSkip){
                    if(getGlyph(new Location(x, y)) == Glyph.NONE) {
                        if(y > 0) {
                            if(getGlyph(new Location(x, y-1)) == Glyph.NONE);
                            else {
                                placableLocation = new Location(x, y);
                                return placableLocation;
                            }
                        } else {
                            placableLocation = new Location(x, y);
                            return placableLocation;
                        }
                    }
                }
                shouldSkip = false;
            }
        }
        return placableLocation;
    }

    public Location getPlacableCipherLocation(Glyph color) {
        Location placableLocation;
        if(wantedCipher != null) {
            ArrayList<Location> placesToExclude = new ArrayList<>();
            for(int y = 0; y < 4; y++) {
                for(int x = 0; x < 3; x++) {
                    if(wantedCipher[x][y] != color) {
                        placesToExclude.add(new Location(x, y));
                    }
                }
            }
            placableLocation = getPlacableLocation(placesToExclude);
        } else {
            placableLocation = getPlacableLocation();
        }
        return placableLocation;
    }

    public static boolean isBoxFull() {
        boolean isFull = false;
        int count = 0;
        for(int y = 0; y < 4; y++) {
            for(int x = 0; x < 3; x++) {
                if(cryptobox[x][y] != Glyph.NONE) count++;
            }
        }
        if(count == 12) isFull = true;
        return isFull;
    }

    public void determineCipherToUse(int cipher){
        switch(cipher) {
            case BROWN_SNAKE:
                wantedCipher = brownSnake;
                break;
            case GRAY_SNAKE:
                wantedCipher = graySnake;
                break;
            case BROWN_BIRD:
                wantedCipher = brownBird;
                break;
            case GRAY_BIRD:
                wantedCipher = grayBird;
                break;
            case BROWN_FROG:
                wantedCipher = brownFrog;
                break;
            case GRAY_FROG:
                wantedCipher = grayFrog;
                break;
        }
    }

    private void determineCipherToUse(){
        if(cryptobox[0][0] == Glyph.BROWN){
            if(cryptobox[1][0] == Glyph.GRAY) {
                if(cryptobox[2][0] == Glyph.BROWN || cryptobox[2][0] == Glyph.NONE) wantedCipher = brownFrog;
            } else if(cryptobox[1][0] == Glyph.BROWN) {
                if(cryptobox[2][0] == Glyph.GRAY || cryptobox[2][0] == Glyph.NONE) wantedCipher = graySnake;
            } else {
                if(cryptobox[2][0] == Glyph.BROWN || cryptobox[2][0] == Glyph.NONE) wantedCipher = brownFrog;
                else wantedCipher = graySnake;
            }
        } else if(cryptobox[0][0] == Glyph.GRAY){
            if(cryptobox[1][0] == Glyph.BROWN){
                if(cryptobox[2][0] == Glyph.GRAY || cryptobox[2][0] == Glyph.NONE) wantedCipher = grayFrog;
            } else if(cryptobox[1][0] == Glyph.GRAY) {
                if(cryptobox[2][0] == Glyph.BROWN || cryptobox[2][0] == Glyph.NONE) wantedCipher = brownSnake;
            } else {
                if(cryptobox[2][0] == Glyph.GRAY || cryptobox[2][0] == Glyph.NONE) wantedCipher = grayFrog;
                else wantedCipher = brownSnake;
            }
        } else {
            if(cryptobox[1][0] == Glyph.BROWN) {
                if(cryptobox[2][0] == Glyph.GRAY || cryptobox[2][0] == Glyph.NONE) wantedCipher = grayFrog;
            } else if(cryptobox[1][0] == Glyph.GRAY) {
                if(cryptobox[2][0] == Glyph.BROWN || cryptobox[2][0] == Glyph.NONE) wantedCipher = brownFrog;
            } else {
                if(cryptobox[2][0] == Glyph.BROWN || cryptobox[2][0] == Glyph.NONE) wantedCipher = brownFrog;
                else wantedCipher = grayFrog;
            }
        }
    }
}
