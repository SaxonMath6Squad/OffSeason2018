package Autonomous.ImageProcessing;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;

import java.util.ArrayList;

/**
 * Created by robotics on 12/12/17.
 */

public class CryptoBoxColumnImageProcessor {
    public int imageWidth;
    public int imageHeight;
    private double percentRequiredInColumnToCheck;
    private int minimumColumnWidth;

    public CryptoBoxColumnImageProcessor(int desiredHeight, int desiredWidth, double percentColumnCheck, int minColumnWidth){
        imageHeight = desiredHeight;
        imageWidth = desiredWidth;
        percentRequiredInColumnToCheck = percentColumnCheck;
        minimumColumnWidth = minColumnWidth;
    }

    public void setRequiredMinimumColumnWidth(double percentOfWidthOfImage){
        int widthInPixels = (int)(percentOfWidthOfImage*imageWidth + .5);
        if(widthInPixels <= 0) widthInPixels = 1;
        else if(widthInPixels >1){
            throw new RuntimeException("Wanted Column Width Greater Than Image Width!");
        }
        minimumColumnWidth = widthInPixels;
    }

    public void setRequiredPercentOfImageColumnBlue(double percentOfImageHeight){
        if(percentOfImageHeight > 1){
            throw new RuntimeException("Wanted Column Height Percent Greater Than 1.0!");
        }
        percentRequiredInColumnToCheck = percentOfImageHeight;
    }

    public Bitmap scaleBmp(Bitmap bmp){
        bmp = Bitmap.createScaledBitmap(bmp,imageWidth,imageHeight,true);
        Bitmap b = bmp.copy( Bitmap.Config.ARGB_8888 , true);
        return b;
    }

    private ArrayList<Integer> getColumnsWithRequiredPercentBlue(double [] blueFrequencyByColumn){
        ArrayList<Integer> interestingColumns = new ArrayList<Integer>();
        //look for columns that have the minimum required % of a color
        for(int i = 0; i < blueFrequencyByColumn.length; i++){
            if(blueFrequencyByColumn[i] >= percentRequiredInColumnToCheck){
                interestingColumns.add(Integer.valueOf(i));
            }
        }
        return interestingColumns;
    }

    private ArrayList<Integer> getColumnsWithRequiredBlueCount(int [] blueFrequencyByColumn){
        ArrayList<Integer> interestingColumns = new ArrayList<Integer>();
        //look for columns that have the minimum required % of a color
        int minimumPixelCount = (int)(percentRequiredInColumnToCheck * imageHeight + .5);
        for(int i = 0; i < blueFrequencyByColumn.length; i++){
            if(blueFrequencyByColumn[i] >= minimumPixelCount){
                interestingColumns.add(Integer.valueOf(i));
            }
        }
        return interestingColumns;
    }

    private ArrayList<Integer> getColumnBounds(ArrayList<Integer> columnsWithRequiredBluePercent){
        ArrayList<Integer> columnBounds = new ArrayList<Integer>();
        if(columnsWithRequiredBluePercent.size() > 0) {
            columnBounds.add(columnsWithRequiredBluePercent.get(0));
            for (int i = 1; i < columnsWithRequiredBluePercent.size(); i++) {
                if (i != 1) columnBounds.add(columnsWithRequiredBluePercent.get(i));
                while (i < columnsWithRequiredBluePercent.size() && columnsWithRequiredBluePercent.get(i).intValue() == columnsWithRequiredBluePercent.get(i - 1).intValue() + 1) {
                    i++;
                }
                columnBounds.add(columnsWithRequiredBluePercent.get(i - 1));
            }
            //look for columns to be at least ___ columns wide
            for (int i = 0; i < columnBounds.size() - 1; i += 2) {
                if (columnBounds.get(i + 1).intValue() - columnBounds.get(i).intValue() >= minimumColumnWidth) {
                } else {
                    columnBounds.remove(i);
                    columnBounds.remove(i);
                    i -= 2;
                }
            }
        }
        return columnBounds;
    }

    private ArrayList<Integer> getColumnCenters(ArrayList<Integer> columnBounds) {
        ArrayList<Integer> columnCenters = new ArrayList<Integer>();
        for (int i = 0; i < columnBounds.size() / 2; i++) {
            columnCenters.add((int) (columnBounds.get(i * 2).intValue() + (columnBounds.get(i * 2 + 1).intValue() - columnBounds.get(i * 2).intValue()) / 2.0 + .5));
        }
        return columnCenters;
    }

    public ArrayList<Integer> findColumnCenters(Bitmap bmp, boolean shouldModifyImage){
        ArrayList<Integer> columns = findColumns(bmp,shouldModifyImage);
        ArrayList<Integer> centers = new ArrayList<Integer>();
        for(int i = 0; i < columns.size() -1; i ++){
            centers.add(Integer.valueOf((int)((columns.get(i) + columns.get(i + 1))/2.0 + .5)));
        }
        return centers;
    }

    public ArrayList<Integer> findColumns(Bitmap bmp, boolean shouldModifyImage){
        Log.d("CF IMG ","Height: " + bmp.getHeight() + " Width: " + bmp.getWidth());
        if(bmp.getHeight() > imageHeight && bmp.getWidth() > imageWidth){
            bmp = scaleBmp(bmp);
        }
        ArrayList<Integer> columnCenters = new ArrayList<Integer>();
        int width = bmp.getWidth(), height = bmp.getHeight();
        int[] pixels = new int[width * height];
        bmp.getPixels(pixels, 0, width, 0, 0, width, height);
        long collapseStart = System.currentTimeMillis();
        int [] blueFrequencyByColumn = collapseVerticallyByBlueCount(pixels,width,height);
        Log.d("CF IMG PROC", "Collapse Time: " + (System.currentTimeMillis() - collapseStart));
        ArrayList<Integer> interestingColumns = getColumnsWithRequiredBlueCount(blueFrequencyByColumn);
        ArrayList<Integer> columnBounds = getColumnBounds(interestingColumns);
        for (int i = 0; i < columnBounds.size(); i++) {
            Log.d("Bound", columnBounds.get(i).toString());
        }
        //get average column locations
        columnCenters = getColumnCenters(columnBounds);
        for (int i = 0; i < columnCenters.size(); i++) {
            Log.d("Centers", columnCenters.get(i).toString());
        }
        Log.d("# of Columns", "" + columnCenters.size());
        if(shouldModifyImage){
            showBluePixels(pixels,height,width, Color.GREEN);
            showColumnCenters(pixels,height,width,columnCenters,Color.RED);
        }
        return columnCenters;
    }

    /*
    public ArrayList<Integer> findColumns(Bitmap bmp, boolean shouldModifyImage){
        Log.d("CF IMG ","Height: " + bmp.getHeight() + " Width: " + bmp.getWidth());
        if(bmp.getHeight() > imageHeight && bmp.getWidth() > imageWidth){
            bmp = scaleBmp(bmp);
        }
        ArrayList<Integer> columnCenters = new ArrayList<Integer>();
        int width = bmp.getWidth(), height = bmp.getHeight();
        int[] pixels = new int[width * height];
        bmp.getPixels(pixels, 0, width, 0, 0, width, height);
        long collapseStart = System.currentTimeMillis();
        double [] blueFrequencyByColumn = collapseVerticallyByBluePercent(pixels,width,height);
        Log.d("CF IMG PROC", "Collapse Time: " + (System.currentTimeMillis() - collapseStart));
        ArrayList<Integer> interestingColumns = getColumnsWithRequiredPercentBlue(blueFrequencyByColumn);
        ArrayList<Integer> columnBounds = getColumnBounds(interestingColumns);
        for (int i = 0; i < columnBounds.size(); i++) {
            Log.d("Bound", columnBounds.get(i).toString());
        }
        //get average column locations
        columnCenters = getColumnCenters(columnBounds);
        for (int i = 0; i < columnCenters.size(); i++) {
            Log.d("Centers", columnCenters.get(i).toString());
        }
        Log.d("# of Columns", "" + columnCenters.size());
        if(shouldModifyImage){
            showBluePixels(pixels,height,width, Color.GREEN);
            showColumnCenters(pixels,height,width,columnCenters,Color.RED);
        }
        return columnCenters;
    }
     */

    private void showColumnCenters(int [] pixels, int height, int width, ArrayList<Integer> columnCenters, int colorToShowWith){
        for (int i = 0; i < columnCenters.size(); i++) {
            for (int r = 0; r < height; r++) {
                pixels[r * width + columnCenters.get(i).intValue()] = colorToShowWith;
            }
        }
    }

    public void showBluePixels(int [] pixels, int height, int width, int colorToReplaceWith){
        for (int c = 0; c < width; c++) {
            int numberOfBluePixels = 0;
            for (int r = 0; r < height; r++) {
                int color = pixels[r * width + c];
                int[] rgba = {Color.red(color), Color.blue(color), Color.green(color), Color.alpha(color)};
                float[] hsv = new float[3];
                Color.colorToHSV(color, hsv);
                //check for blue
                if (checkIfBlue(hsv)) {
                    rgba[0] = 0;
                    rgba[1] = 250;
                    rgba[2] = 250;
                    pixels[r * width + c] = colorToReplaceWith;
                    numberOfBluePixels++;
                }
            }
        }
    }
    
    public boolean isCentered(int desiredCenter, int imageCenter){
        boolean centered = false;
        if(imageCenter > desiredCenter){
            if(imageCenter - desiredCenter <= 20){ //TODO: make the tolerance a constant
                centered = true;
            }
        } else {
            if(desiredCenter - imageCenter <= 20){
                centered = true;
            }
        }
        return centered;
    }


    public int [] collapseVerticallyByBlueCount(int [] pixels, int imageWidth, int imageHeight){
        //collapse into a single, frequency based
        int [] toReturn = new int[imageWidth];
        for(int c = 0; c < imageWidth; c ++){
            for(int r = 0; r < imageHeight; r ++){
                int color = pixels[r*imageWidth + c];
                if(checkIfBlue(color)){
                    toReturn[c] ++;
                }
            }
        }
        return toReturn;
    }

    public double [] collapseVerticallyByBluePercent(int [] pixels, int imageWidth, int imageHeight){
        //collapse into a single, frequency based
        double [] toReturn = new double[imageWidth];
        for(int c = 0; c < imageWidth; c ++){
            for(int r = 0; r < imageHeight; r ++){
                int color = pixels[r*imageWidth + c];
                if(checkIfBlue(color)){
                    toReturn[c] ++;
                }
            }
            toReturn[c] = toReturn[c]/imageHeight;
        }
        return toReturn;
    }

    public boolean checkIfBlue(float [] hsl){
        if (hsl[0] > 180 && hsl[0] < 300) {
            //make sure it's not a white blue
            if (hsl[1] > .5) {
                //make sure it's not a black blue
                if (hsl[2] > .2) {
                    return true;
                }
            }
        }
        return false;
    }
    public boolean checkIfBlue(int color){
        float [] hsl = new float[3];
        Color.colorToHSV(color,hsl);
        return checkIfBlue(hsl);
    }





}
