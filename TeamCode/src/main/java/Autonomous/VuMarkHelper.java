package Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by robotics on 1/9/18.
 */

public class VuMarkHelper {
    VuforiaHelper vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public VuMarkHelper(){
        vuforia = new VuforiaHelper();
        relicTrackables = vuforia.vuLoc.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }

    public RelicRecoveryVuMark getMark(){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }
}
