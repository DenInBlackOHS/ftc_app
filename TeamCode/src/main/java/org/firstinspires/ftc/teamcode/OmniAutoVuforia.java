package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: Auto Vuforia", group ="Auto")

public class OmniAutoVuforia extends OmniAutoClass {
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException{
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATaHrPr/////AAAAGYhG118G0EZgjFy6T7Snt3otqlgNSultuXDM66X1x1QK3ov5GUJcqL/9RTkdWkDlZDRxBKTAWm/szD7VmJteuQd2WfAk1t8qraapAsr2b4H5k5r4IpIO0UZghwNqhUqfZnCYl3e9tmmuocgZlfLXt4Xw+IAGxZ5e9MaQLR5lTv9/aFO1/CnH9/8jvnSq5NGeLrCHA6BtvqS30sAv7NYX8gz79MHaNiGZvyrUXZslbp2HHkehCocBbc080NrnYCouuUCqIbaMFl4ei8/ViSvdvtJDks4ox5KynBth4HaLHYpYkK3T2XJ1dBab6KfrWn6dm8ug7tfHTy68wLqWev7IWB0oPcqGOY+bZiz343VteHzk";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables allTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable redTarget1 = allTargets.get(0);
        redTarget1.setName("RedTarget1");  // ???
        VuforiaTrackable redTarget2 = allTargets.get(1);
        redTarget1.setName("RedTarget2");  // ???

        VuforiaTrackable blueTarget1  = allTargets.get(2);
        blueTarget1.setName("BlueTarget1");  // ???
        VuforiaTrackable blueTarget2  = allTargets.get(2);
        blueTarget2.setName("BlueTarget2");  // ???

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(allTargets);

        setupRobotParameters(4, 20);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();
        telemetry.addLine("Set");
        updateTelemetry(telemetry);

        shoot(.35, 4000);

        telemetry.addLine("Done Shooting");
        updateTelemetry(telemetry);

        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }
        sleep(6000);

        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }
        driveForward(-0.4, 72, 3000);
        telemetry.addLine("Go");
        updateTelemetry(telemetry);
    }

}
