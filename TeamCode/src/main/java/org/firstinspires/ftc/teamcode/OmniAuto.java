package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: Auto", group ="Auto")

public class OmniAuto extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException{

//        robot.init(hardwareMap);
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
