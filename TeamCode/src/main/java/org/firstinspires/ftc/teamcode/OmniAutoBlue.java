package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoBlue", group ="Auto")

public class OmniAutoBlue extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException{

        boolean beacon1Pressed = false;
        setupRobotParameters(4, 20);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();
        telemetry.addLine("Set");
        updateTelemetry(telemetry);

        //first, the robot will shoot.
        shoot(HardwareOmnibot.MID_LOW_SHOOT_SPEED, 4000);

        telemetry.addLine("Done Shooting");
        updateTelemetry(telemetry);

        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }
        sleep(1000);

        //turn and drive forward to the first beacon
        rotateRobot(0.2, 35.0, 1000);
        telemetry.addLine("Rotated");
        updateTelemetry(telemetry);

        sleep(1000);

        driveForwardOnHeading(-0.7, 60, 7000);
        telemetry.addLine("Drove");
        updateTelemetry(telemetry);

        sleep(1000);

        rotateRobot(-0.2, 70.0, 2000);
        driveForwardOnHeading(-0.7, 4, 1000);

        //sense for color...
        /*
        while(!beacon1Pressed) {
            if (!returnedBlue()) {
                driveForwardOnHeading(-0.7, 4, 1000);
            } else {
                driveRightOnHeading(0.5, 2, 1000);
                beacon1Pressed = true;
                driveForwardOnHeading(-0.7, 36, 5000);
            }
        }

         */

        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }

        telemetry.addLine("Go");
        updateTelemetry(telemetry);

        endAuto();
    }
}
