package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoRed", group ="Auto")

public class OmniAutoRed extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException{

        boolean beacon1Pressed = false;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();
        telemetry.addLine("Set");
        updateTelemetry(telemetry);

        //first, the robot will shoot.
//        shoot(HardwareOmnibot.MID_LOW_SHOOT_SPEED, 4000);

//        telemetry.addLine("Done Shooting");
//        updateTelemetry(telemetry);

        // Move 12" from wall.
        telemetry.addLine("Move To Wall");
        updateTelemetry(telemetry);
        moveToWall(0.5, 12*OmniAutoClass.MM_PER_INCH, 5000);
        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }
        sleep(1000);

        //turn and drive forward to the first beacon
//        rotateRobot(0.2, -35.0, 1000);
//        telemetry.addLine("Rotated");
//        updateTelemetry(telemetry);

//        sleep(1000);

//        driveForwardOnHeading(-0.7, 55, 5000);
//        telemetry.addLine("Drove");
//        updateTelemetry(telemetry);

//        sleep(1000);

//        rotateRobot(0.2, 50.0, 2000);
//        driveForwardOnHeading(-0.7, 4, 1000);

        //sense for color...
        /*
        while(!beacon1Pressed) {
            if (!returnedRed()) {
                driveForwardOnHeading(-0.7, 4, 1000);
            } else {
                driveRightOnHeading(-0.5, 2, 1000);
                beacon1Pressed = true;
                driveForwardOnHeading(-0.7, 36, 5000);
            }
        }

         */

        // Check to see if the program should exit
//        if(isStopRequested())
//        {
//            return;
//        }

        telemetry.addLine("Go");
        updateTelemetry(telemetry);

        endAuto();
    }
}
