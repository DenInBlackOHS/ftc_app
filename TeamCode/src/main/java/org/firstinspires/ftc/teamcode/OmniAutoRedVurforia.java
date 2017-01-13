package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoRedVuforia", group ="Expiremental")

public class OmniAutoRedVurforia extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        final double ROBOT_ANGLE = 269.0;
        setupRobotParameters(4, 40);
        setupVuforiaImages();
        setupTextToSpeech();

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();
        enableVuforiaTracking();

        driveToWall(1.0, 0.2, 30.0, 5000, false);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.MID_LOW_SHOOT_SPEED);
        rotateRobot(0.6, 90.0, 7000);
        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }
        shoot(2000);
        robot.setShooterSpeed(0.0);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        // We should be able to acquire the gears target here
        rotateRobotToAngle(0.6, ROBOT_ANGLE - 22.0, 7000);
        if(isStopRequested())
        {
            return;
        }
        acquireRedTarget1(30000);
        if(isStopRequested())
        {
            // +14 degrees
            // -880 mm
            return;
        }
        endAuto();
    }
}
