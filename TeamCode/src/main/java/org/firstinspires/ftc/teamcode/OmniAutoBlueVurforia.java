package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoBlueVuforia", group ="Expiremental")

public class OmniAutoBlueVurforia extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        final double ROBOT_ANGLE = 90.0;
        setupRobotParameters(4, 40);
        setupVuforiaImages();
        setupTextToSpeech();

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();

        telemetry.addLine("Set");
        updateTelemetry(telemetry);

        // Move to shooting position from wall using range sensor.
        telemetry.addLine("Move To Wall");
        updateTelemetry(telemetry);

        driveToWall(1.0, 0.2, 30.0, 5000, false);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.MID_LOW_SHOOT_SPEED);
        rotateRobot(0.4, 90.0, 7000);
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

        enableVuforiaTracking();
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
