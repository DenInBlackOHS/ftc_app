package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoRedShootOnly", group ="Auto")

public class OmniAutoRedShootOnly extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        final double ROBOT_ANGLE = 180.0;
        final double DRIVE_ANGLE = -90.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        sleep(10000);

        robot.resetGyro();

        driveToWall(1.0, 0.2, 50.0, 5000, true);
        if(isStopRequested())
        {
            return;
        }

        driveToWall(1.0, 0.2, 52.0, 5000, false);
        if(isStopRequested())
        {
            return;
        }
        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        rotateRobot(0.4, 55.0, 7000);
        shoot(2000);
        robot.setShooterSpeed(0.0);

        rotateRobot(0.4, 120.0, 7000);
        // This should knock the cap ball off, and set the robot on the pedestal
        driveAtHeadingForTime(1.0, 0.2, 322.0, 322.0, 500);

        endAuto();
    }
}
