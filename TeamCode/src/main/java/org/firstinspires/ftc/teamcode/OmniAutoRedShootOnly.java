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
        final double WALL_DISTANCE = 44.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

//        sleep(10000);

        robot.resetGyro();

        driveToWall(1.0, 0.2, WALL_DISTANCE, 10000, true);
        if(isStopRequested())
        {
            return;
        }

        rotateRobotToAngle(1.0, 0.0, 1000);
        if(isStopRequested())
        {
            return;
        }
        driveDistanceSidewaysOnHeading(1.0, 30.0, 10000, false);
        if(isStopRequested())
        {
            return;
        }
        driveToWall(1.0, 0.2, WALL_DISTANCE, 10000, true);
        if(isStopRequested())
        {
            return;
        }

        rotateRobotToAngle(1.0, 0.0, 1000);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot to shooting angle
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        rotateRobot(0.7, 155.0, 7000);
        shoot(2000);
        robot.setShooterSpeed(0.0);

        endAuto();
    }
}
