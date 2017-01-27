package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: RotateTest", group ="Expiremental")

public class OmniAutoBlueShootOnlyNoCap extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();

        spinRobot(-1.0, 720.0, 10000);

        endAuto();
    }
}
