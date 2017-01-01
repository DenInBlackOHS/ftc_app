package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 11/19/2016.
 */

@Autonomous(name="Omni: ShootOnly", group ="Auto")

public class OmniShootAuto extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        setupRobotParameters(4, 20);

        waitForStart();

        shoot(HardwareOmnibot.HIGH_SHOOT_SPEED, 4000);

        endAuto();
    }
}
