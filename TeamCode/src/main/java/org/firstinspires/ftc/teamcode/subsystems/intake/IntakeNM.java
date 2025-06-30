package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotNM;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class IntakeNM {

    private PriorityMotor activeIntake;
    private RobotNM robot;

    public IntakeNM(RobotNM robot) {
        this.robot = robot;
        activeIntake = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "activeIntake"), "activeIntake", 5, 5, robot.sensors);
        robot.hardwareQueue.addDevice(activeIntake);

    }

    public void update() {



    }

}
