package org.firstinspires.ftc.teamcode.subsystems.deposit;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class Claw {

    public final nPriorityServo mode;
    public final nPriorityServo pitch;

    static final double extendPos = 1.0; // TODO: find out the extended position
    static final double retractPos = 0.0; // TODO: find out the retracted position

    public Claw(Robot robot) {

        mode = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawMode")},
                "clawMode",
                nPriorityServo.ServoType.AXON_MINI, // TODO: find out what type of servo is used for the claw opening and closing
                0.335, // TODO: find out the actual numbers for these parameters below
                0.982, // TODO
                0.871, // TODO
                new boolean[] {false}, // TODO: this is an orientation thing I think, find out what it is
                1.0,
                2.0
        );
        pitch = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "clawPitch")},
                "clawPitch",
                nPriorityServo.ServoType.AXON_MINI, // TODO: find out what type of servo is used for the claw opening and closing
                0.335, // TODO: find out the actual numbers for these parameters below
                0.982, // TODO
                0.871, // TODO
                new boolean[] {false}, // TODO: this is an servo orientation thing I think, find out what it is
                1.0,
                2.0
        );

        robot.hardwareQueue.addDevices(mode, pitch);
    }

    public void open() {
        mode.setTargetPos(extendPos, 1.0);
    }

    public void close() {
        mode.setTargetPos(retractPos, 1.0);
    }

    public void rotateToTransfer(double armAngle) { // TODO: this assumes that the claw is parallel to the floor during transfer, idk what else it could be but need to confirm this
        pitch.setTargetAngle(-armAngle, 1.0);
    }

    public void rotateToPixelTransfer(double armAngle) {
        // targetAngle + armAngle = pi / 2, assuming the 0 angle is the x axis elevated to h knot
        // logic: the total angle should have the claw aligned to the walls for pixel deposit
        pitch.setTargetAngle(Math.PI / 2 - armAngle, 1.0);
    }

    public void rotateToSampleDeposit(double armAngle) { // this will do the same thing as rotateToTransfer() since it makes the sample drop from the claw directly perpendicular to the claw alignment for ease
        pitch.setTargetAngle(-armAngle, 1.0);
    }

    public boolean inPosition() {
        return mode.inPosition() && pitch.inPosition();
    }

}
