package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.DriveHWNikhil;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.sensors.SensorsNM;
import org.firstinspires.ftc.teamcode.subsystems.deposit.DepositNM;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeNM;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

import java.util.function.BooleanSupplier;

public class RobotNM {

    public final HardwareMap hardwareMap;
    public final HardwareQueue hardwareQueue;
    public final SensorsNM sensors;
    public final IntakeNM intake;
    public final DepositNM deposit;

    private BooleanSupplier stopChecker = null;


    public RobotNM(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();
        this.sensors = new SensorsNM(this);
        deposit = new DepositNM(this);
        intake = new IntakeNM(this);
        TelemetryUtil.setup();
        LogUtil.reset();

    }


    public void update() {
        START_LOOP();

        if (stopChecker != null && stopChecker.getAsBoolean()) {
            hardwareQueue.update();
            return;
        }

        sensors.update();

        intake.update();
        deposit.update();

        hardwareQueue.update();

    }

    public void setStopChecker(BooleanSupplier stopChecker) {
        this.stopChecker = stopChecker;
    }
}
