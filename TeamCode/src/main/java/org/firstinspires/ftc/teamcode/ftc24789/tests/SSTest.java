package org.firstinspires.ftc.teamcode.ftc24789.tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SSTest {
    private final String description;

    SSTest(String description) {
        this.description = description;
    }

    public abstract void run(boolean on, Telemetry telemetry);
}
