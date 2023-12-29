package org.firstinspires.ftc.teamcode.roadrunner.drive

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

// implementing gyrodometry from this paper https://public.websites.umich.edu/~johannb/Papers/paper63.pdf
class Gyrodometry(private val gyroHeadingProvider : () -> Double,
                  private val odoHeadingProvider : () -> Double) {
    private val threshold : Double = Math.toRadians(0.125)
    private var lastGyroHeading = 0.0
    private var lastOdoHeading = 0.0
    private var lastHeading = 0.0
    private val timer = ElapsedTime()
    var heading = 0.0
        private set(value) {
            field = value
            lastHeading = value
        }
    init {
        timer.reset()
    }
    fun updateForGryrodometry() {
        val gyroHeading = gyroHeadingProvider()
        val odoHeading = odoHeadingProvider()
        val deltaT = timer.seconds()

        val deltaGyro = AngleUnit.normalizeRadians(gyroHeading - lastGyroHeading) / deltaT
        val deltaOdo = AngleUnit.normalizeRadians(odoHeading - lastOdoHeading) / deltaT
        val deltaGOHeading = deltaGyro - deltaOdo

        lastGyroHeading = gyroHeading
        lastOdoHeading = odoHeading
        timer.reset()

        heading = if (Math.abs(deltaGOHeading) > threshold) {
            AngleUnit.normalizeRadians(lastHeading + (deltaGyro * deltaT))
        } else {
            AngleUnit.normalizeRadians(lastHeading + (deltaOdo * deltaT))
        }
    }
}