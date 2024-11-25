package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.helpers.bulkCachingMode
import org.firstinspires.ftc.teamcode.intothedeep.Dave

@Autonomous(group = "Into The Deep", name = "Park Right", preselectTeleOp = "Dave" )
class ParkRight : LinearOpMode() {
    val dave by lazy { Dave(hardwareMap, telemetry) }

    override fun runOpMode() {
        hardwareMap.bulkCachingMode = AUTO
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        dave.initialize()

        waitForStart()

        dave.autoStrafeFor(2000)
    }
}

@Autonomous(group = "Into The Deep", name = "Specimen - Park Right", preselectTeleOp = "Dave" )
class SpecimenParkRight : LinearOpMode() {
    val dave by lazy { Dave(hardwareMap, telemetry) }

    override fun runOpMode() {
        hardwareMap.bulkCachingMode = AUTO
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        dave.initialize()

        waitForStart()

        dave.autoStrafeFor(2000)
    }
}

object AutoVars{

}

@Autonomous(group = "Into The Deep", name = "Specimen - Don't Park", preselectTeleOp = "Dave" )
class SpecimenNoPark : LinearOpMode() {
    val dave by lazy { Dave(hardwareMap, telemetry) }

    override fun runOpMode() {
        hardwareMap.bulkCachingMode = AUTO
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        dave.initialize()

        dave.closePincer()

        waitForStart()

//        sleep(10000)

        dave.autoForwardFor(1550)

        dave.armTo(-1950)
        dave.extensionTo(3900)
        dave.wristPosition = 0.7

        sleep(500)
        dave.wristPosition = 0.8
        dave.extensionTo(3000)
        sleep(600)
        dave.openPincer()
        dave.autoBackwardFor(1000)
        dave.extensionTo(0)
        dave.armTo(0)
        sleep(100000)

    }
}

@Autonomous(group = "Into The Deep", name = "Sample", preselectTeleOp = "Dave" )
class Sample : LinearOpMode() {
    val dave by lazy { Dave(hardwareMap, telemetry) }

    override fun runOpMode() {
        hardwareMap.bulkCachingMode = AUTO
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        dave.initialize()

        waitForStart()

        dave.wrist.position = 0.0
    }
}