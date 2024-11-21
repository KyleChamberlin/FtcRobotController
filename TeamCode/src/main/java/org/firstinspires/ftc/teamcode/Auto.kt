package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
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

@Autonomous(group = "Into The Deep", name = "Specimen - Don't Park", preselectTeleOp = "Dave" )
class SpecimenNoPark : LinearOpMode() {
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

@Autonomous(group = "Into The Deep", name = "Sample", preselectTeleOp = "Dave" )
class Sample : LinearOpMode() {
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