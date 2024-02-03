package org.firstinspires.ftc.teamcode.robot.commands

import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeState
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

class MoveToPlace(val robot : Robot) : SetTelescopePosition(robot.telescope, TelescopeState.TRAVEL)