package org.firstinspires.ftc.teamcode.robot.commands

import com.arcrobotics.ftclib.command.CommandBase

abstract class ConfigurableCommandBase() : CommandBase() {

    private var command : CommandBase? = null

    /**
     * If you override this method, you must call super.initialize() at the beginning of your method.
     */
    override fun initialize() {
        command = configure()
        if(null != command)
            command!!.initialize()

    }

    /**
     * If you override this method, you must call super.initialize() at the beginning of your method.
     */
    override fun execute() {
        if(null != command)
            command!!.execute()
    }

    /**
     * If you override this method, you must call super.initialize() at the beginning of your method.
     */
    override fun isFinished(): Boolean {
        return if(null != command)
            command!!.isFinished()
        else
            true
    }

    /**
     * If you override this method, you must call super.initialize() at the beginning of your method.
     */
    override fun end(interrupted: Boolean) {
        if(null != command)
           command!!.end(interrupted)
    }

    /**
     * This function is called when the command is initialized. It is
     * used to configure a command or commandgroup that will be executed.
     * The reason for this is so that the command can be configured
     * just before it is executed.
     *
     * This function must return a CommandBase object.
     */
    abstract fun configure() : CommandBase

}