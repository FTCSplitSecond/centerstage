package org.firstinspires.ftc.teamcode.util

import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason

open class Dynamic(val block: Component.() -> Component): Component() {
    lateinit private var underlyingComponent : Component
    override fun start() {
        underlyingComponent = block()
        underlyingComponent.start()
    }

    override fun isComplete(): Boolean {
        return underlyingComponent.isComplete()
    }
    override fun loop() {
        underlyingComponent.loop()
    }
    override fun end(reason: FinishReason) {
        underlyingComponent.end(reason)
    }
}
fun dynamic(block: Component.() -> Component) = Dynamic(block)