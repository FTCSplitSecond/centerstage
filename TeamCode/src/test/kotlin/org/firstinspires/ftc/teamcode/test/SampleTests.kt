package org.firstinspires.ftc.teamcode.test

import io.kotest.core.spec.style.ShouldSpec
import io.kotest.matchers.shouldBe

class SampleTests: ShouldSpec({
    context("A Sample Test") {
        should("should pass a test") {
            val testResult = true
            testResult shouldBe true
        }
    }
})