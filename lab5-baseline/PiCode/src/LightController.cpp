/**
 * @file LightController.cpp
 * @author  Walter Schilling (schilling@msoe.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * This code is developed as part of the MSOE SWE4211 Real Time Systems course,
 * but can be freely used by others.
 *
 * SWE4211 Real Time Systems is a required course for students studying the
 * discipline of software engineering.
 *
 * This Software is provided under the License on an "AS IS" basis and
 * without warranties of any kind concerning the Software, including
 * without limitation merchantability, fitness for a particular purpose,
 * absence of defects or errors, accuracy, and non-infringement of
 * intellectual property rights other than copyright. This disclaimer
 * of warranty is an essential part of the License and a condition for
 * the grant of any rights to this Software.
 *
 * @section DESCRIPTION
 *
 *      This class is a controller for the light.  It is responsible for managing the operation of the lights based upon incoming commands
 *      from the command queue.
 */

#include "CommandQueue.h"
#include "LightController.h"
#include "NetworkCommands.h"
#include <time.h>

using namespace SWE4211RPi;

/**
 * Instantiates a light controller for one LED and its paired pushbutton.
 * The LED starts off (active-low, so GPIO high).  All reference members are
 * initialized in the member initializer list.
 */
LightController::LightController(int gpioOutPin, int gpioInPin,
		SWE4211RPi::CommandQueue& queue, std::string threadName, uint32_t period) :
		PeriodicTask(threadName, period),
		referencequeue(queue),
		light(GPIO::getInstance(gpioOutPin, GPIO::GPIO_OUT, GPIO::GPIO_HIGH)),
		pushbutton(GPIO::getInstance(gpioInPin, GPIO::GPIO_IN)) {
}

/**
 * Releases the GPIO instances obtained in the constructor.
 */
LightController::~LightController() {
	GPIO::freeInstance(light);
	GPIO::freeInstance(pushbutton);
}

/**
 * Invoked once per task period.  Drains waiting commands, then generates one
 * period of software PWM for the light if it should be lit (network on or button held).
 */
void LightController::taskMethod() {
	// Process every waiting command before updating the light.
	while (referencequeue.hasItem()) {
		CommandQueueEntry entry = referencequeue.dequeue();

		// Commands are bitmapped; check in the required order.
		if (entry.command & LIGHTOFFCMD) {
			lampOn = false;
		}
		if (entry.command & LIGHTONCMD) {
			lampOn = true;
		}
		if (entry.command & LIGHTPWMADJUSTMENTCMD) {
			uint32_t requested = entry.parameter1;
			// Message scale is 0–1000; store as whole percent 0–100.
			if (requested <= 1000) {
				dutyCycle = static_cast<int>(requested / 10);
			}
		}
		// Unrecognized commands (e.g. broadcast IP reports) are ignored.
	}

	// Active-low pushbutton: pressed when the pin reads low.
	bool buttonPressed = (pushbutton.getValue() == GPIO::GPIO_LOW);

	if (!lampOn && !buttonPressed) {
		// Light should stay off; nothing to do this period.
		return;
	}

	// Light the LED at the current duty cycle (active low = GPIO low).
	light.setValue(GPIO::GPIO_LOW);

	// Multiply before dividing to avoid truncating to zero.
	long onTimeUs = (static_cast<long>(getTaskPeriod()) * dutyCycle) / 100;

	struct timespec sleepDelay;
	sleepDelay.tv_sec = onTimeUs / 1000000L;
	sleepDelay.tv_nsec = (onTimeUs % 1000000L) * 1000L;
	nanosleep(&sleepDelay, NULL);

	// Off for the remainder of the period until the next invocation.
	light.setValue(GPIO::GPIO_HIGH);
}
