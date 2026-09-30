/**
 * @file CommandQueue.cpp
 * @author  Walter Schilling (schilling@msoe.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
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
 * Implementation of the CommandQueue class.
 * The CommandQueue is a thread-safe, first-in first-out (FIFO) queue.  One or more threads
 * (producers) enqueue commands for a device, and another thread (the consumer) dequeues and
 * processes them.  Each command is stored in a CommandQueueEntry structure.
 * Two synchronization objects work together to make the queue safe to share between threads:
 * a mutex, which ensures that only one thread at a time modifies the underlying queue, and a
 * counting semaphore, which tracks how many items are waiting so that a consumer can block
 * until work is available.
 */

#include "CommandQueue.h"
#include <semaphore.h>

namespace SWE4211RPi {

/**
 * This is the default constructor, which creates an empty instance of the queue.
 * The counting semaphore is initialized to zero so that a consumer blocks until
 * the first item is enqueued.  The semaphore is shared only among threads of this process.
 */
CommandQueue::CommandQueue() {
	sem_init(&queueCountSemaphore, 0, 0);
}

/**
 * The destructor releases the counting semaphore initialized in the constructor.
 */
CommandQueue::~CommandQueue() {
	sem_destroy(&queueCountSemaphore);
}

/**
 * Indicates whether the queue currently contains at least one item ready to dequeue.
 * Uses the queue's own state (not the semaphore).  Result is a snapshot only.
 * @return true if there is at least one item on the queue; false otherwise.
 */
bool CommandQueue::hasItem() {
	std::lock_guard<std::mutex> lock(queueMutex);
	return !commandQueueContents.empty();
}

/**
 * Removes and returns the next command from the front of the queue.
 * Blocks on the counting semaphore if the queue is empty until another thread enqueues.
 * @return The next command to be processed.
 */
CommandQueueEntry CommandQueue::dequeue() {
	CommandQueueEntry retVal = {0, 0, 0, 0};

	// Block until an item is available.
	sem_wait(&queueCountSemaphore);

	{
		std::lock_guard<std::mutex> lock(queueMutex);
		retVal = commandQueueContents.front();
		commandQueueContents.pop();
	}

	return retVal;
}

/**
 * Enqueues a command.  Unblocks a thread waiting in dequeue() if one is blocked.
 * @param value The command queue entry to enqueue.
 */
void CommandQueue::enqueue(CommandQueueEntry value) {
	{
		std::lock_guard<std::mutex> lock(queueMutex);
		commandQueueContents.push(value);
	}
	sem_post(&queueCountSemaphore);
}

}
