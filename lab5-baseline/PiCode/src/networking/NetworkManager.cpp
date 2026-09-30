/**
 * @file NetworkManager.cpp
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
 * This file defines the implementation for the Network Manager.  The Network Manager manages network connections and acts as a server, receiving messages sent over a socket.
 */

#include "NetworkManager.h"
#include "NetworkCfg.h"
#include "CommandQueue.h"
#include "NetworkMessage.h"
#include "NetworkCommands.h"
#include "NetworkCommandTypes.h"
#include "NetworkDebugLibrary.h"
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string>

namespace SWE4211RPi {

/**
 * Constructs a NetworkManager that listens on the given port and routes
 * validated messages into the provided array of command queues.
 */
NetworkManager::NetworkManager(unsigned short port, CommandQueue *queues[],
		std::string threadName) :
		RunnableClass(threadName),
		portNumber(port),
		referencequeues(queues) {
}

/**
 * Ensures any open sockets are closed if run() did not finish cleanly.
 */
NetworkManager::~NetworkManager() {
	if (connectedSocket >= 0) {
		close(connectedSocket);
		connectedSocket = -1;
	}
	if (server_fd >= 0) {
		close(server_fd);
		server_fd = -1;
	}
}

/**
 * @return File descriptor of the connected client socket, or -1 if none.
 */
int NetworkManager::getSocketID() {
	return connectedSocket;
}

/**
 * Receives one complete networkMessageStruct.  Fields remain in network byte order.
 * @return true on a full message; false on disconnect, partial read, or error.
 */
bool NetworkManager::receiveNetworkMessage(networkMessageStruct &message) {
	memset(&message, 0, sizeof(message));

	ssize_t bytesRead = recv(connectedSocket, &message, sizeof(message),
			MSG_WAITALL);

	if (bytesRead < 0) {
		if (keepGoing) {
			perror("recv");
		}
		return false;
	}
	if (bytesRead == 0) {
		// Client closed the connection.
		return false;
	}
	if (static_cast<size_t>(bytesRead) != sizeof(message)) {
		fprintf(stderr,
				"Incomplete network message (%zd of %zu bytes); closing connection.\n",
				bytesRead, sizeof(message));
		return false;
	}

	return true;
}

/**
 * Converts endianness, verifies checksum/type/destination, and enqueues valid commands.
 * @return The calculated XOR checksum.
 */
int NetworkManager::processReceivedMessage(networkMessageStruct &receivedMessage) {
	int calculatedChecksum = 0;

	receivedMessage.securityMode = ntohl(receivedMessage.securityMode);

	if (receivedMessage.securityMode == 0) {
		receivedMessage.messageID = ntohl(receivedMessage.messageID);
		receivedMessage.timestampHigh = ntohl(receivedMessage.timestampHigh);
		receivedMessage.timestampLow = ntohl(receivedMessage.timestampLow);
		receivedMessage.messageType = ntohl(receivedMessage.messageType);
		receivedMessage.messageDestination = ntohl(
				receivedMessage.messageDestination);
		receivedMessage.message = ntohl(receivedMessage.message);
		receivedMessage.parameter1 = ntohl(receivedMessage.parameter1);
		receivedMessage.parameter2 = ntohl(receivedMessage.parameter2);
		receivedMessage.xorChecksum = ntohl(receivedMessage.xorChecksum);

		calculatedChecksum = receivedMessage.securityMode
				^ receivedMessage.messageID ^ receivedMessage.timestampHigh
				^ receivedMessage.timestampLow ^ receivedMessage.messageType
				^ receivedMessage.messageDestination ^ receivedMessage.message
				^ receivedMessage.parameter1 ^ receivedMessage.parameter2;

		printNetworkMessageToConsole(&receivedMessage);

		if (static_cast<uint32_t>(calculatedChecksum)
				== receivedMessage.xorChecksum) {
			if (receivedMessage.messageType == COMMAND_MSG_TYPE
					&& receivedMessage.messageDestination >= 1
					&& receivedMessage.messageDestination
							<= NUMBER_OF_QUEUES) {
				CommandQueueEntry entry;
				entry.commandType = 0;
				entry.command = receivedMessage.message;
				entry.parameter1 = receivedMessage.parameter1;
				entry.parameter2 = receivedMessage.parameter2;
				referencequeues[receivedMessage.messageDestination - 1]->enqueue(
						entry);
			}
		}
	}

	return calculatedChecksum;
}

/**
 * Broadcasts a REPORT_NEW_IPADDRESS entry on every command queue.
 * @param clientAddress Client address; IP of 0 means no client is connected.
 */
void NetworkManager::reportNewClientConnection(
		const struct sockaddr_in &clientAddress) {
	uint32_t ipAddress = ntohl(clientAddress.sin_addr.s_addr);

	CommandQueueEntry entry;
	entry.commandType = BROADCAST_MESSAGE_TYPE;
	entry.command = REPORT_NEW_IPADDRESS;
	entry.parameter1 = ipAddress;
	entry.parameter2 = 0;

	for (int i = 0; i < NUMBER_OF_QUEUES; i++) {
		referencequeues[i]->enqueue(entry);
	}
}

/**
 * Opens a TCP listening socket on portNumber and accepts one client at a time,
 * validating and routing each received message until stop is requested.
 */
void NetworkManager::run() {
	server_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (server_fd < 0) {
		perror("socket");
		return;
	}

	int opt = 1;
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))
			< 0) {
		perror("setsockopt SO_REUSEADDR");
		close(server_fd);
		server_fd = -1;
		return;
	}
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt))
			< 0) {
		perror("setsockopt SO_REUSEPORT");
		close(server_fd);
		server_fd = -1;
		return;
	}

	struct sockaddr_in address;
	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(portNumber);

	if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0) {
		perror("bind");
		close(server_fd);
		server_fd = -1;
		return;
	}

	if (listen(server_fd, 3) < 0) {
		perror("listen");
		close(server_fd);
		server_fd = -1;
		return;
	}

	while (keepGoing) {
		struct sockaddr_in their_address;
		socklen_t addrlen = sizeof(their_address);

		connectedSocket = accept(server_fd, (struct sockaddr*) &their_address,
				&addrlen);

		if (connectedSocket < 0) {
			if (!keepGoing) {
				break;
			}
			if (errno == EINTR) {
				continue;
			}
			perror("accept");
			break;
		}

		int keepAlive = 1;
		if (setsockopt(connectedSocket, SOL_SOCKET, SO_KEEPALIVE, &keepAlive,
				sizeof(keepAlive)) < 0) {
			perror("setsockopt SO_KEEPALIVE");
		}

		reportNewClientConnection(their_address);

		while (keepGoing) {
			networkMessageStruct message;
			if (receiveNetworkMessage(message)) {
				processReceivedMessage(message);
			} else {
				break;
			}
		}

		close(connectedSocket);
		connectedSocket = -1;

		struct sockaddr_in zeroAddress;
		memset(&zeroAddress, 0, sizeof(zeroAddress));
		reportNewClientConnection(zeroAddress);
	}

	if (server_fd >= 0) {
		close(server_fd);
		server_fd = -1;
	}
}

/**
 * Stops the thread and shuts down sockets so blocked accept()/recv() unblock.
 */
void NetworkManager::stopThreadExecution() {
	RunnableClass::stopThreadExecution();

	if (connectedSocket >= 0) {
		shutdown(connectedSocket, SHUT_RDWR);
	}
	if (server_fd >= 0) {
		shutdown(server_fd, SHUT_RDWR);
	}
}

}
