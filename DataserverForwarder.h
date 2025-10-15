#ifndef DATASERVER_FORWARDER_H
#define DATASERVER_FORWARDER_H

#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstring>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "SharedQueue.hpp"

// Data point structure for queuing
struct DataPoint {
    std::string name;
    int dtype;
    std::vector<uint8_t> data;
    
    DataPoint() = default;
    
    DataPoint(const std::string& n, int dt, const void* d, size_t len)
        : name(n), dtype(dt), data((uint8_t*)d, (uint8_t*)d + len) {}
};

// Forward declarations
extern SharedQueue<DataPoint> ds_forward_queue;

class DataserverForwarder {
private:
    int sockfd;
    std::string server_address;
    int server_port;
    std::atomic<bool> connected;
    std::atomic<bool> should_run;
    std::atomic<bool> drain_only;  // just drain queue without sending
    std::thread forward_thread;
    
    static constexpr uint8_t BINARY_MSG_CHAR = '>';
    static constexpr size_t FIXED_LENGTH = 128;
    static constexpr int RECONNECT_DELAY_MS = 5000;
    
public:
    enum DataType {
        BYTE = 0, STRING, FLOAT, DOUBLE, SHORT, INT,
        DG, SCRIPT, TRIGGER_SCRIPT, EVT, NONE, UNKNOWN
    };
    
    DataserverForwarder(const std::string& addr, int port = 4620) 
        : sockfd(-1), server_address(addr), server_port(port), 
          connected(false), should_run(false), drain_only(false) {}
    
    ~DataserverForwarder() {
        stop();
        disconnect();
    }

  bool connect() {
    if (connected.load()) return true;
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) return false;
    
    // Set non-blocking mode BEFORE connect
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
    
    struct hostent* host = gethostbyname(server_address.c_str());
    if (!host) {
      close(sockfd);
      sockfd = -1;
      return false;
    }
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(server_port);
    memcpy(&addr.sin_addr, host->h_addr, host->h_length);
    
    int result = ::connect(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    
    if (result < 0 && errno != EINPROGRESS) {
      close(sockfd);
      sockfd = -1;
      return false;
    }
    
    // Wait for connection with select (interruptible)
    fd_set write_fds;
    struct timeval timeout = {2, 0};  // 2 second timeout
    
    FD_ZERO(&write_fds);
    FD_SET(sockfd, &write_fds);
    
    result = select(sockfd + 1, NULL, &write_fds, NULL, &timeout);
    
    if (result <= 0) {
      // Timeout or error
      close(sockfd);
      sockfd = -1;
      return false;
    }
    
    // Check if connection succeeded
    int so_error;
    socklen_t len = sizeof(so_error);
    getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &so_error, &len);
    
    if (so_error != 0) {
      close(sockfd);
      sockfd = -1;
      return false;
    }
    
    // Set back to blocking mode
    fcntl(sockfd, F_SETFL, flags & ~O_NONBLOCK);
    
    // Disable Nagle's algorithm
    int flag = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    
    connected.store(true);
    std::cout << "DataserverForwarder: Connected to " << server_address 
              << ":" << server_port << std::endl;
    return true;
  }  
  
  void disconnect() {
        if (sockfd >= 0) {
            close(sockfd);
            sockfd = -1;
        }
        connected.store(false);
    }
    
    bool isConnected() const { return connected.load(); }

  
  bool sendDataPoint(const DataPoint& dp) {
    if (drain_only.load()) {
      return true;
    }
    
    if (!connected.load() || sockfd < 0) return false;  
    
    char buf[FIXED_LENGTH];
    memset(buf, 0, FIXED_LENGTH);  // Zero the entire buffer
    
    uint16_t varlen = dp.name.length();
    uint64_t timestamp =
      std::chrono::duration_cast<std::chrono::microseconds>(
							    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    size_t idx = 0;
    buf[idx++] = BINARY_MSG_CHAR;  // '>'
    
    memcpy(&buf[idx], &varlen, sizeof(uint16_t)); 
    idx += sizeof(uint16_t);
    
    memcpy(&buf[idx], dp.name.c_str(), varlen); 
    idx += varlen;
    
    memcpy(&buf[idx], &timestamp, sizeof(uint64_t)); 
    idx += sizeof(uint64_t);
    
    uint32_t dt = dp.dtype;
    uint32_t dl = dp.data.size();
    
    memcpy(&buf[idx], &dt, sizeof(uint32_t)); 
    idx += sizeof(uint32_t);
    
    memcpy(&buf[idx], &dl, sizeof(uint32_t)); 
    idx += sizeof(uint32_t);
    
    // Make sure we don't overflow the buffer
    if (idx + dp.data.size() > FIXED_LENGTH) {
      std::cerr << "Data too large: " << (idx + dp.data.size()) 
		<< " > " << FIXED_LENGTH << std::endl;
      return false;
    }
    
    memcpy(&buf[idx], dp.data.data(), dp.data.size());
    // Rest of buffer is already zeroed
    
    // Send exactly 128 bytes
    ssize_t sent = send(sockfd, buf, FIXED_LENGTH, MSG_NOSIGNAL);
    if (sent < 0) {
      std::cerr << "Send failed: " << strerror(errno) << std::endl;
      if (errno == EPIPE || errno == ECONNRESET || errno == ENOTCONN) {
	connected.store(false);
	disconnect();
      }
      return false;
    }
    
    if (sent != FIXED_LENGTH) {
      std::cerr << "Incomplete send: " << sent << " of " << FIXED_LENGTH << std::endl;
      return false;
    }
    
    return true;
  }
  
  void start() {
    drain_only.store(false);
    should_run.store(true);
    forward_thread = std::thread([this]() {
      while (should_run.load()) {
	// Auto-reconnect
	if (!connected.load()) {
	  if (connect()) {
	    std::cout << "DataserverForwarder: Connection established" << std::endl;
	  } else {
	    std::this_thread::sleep_for(
					std::chrono::milliseconds(RECONNECT_DELAY_MS));
	    continue;
	  }
	}
        
	// Process queue
	if (ds_forward_queue.size() > 0) {
	  DataPoint dp = ds_forward_queue.front();
	  ds_forward_queue.pop_front();
          
	  if (!sendDataPoint(dp)) {
	    // Connection lost - requeue and reconnect
	    ds_forward_queue.push_front(dp);
	    connected.store(false);
	  }
	} else {
	  // No data, brief sleep
	  std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
      }
    });
  }

  void startDrainOnly() {
    drain_only.store(true);
    should_run.store(true);
    forward_thread = std::thread([this]() {
      while (should_run.load()) {
	// Just drain the queue to prevent memory buildup
	if (ds_forward_queue.size() > 0) {
	  DataPoint dp = ds_forward_queue.front();
	  ds_forward_queue.pop_front();
	  // Discard the data
	} else {
	  std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
      }
    });
  }
  
  void stop() {
    should_run.store(false);
    if (forward_thread.joinable()) {
      forward_thread.join();
    }
  }
};

#endif // DATASERVER_FORWARDER_H
