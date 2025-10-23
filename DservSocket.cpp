#include "DservSocket.h"

#ifndef _WIN32
#include <unistd.h>
#endif

// Implementation of the static client process handler
void DservSocket::ds_client_process(DservSocket* instance, int sockfd)
{
  char buf[16384];
  int rval;

  //  std::cout << "starting tcp_client_process: " << std::to_string(sockfd) << std::endl;

  std::string dpoint_str;
#ifndef _MSC_VER  
  while ((rval = read(sockfd, buf, sizeof(buf))) > 0) {
#else
    while ((rval = recv(sockfd, buf, sizeof(buf), 0)) > 0) {
#endif
    for (int i = 0; i < rval; i++) {
      char c = buf[i];
      if (c == '\n') {

	if (dpoint_str.length() > 0) {
	  // Use instance->ds_queue to access the queue
	  if (instance->ds_queue != nullptr) {
	    instance->ds_queue->push_back(std::move(dpoint_str));
	  }
	}
	dpoint_str = "";
      }
      else {
	dpoint_str += c;
      }
    }
  }
  
  // Close the socket
#ifndef _MSC_VER
  close(sockfd);
#else
  closesocket(sockfd);
#endif
}
