#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <errno.h>

#include <iostream>

#include <ros/ros.h>

#include "udp_data.h"

namespace jmc_auto {
namespace udp{

UdpStream::UdpStream(const char* address,
                     uint16_t port,
                     uint32_t timeout_usec) :
    _sockfd(-1),
    _errno(0) {
    _peer_addr = inet_addr(address);
    _peer_port = htons(port);
    _timeout_usec = timeout_usec;
    // call open or call open in connect later

}

UdpStream::~UdpStream() {
    this->close();
}

void UdpStream::open() {
    int fd = -1;

    fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        // error
        ROS_ERROR_STREAM("Create socket failed, errno: " << errno << ", " << strerror(errno));
        return ;
    }

    // block or not block
    if (_timeout_usec != 0) {
        int flags  = fcntl(fd, F_GETFL, 0);
        if (flags == -1) {
            ::close(fd);
            ROS_ERROR_STREAM("fcntl get flag failed, errno: " << errno << ", " << strerror(errno));
            return ;
        }

        if (fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
            ::close(fd);
            ROS_ERROR_STREAM("fcntl set block failed, errno: " << errno << ", " << strerror(errno));
            return ;
        }

        struct timeval block_to = {_timeout_usec / 1000000,
                                                     _timeout_usec % 1000000};
        if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                       (char *)&block_to, sizeof(block_to)) < 0) {
            ::close(fd);
            ROS_ERROR_STREAM("setsockopt set rcv timeout failed, errno: "
                       << errno << ", " << strerror(errno));
            return ;
        }

        if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
                            (char *)&block_to, sizeof(block_to)) < 0) {
            ::close(fd);
            ROS_ERROR_STREAM("setsockopt set snd timeout failed, errno: "
                       << errno << ", " << strerror(errno));
            return ;
        }
    } else {
        int flags  = fcntl(fd, F_GETFL, 0);
        if (flags == -1) {
            ::close(fd);
            ROS_ERROR_STREAM("fcntl get flag failed, errno: " << errno << ", " << strerror(errno));
            return ;
        }

        if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
            ::close(fd);
            ROS_ERROR_STREAM("fcntl set non block failed, errno: "
                             << errno << ", " << strerror(errno));
            return ;
        }
    }
    _sockfd = fd;
    return ;
}

void UdpStream::close() {
    if (_sockfd > 0) {
        ::close(_sockfd);
        _sockfd = -1;
        _status = Status::DISCONNECTED;
    }
}

bool UdpStream::connect() {
    if (_sockfd < 0) {
        this->open();
        if (_sockfd < 0) {
            return false;
        }
    }

    if (_status == Status::CONNECTED) {
        return true;
    }

    // upper layer support ping method ??
    //login();
    _status =Status::CONNECTED;
    return true;
}

bool UdpStream::disconnect() {
    if (_sockfd <  0) {
        // not open
        return false;
    }

    this->close();
    return true;
}

size_t UdpStream::read(uint8_t* buffer, size_t max_length) {
    ssize_t ret = 0;
    struct sockaddr_in peer_sockaddr;
    socklen_t socklenth = sizeof(peer_sockaddr);
    bzero(&peer_sockaddr, sizeof(peer_sockaddr));
    peer_sockaddr.sin_family = AF_INET;
    peer_sockaddr.sin_port = _peer_port;
    //peer_sockaddr.sin_port = 8080;
    peer_sockaddr.sin_addr.s_addr = _peer_addr;
    while ((ret = ::recvfrom(_sockfd,
                             buffer,
                             max_length,
                             0,
                             (struct sockaddr*)&peer_sockaddr,
                             (socklen_t*)&socklenth)) < 0) {
        if (errno == EINTR) {
            continue;
        } else {
            // error
            if (errno != EAGAIN) {
                _status = Status::ERROR;
                _errno = errno;
            }
        }

        return 0;
    }
	ROS_INFO("Reci port is %d.\n",peer_sockaddr.sin_port);
    return ret;
}

size_t UdpStream::write(const uint8_t* data, size_t length) {
    ssize_t nsent = 0;
    size_t total_nsent = 0;
    struct sockaddr_in peer_sockaddr;
    bzero(&peer_sockaddr, sizeof(peer_sockaddr));
    peer_sockaddr.sin_family = AF_INET;
    peer_sockaddr.sin_port = _peer_port;
    peer_sockaddr.sin_addr.s_addr = _peer_addr;

    while (length > 0) {
        nsent = ::sendto(_sockfd, data, length, 0,
                                 (struct sockaddr*)&peer_sockaddr, 
                                 (socklen_t) sizeof(peer_sockaddr));
        if (nsent < 0) { // error
            if (errno == EINTR) {
                continue;
            } else {
                // error
                if (errno == EPIPE || errno == ECONNRESET) {
                    _status = Status::DISCONNECTED;
                    _errno = errno;
                } else if (errno != EAGAIN) {
                    _status = Status::ERROR;
                    _errno = errno;
                }
                return total_nsent;
            }
        }

        total_nsent += nsent;
        length -= nsent;
        data += nsent;
    }

    return total_nsent;
}


} // namespace gnss
} // namespace drivers
