#ifndef _UDP_DATA_H_
#define	_UDP_DATA_H_

namespace jmc_auto {
namespace udp{


class UdpStream 
{
	typedef uint16_t  be16_t;
	typedef uint32_t  be32_t;
	enum class Status 
		{
	        DISCONNECTED,
	        CONNECTED,
	        ERROR,
	    };

public:
    UdpStream(const char* address, uint16_t port, uint32_t timeout_usec);
    ~UdpStream();

    //virtual bool connect();
    //virtual bool disconnect();
    //virtual size_t read(uint8_t* buffer, size_t max_length);
    //virtual size_t write(const uint8_t* data, size_t length);
	void open();
	bool connect();
    bool disconnect();
    size_t read(uint8_t* buffer, size_t max_length);
    size_t write(const uint8_t* data, size_t length);
	void close();
private:
    UdpStream() {}
    be16_t _peer_port = 0;
    be32_t _peer_addr = 0;
    uint32_t _timeout_usec = 0;
    int _sockfd = -1;
    int _errno = 0;
	int	band_flag=-1;
protected:
	Status _status= Status::DISCONNECTED;

};

} // namespace gnss
} // namespace drivers

#endif
