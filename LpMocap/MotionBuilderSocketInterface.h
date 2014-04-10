#include <winsock.h>
#include <windows.h>

#define kFBTCPIP_Stream 0

typedef int FBTCPIPSocketType;
typedef unsigned long kULong;

const char LPMB_READY = 0x00;
const char LPMB_GET_INFO = 0x01;
const char LPMB_GET_DATA = 0x02;
const char LPMB_DISCONNECT = 0x03;

class FBTCPIP {
public:
	bool CreateSocket(int &pSocket, FBTCPIPSocketType pType, const char *pProtocol="ip", bool pNonBlocking=true) 
	{
		WSADATA wsadata;
		int error;

		error = WSAStartup(0x0202, &wsadata);

		if (error) {
			return false;
		}

		if (wsadata.wVersion != 0x0202) {
			WSACleanup();
			return false;
		}

		socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (socket_ == INVALID_SOCKET) {
			return false;
		}
		
		return true;
	}
	
	bool Connect(int &pSocket, const char *pAddr, int pPort) 
	{
		SOCKADDR_IN target;
		
		target.sin_family = AF_INET;
		target.sin_port = htons (pPort);
		target.sin_addr.s_addr = inet_addr (pAddr);	
	
		if (::connect(socket_, (SOCKADDR *)&target, sizeof(target)) == SOCKET_ERROR) {
			return false;
		} else {
			pSocket = socket_;
		
			return true;
		}
	}
	
	bool CloseSocket(int &pSocket) {
		if (socket_) closesocket(socket_);

		WSACleanup();
		
		return true;
	}
	
	bool Write(int pSocket, void *lpBuffer, int nNumberOfBytesToWrite, int *lpNumberOfBytesWritten=NULL)
	{
		send(socket_, (char *)lpBuffer, nNumberOfBytesToWrite, 0);

		return true;		
	}
	
	bool ReadBlocking(int pSocket, void *lpBuffer, int nNumberOfBytesToRead, int *lpNumberOfBytesRead=NULL, kULong pTimeOut=500)
	{
		recv(socket_, (char *)lpBuffer, nNumberOfBytesToRead, 0);
		
		return true;
	}
	
public:
	SOCKET socket_;
};