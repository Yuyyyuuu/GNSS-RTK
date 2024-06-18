#include<stdio.h>
#include<windows.h>

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)



// SocketÍøÂç
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);
void CloseSocket(SOCKET& sock);
