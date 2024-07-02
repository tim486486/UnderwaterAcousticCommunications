#include <iostream>
#include <string>
#include <conio.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define BUF_LEN 100

using namespace std;

char comport[10] = "COM";

// Declare variables and structures
HANDLE port;
HANDLE keyboard = GetStdHandle(STD_INPUT_HANDLE);
HANDLE screen = GetStdHandle(STD_OUTPUT_HANDLE);
HANDLE hFile;

DCB dcbSerialParams = {0};
COMMTIMEOUTS timeouts = {0};
DWORD mode;
DWORD read, written, written2;

int count = 0;

bool newFileFlag = false;
bool screenFlag = true;
bool fileFlag = false;
bool exitFlag = false;
char ch;
char buffer[BUF_LEN];
char fileName[100];
int bufIndex = 0;
unsigned long kbMax = 1024L; //Maximum size of a data file before creating a new one, in KB
unsigned long bytesWritten = 0L;

int openPort();
int closePort();
void system_error(char *name);
int loadTextFile();
/*
 *
 */
int main() {

	int com = 0;
	cout << "Please enter the COM Port of the Master: ";
	cin >> com;

	char comstr[10]; // enough to hold all numbers up to 64-bits
	sprintf(comstr, "%d", com);
	strcat(comport,comstr);

	cout << comport << " will be used.\n";

	// set keyboard to raw reading.
	if (!GetConsoleMode(keyboard, &mode)) system_error("getting keyboard mode");
	mode &= ~ ENABLE_PROCESSED_INPUT;
	if (!SetConsoleMode(keyboard, mode)) system_error("setting keyboard mode");

	if (openPort() == 0){
		do {
			// check for data on port and display it on screen.
			while (!ReadFile(port, buffer, sizeof(buffer), &read, NULL));

			if (read)
			{
				if (buffer[0] == ';'){ //start a new file
					fileFlag = true;
					newFileFlag = true;
				}

				if (buffer[0] == '*'){ //the system has been stopped, close the current file
					fileFlag = false;
					CloseHandle(hFile);
					bytesWritten = 0;
				}

				if(!newFileFlag){
					if (screenFlag) WriteFile(screen, buffer, read, &written, NULL);
					if (fileFlag) WriteFile(hFile, buffer, read, &written, NULL);
					bytesWritten += written;
				}

				if (newFileFlag || bytesWritten >= kbMax*1024L){
					CloseHandle(hFile);
					if (loadTextFile() == 1) exitFlag = true;
					bytesWritten = 0;
					newFileFlag = false;
				}
			}

			// check for keypress
			if (kbhit()){
				ch = getch();
				if (ch == '-') screenFlag = false; // -
				else if (ch == '+') screenFlag = true; // +
				else if (ch == 127) exitFlag = true; // ctrl+backspace
				else if (ch == 9){ // tab
					char inputBuffer[BUF_LEN];
					printf ("\nEnter a Command: ");
					scanf("%s", inputBuffer);
					if (!WriteFile(port, inputBuffer, strlen(inputBuffer), &written2, NULL)){
						fprintf(stderr, "Error\n");
						CloseHandle(port);
						exitFlag = true;
					}
				}
			}
		// until user hits ctrl-backspace.
		} while (!exitFlag);
	}

	// close up and go home.
	CloseHandle(keyboard);
	CloseHandle(port);
	CloseHandle(hFile);

	return 0;
}
/*
 *
 */
int openPort(){

	// Open the highest available serial port number
	fprintf(stderr, "Opening serial port...");
	port = CreateFile(
	            comport, GENERIC_READ|GENERIC_WRITE, 0, NULL,
	            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
	if (port == INVALID_HANDLE_VALUE)
	{
	        fprintf(stderr, "Error\n");
	        return 1;
	}
	else fprintf(stderr, "OK\n");

	// Set device parameters (19200 baud, 1 start bit,
	// 1 stop bit, no parity)
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (GetCommState(port, &dcbSerialParams) == 0)
	{
	    fprintf(stderr, "Error getting device state\n");
	    CloseHandle(port);
	    return 1;
	}

	dcbSerialParams.BaudRate = CBR_19200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if(SetCommState(port, &dcbSerialParams) == 0)
	{
	    fprintf(stderr, "Error setting device parameters\n");
	    CloseHandle(port);
	    return 1;
	}

	// Set COM port timeout settings
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if(SetCommTimeouts(port, &timeouts) == 0)
	{
	    fprintf(stderr, "Error setting timeouts\n");
	    CloseHandle(port);
	    return 1;
	}

	Sleep(1000);

	return 0;

}
/*
 *
 */
int closePort(){
	// Close serial port
	fprintf(stderr, "Closing serial port...");
	if (CloseHandle(port) == 0)
	{
		fprintf(stderr, "Error\n");
		return 1;
	}
	fprintf(stderr, "OK\n");

	// exit normally
	return 0;
}
/*
 *
 */
int loadTextFile(){
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);

	sprintf(fileName, "%d%02d%02d_%02d%02d%02d.m", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	count++;

    hFile = CreateFile(fileName,                // name of the write
                       GENERIC_WRITE,          // open for writing
					   FILE_SHARE_READ,        // Allow the file to be read
                       NULL,                   // default security
                       OPEN_ALWAYS,            // create new file if one does not already exist
                       FILE_ATTRIBUTE_NORMAL,  // normal file
                       NULL);                  // no attr. template

    if (hFile == INVALID_HANDLE_VALUE)
    {
        return 1;
    }

    return 0;

}
/*
 *
 */
void system_error(char *name) {
// Retrieve, format, and print out a message from the last error.  The
// `name' that's passed should be in the form of a present tense noun
// (phrase) such as "opening file".
//
    char *ptr = NULL;
    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM,
        0,
        GetLastError(),
        0,
        (char *)&ptr,
        1024,
        NULL);

    fprintf(stderr, "\nError %s: %s\n", name, ptr);
    LocalFree(ptr);
}
