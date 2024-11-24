#include <stdint.h>
#include "stdbool.h"

#ifndef CIRCULAR_BUFFER_INCLUDED
	#define CIRCULAR_BUFFER_INCLUDED

	// Vul hier proefondervindelijk het maximum aantal in zodat de RAM zo volledige mogelijk gebruikt wordt (indien gewenst). 
	#define CIRCULAR_BUFFER_SIZE 2001				// Grootte van het buffer in bytes. Eén plaats blijft sowieso vrij, dus ... plus 1
	#define CHAR_ARRAY_BUFFER_LENGTH 501		// De maximum lengte opgeven van stukken tekst die je wil verwerken.
																					// Let op: als de array's korter zijn (101 bijvoorbeeld), kunnen er problemen ontstaan bij het resetten. 
																					// Dan komen er vrij veel karakters binnen zonder één '\n', daardoor kunnen de array's overlopen en krijg je 
																					// onvoorspelbaar gedrag van de code. De ideale lengte zit wellicht niet lager dan 350...
																					// Want de "ready" die aangeeft dat de module klaar is voor gebruik, zit ongeveer op index 325. Dus de array's moeten
																					// zeker langer zijn dan dat...

	// Enumeratie (opsomming) van alle mogelijke resultaten die je kan bekomen na een opdracht met het circulair buffer.
	enum CircularBufferActionResult { readSucceeded, writeSucceeded, initSucceeded, bufferEmpty, bufferFull, readStringSucceeded, readStringFailed};

	// Struct die een circulair buffer voorstelt.
	typedef struct
	{
			uint16_t readIndex;
			uint16_t writeIndex;
			uint16_t numberOfStringsInBuffer;
			uint16_t startIndexOfCurrentlyArrivingString;
			char bufferData[CIRCULAR_BUFFER_SIZE];
	}CircularBuffer;
		
	// Functieprototypes.
	enum CircularBufferActionResult InitCircularBuffer(volatile CircularBuffer* buffer);
	enum CircularBufferActionResult PopCharFromCircularBuffer(volatile CircularBuffer* buffer, char* data);
	enum CircularBufferActionResult PushCharToCircularBuffer(volatile CircularBuffer* buffer, char data);
	enum CircularBufferActionResult PopStringFromCircularBuffer(volatile CircularBuffer* buffer, char* string);
	enum CircularBufferActionResult PushStringToCircularBuffer(volatile CircularBuffer* buffer, char* string);
	bool IsCircularBufferEmpty(volatile CircularBuffer* buffer);
	bool IsCircularBufferFull(volatile CircularBuffer* buffer);
	
	bool LookForString1(volatile CircularBuffer* circularBuffer, char needle[]);
	uint8_t LookForString2(volatile CircularBuffer* circularBuffer, char needle1[], char needle2[]);
	uint8_t LookForString3(volatile CircularBuffer* circularBuffer, char needle1[], char needle2[], char needle3[]);
#endif
