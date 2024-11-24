#include "circularBuffer.h"
#include "stdbool.h"
#include <stdint.h>
#include "string.h"

// Dit circulair buffer verwacht reeksen karakters, die afgesloten worden met een '\n'. Ideaal om AT-commando's mee te verwerken...
// Wil je 'gewone' strings verwerken, laat die dan steeds eindigen met een '\n'.

// Globale variabelen aanmaken. Die variabelen kunnen via het 'extern' keyword ook gebruikt worden in andere
// bestanden van dit project. Het buffer moet 'volatile' zijn omdat het kan wijzigen 'tussen twee leesacties' (in de interrupt).
volatile CircularBuffer circularBuffer;
enum CircularBufferActionResult circularBufferActionResult;

// Deze functie stelt een nieuw aangemaakt CircularBuffer correct in. Roep ze dan ook steeds op aan het begin van je code.
enum CircularBufferActionResult InitCircularBuffer(volatile CircularBuffer* buffer)
{
    // Alle bytes op nul initialiseren
    for(uint16_t index = 0; index < CIRCULAR_BUFFER_SIZE; index++)
        buffer->bufferData[index] = 0;

    // Lees- en schrijfpointer resetten.
    buffer->writeIndex = 0;
    buffer->readIndex = 0;

    // Aangeven dat er nog geen geldige stings ontvangen zijn.
    buffer->numberOfStringsInBuffer = 0;

    // Start van huidig binnenkomende string resetten.
    buffer->startIndexOfCurrentlyArrivingString = 0;

    // Aangeven dat initialisatie gelukt is.
    return initSucceeded;
}

// Haal één karakter op uit het circulair buffer, via een pointer...
enum CircularBufferActionResult PopCharFromCircularBuffer(volatile CircularBuffer* buffer, char* data)
{
    enum CircularBufferActionResult temp;

    // Opm: (*ptr).a is hetzelfde als ptr->a.

    if (IsCircularBufferEmpty(buffer))
			temp = bufferEmpty;
    else
    {
			*data = buffer->bufferData[buffer->readIndex];
			buffer->readIndex++;

			if (buffer->readIndex >= CIRCULAR_BUFFER_SIZE)
				buffer->readIndex = 0;

			temp = readSucceeded;
    }

    return temp;
}

// Sla één karakter op in het circulair buffer.
enum CircularBufferActionResult PushCharToCircularBuffer(volatile CircularBuffer* buffer, char data)
{
    enum CircularBufferActionResult temp;

    // Als writeIndex vlak vóór de readIndex staat, of als de writeIndex helemaal op het einde staat terwijl de 
    // readIndex nog helemaal in het begin => buffer is vol...
    // Je wil steeds één plaats ongebruikt houden, zodat je duidelijk kan uitmaken wanneer een buffer vol of leeg is.
    // Leeg => als writeIndex en readIndex dezelfde waarde hebben.
    // Vol => als writeIndex vlak vóór de readIndex staat.

    if (IsCircularBufferFull(buffer))
			return bufferFull;
    else
    {
			buffer->bufferData[buffer->writeIndex] = data;
			buffer->writeIndex++;

			if (buffer->writeIndex >= CIRCULAR_BUFFER_SIZE)
				buffer->writeIndex = 0;
			
			// Nieuwe lijn ontvangen. Verhoog de teller die aangeeft hoeveel lijnen er aanwezig zijn.
			if(data == '\n')
				buffer->numberOfStringsInBuffer++;

			return writeSucceeded;
    }
}

// Haal één volledige string op uit het circulair buffer. Een string MOET eindigen op '\n'.
enum CircularBufferActionResult PopStringFromCircularBuffer(volatile CircularBuffer* buffer, char* string)
{
	enum CircularBufferActionResult result;
	char temp;
	uint8_t index = 0;

	if (IsCircularBufferEmpty(buffer))
		result = bufferEmpty;
	else
	{
		if(buffer->numberOfStringsInBuffer > 0)
		{
			PopCharFromCircularBuffer(buffer, &temp);
			string[index++] = temp;

			// TODO: om onverwachte oneindige lussen te vermijden, voeg een timeout toe...
			while(temp != '\n')								// Zolang geen line feed gezien, doe verder...
			{
				PopCharFromCircularBuffer(buffer, &temp);
				string[index++] = temp;
			}
			
			// Einde-string-teken toevoegen.
			string[index] = 0;

			buffer->numberOfStringsInBuffer--;
			
			result = readStringSucceeded;
		}
		else
			result = readStringFailed;
	}
	
	return result;
}

// Eén string toevoegen aan het circulair buffer.
enum CircularBufferActionResult PushStringToCircularBuffer(volatile CircularBuffer* buffer, char* string)
{
	uint8_t index = 0;
	enum CircularBufferActionResult temp;
	
	// TODO: timeout voorzien?
	// Zolang geen einde string teken, push naar het circulair buffer.
	while(string[index] != 0)
		temp = PushCharToCircularBuffer(buffer, string[index++]);
	
	return temp;
}

// Controle op het leeg zijn van het circulair buffer.
bool IsCircularBufferEmpty(volatile CircularBuffer* buffer)
{
	if(buffer->readIndex == buffer->writeIndex)
		return true;
	else
		return false;
}

// Controle op het vol zijn van het circulair buffer.
bool IsCircularBufferFull(volatile CircularBuffer* buffer)
{
	if ((buffer->writeIndex == (buffer->readIndex - 1)) || ((buffer->writeIndex == (CIRCULAR_BUFFER_SIZE-1)) && (buffer->readIndex == 0)))
		return true;
	else
		return false;
}

// Zoek achter één bepaalde string in het huidige buffer. Na deze functie is het buffer geledegd.
bool LookForString1(volatile CircularBuffer* circularBuffer, char needle[])
{
	char haystack[CHAR_ARRAY_BUFFER_LENGTH];
	enum CircularBufferActionResult circularBufferActionResult;	
	
	while(circularBuffer->numberOfStringsInBuffer > 0)
	{
		circularBufferActionResult = PopStringFromCircularBuffer(circularBuffer, haystack);
		if(circularBufferActionResult == readStringSucceeded)
		{
			if(strcmp(haystack, needle) == 0)
				return true;
		}
	}

	return false;    
}

// Zoek achter één van twee opgegeven strings in het huidige buffer. Na deze functie is het buffer geledegd.
uint8_t LookForString2(volatile CircularBuffer* circularBuffer, char needle1[], char needle2[])
{
	char haystack[CHAR_ARRAY_BUFFER_LENGTH];
	enum CircularBufferActionResult circularBufferActionResult;
	
	while(circularBuffer->numberOfStringsInBuffer > 0)
	{
		circularBufferActionResult = PopStringFromCircularBuffer(circularBuffer, haystack);
		if(circularBufferActionResult == readStringSucceeded)
		{
			if(strcmp(haystack, needle1) == 0)
				return 1;
			
			if(strcmp(haystack, needle2) == 0)
				return 2;
		}
	}

	return 0;    
}

// Zoek achter één van drie opgegeven strings in het huidige buffer. Na deze functie is het buffer geledegd.
uint8_t LookForString3(volatile CircularBuffer* circularBuffer, char needle1[], char needle2[], char needle3[])
{
	char haystack[CHAR_ARRAY_BUFFER_LENGTH];
	enum CircularBufferActionResult circularBufferActionResult;	
	
	while(circularBuffer->numberOfStringsInBuffer > 0)
	{
		circularBufferActionResult = PopStringFromCircularBuffer(circularBuffer, haystack);
		if(circularBufferActionResult == readStringSucceeded)
		{
			if(strcmp(haystack, needle1) == 0)
				return 1;
			
			if(strcmp(haystack, needle2) == 0)
				return 2;
			
			if(strcmp(haystack, needle3) == 0)
				return 3;
		}
	}

	return 0;    
}
