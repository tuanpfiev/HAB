# Simple-RFD900-Network - Network Rework Branch
This branch of the simple RFD900 network attempts to fix many of the bugs that were encountered in the main branch. The main change is that each byte in a message frame now has the system ID of the sending radio in the upper 4 bits. This means:

- There is only 15 unique IDs (1 to F) and one braod cast address (0).
- The amount of data sent is doubled.

This is a pretty rough solution and was never intended to be the final version of the network. 

## Usage
exicuting these scripts requires sudo and there two scripts you can exicute:
- RadioNetworkMain.py - This can be used on the balloon or as the ground station. the command to run it is as follows:
	- sudo python3 RadioNetworkMain.py mode SystemID ComPort
	- mode - use g for ground station and b for balloon.
	- SystemID - a number between 1 and F
	- ComPort - the comport for the RFD900 radio 
- DummyRadio.py - This is used for testing, it creates a dummy system that comunicates over the RFD900 radios. It will send fake GPS data based on it's system ID. the command to run it is:
	- sudo python3 DummyRadio.py SystemID ComPort  
	- SystemID - a number between 1 and F
	- ComPort - the comport for the RFD900 radio 

## How the code works
I have made sure to include many comments explaining what is happening in the script. The heart of the whole thing is NetworkManager.py which manages on the incoming and outgoing serial data for the RFD900. For this version of the network, when recieving data the following events should occur:

1. The data from the serial port is buffered until the buffer is full or the serial port read times out.
2. Each byte from the serial port data is examined individualy.
3. The system ID in the byte is extracted
4. The system ID is compared with the system IDs of the 'byte buckets'
5. If a match is found then the byte is added to the bucket. Otherwise, a new bucket is created with a matching ID.
6. each bucket is checked to see if it has a complete packet 
7. when a complete packet is found it is taken from the bucket and passed to the packet processing function. 
8. repeat from 3. for the next byte. 

If you have any questions just send me an email (adrian.bingham@rmit.edu.au)

## Known bugs 
- Ping time out in test conditions - it happens less often now but it is still occuring for some reason.
- GPS packets containing a the ping payload after CRC checks - not sure how this happens but i think it could be related to how python manages it's memory. 
- Other CRC check fails - need to examine the byte dumps to see what is happening. 