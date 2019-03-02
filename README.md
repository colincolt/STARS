# S.T.A.R.S App

## Physical Setup

![alt text](https://github.com/colincolt/STARS/blob/master/Images/IOflow.png)

For testing purposes, the "External Computer" (Laptop) is connected to the server Pi via ssh (use Putty on Windows/Mac or bash terminal on Linux)
  - For the time being this requires hotspotting a phone and connecting both the laptop and Raspberry Pi
  - Note: Raspbian(Debian) or any OS can only assign one IP address per physical link (i.e. One IP for the WiFi chip, another for the Ethernet port, as well as virtual internal addresses for SOCKETS connections between python threads/processes)

## Pseudo-Code for the server_pi.py file

![alt text](https://github.com/colincolt/STARS/blob/master/Main%20File/Overview-Physical%20Overview.png)


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

Step 1:
Running the application (both client and server) requires having openCV 3.3.0 or greater, follow the instructions in the following link with the exception of installing and entering a virtual environment in step #4 (virtual environments are for testing purposes only, this Raspberry Pi will be dedicated to running an openCV program, thus installing system-wide is preferred): https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/

Step 2: 
Once the two Raspberry Pi's have all the necessary python modules installed, connect the ethernet cables between them
  - once connected, the server IP will not change as it is not connected to any router that would assign Dynamic IP's, thus there is no need to set a static IP within the Debian system for the sake of a sockets connection between operating python programs on each Raspberry Pi.

Step 3: Set the client_pi.py script to start at boot on the client Raspberry Pi using the method described here: https://www.pyimagesearch.com/2016/05/16/running-a-python-opencv-script-on-reboot/

Step 4: Set the gui.py application to start on boot on the server Raspberry Pi using the method described in the link listed above
  - Copy all files into a folder:
    - gui.py
    - logo.gif
    - server_pi.py

Step 5: Make sure BOTH the Arduino Mega and Uno are connected to the sensors and motors, and the serial cable is plugged into the server

## Running the tests
Step 1: Ensure 5 balls are loaded into the hopper

  - If the drill selection is Dynamic
  
    - Send voice command "Begin"
    - player starts further away (15-25 meters) or closer (5-15 meters) and runs in the opposite direction of starting distance
    - Launcher sends a ball every 8 seconds to players predicted location, waiting for a voice command to "Stop" in between each
    
  - If the drill selection is Static
  
    - Send voice command "Begin"
    - player starts further away (15-25 meters) or closer (5-15 meters) and moves in the opposite direction of starting distance
    - Launcher sends a ball every 8 seconds to players static location, waiting for a voice command to "Stop" in between each
  
   - If the drill selection is Manual
   
    - Send voice command "Begin"
    - Player starts at any distance and ball launches everytime the voice command "Begin" is received   


## Deployment

Threading relies heavily on the use communication channels such as "Queue's" in order to pass information, this because obviously Threads cannot share data and variable directly, this would cause a mess, therefore we must use Queue: https://docs.python.org/3/library/asyncio-queue.html or something like "Sockets" to create a web server and port number to transfer data between threads.

See here for a Last in First Out (LIFO) Queue example: http://www.learn4master.com/programming-language/python/python-queue-for-multithreading

For more info on Threading: https://docs.python.org/3/library/threading.html


## Built With

* [Python3.6]


## License


## Acknowledgments

