# S.T.A.R.S App

## Physical Setup

![alt text](https://github.com/colincolt/STARS/blob/master/Main%20File/Overview-Physical%20Overview.png)

For testing purposes, the "External Computer" (Laptop) is connected to the server Pi via ssh (use Putty on Windows/Mac or bash terminal on Linux)
  - For the time being this requires hotspotting a phone and connecting both the laptop and Raspberry Pi
  - Note: Raspbian(Debian) or any OS can only assign one IP address per physical link (i.e. One IP for the WiFi chip, another for the Ethernet port, as well as virtual internal addresses for SOCKETS connections between python threads/processes)

## Pseudo-Code for the server_pi.py file

![alt text](https://github.com/colincolt/STARS/blob/master/Main%20File/Overview-Pseudo-code Overview.png)


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

Step 1:
Running the STEREOSCOPICMAIN_Server.py Script requires the OpenCV2 Libraries(Just the basic ones), follow the first half of instruction in the
following links: 
Windows: https://docs.opencv.org/2.4/doc/tutorials/introduction/windows_install/windows_install.html
Mac OS: https://stackoverflow.com/questions/46066903/how-can-i-install-opencv2-with-brew-under-osx

Step 2: Also requires python module "Numpy", navigate to (in windows cmd) "cd c:\Python27\Scripts" and run "easy_install.exe Numpy"

Step 3: Might have to "pip install imutils" and maybe "multithreading" if not present

### Installing


## Running the tests



## Deployment

Threading relies heavily on the use communication channels such as "Queue's" in order to pass information, this because obviously Threads cannot share data and variable directly, this would cause a mess, therefore we must use Queue: https://docs.python.org/3/library/asyncio-queue.html or something like "Sockets" to create a web server and port number to transfer data between threads.

See here for a Last in First Out (LIFO) Queue example: http://www.learn4master.com/programming-language/python/python-queue-for-multithreading

For more info on Threading: https://docs.python.org/3/library/threading.html


## Built With

* [Python2.7]


## License


## Acknowledgments

