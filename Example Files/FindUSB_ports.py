import serial
import serial.tools.list_ports

def findUNO():
    # Find Live Ports, return port name if found, NULL if not
    # print ('Scanning all live ports on this PC')
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        #print p # This causes each port's information to be printed out.
        if "0043" in p[2]:
            # print ('Evo found on port ' + p[0])
            return(p[0])
    return ('NULL')

def findMEGA():
    # Find Live Ports, return port name if found, NULL if not
    # print ('Scanning all live ports on this PC')
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        #print p # This causes each port's information to be printed out.
        if "0042" in p[2]:
            # print ('Evo found on port ' + p[0])
            return(p[0])
    return ('NULL')

uno_port = findUNO()
mega_port = findMEGA()

print(uno_port)
print(mega_port)
