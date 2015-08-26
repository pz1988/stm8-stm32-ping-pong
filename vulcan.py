#Library importation
import serial
import datetime
import time
import string

#Init serial
ser = serial.Serial(5, 9600, timeout=5)

ser.write("AT?" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+ID=?" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+RESET" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=0, 433.3" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=1, 433.5" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=2, 433.7" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=3, 433.9" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=4, 434.0" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=5, 434.2" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=6, 434.4" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ=7, 434.6" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+SF=7" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+ADR=0" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+FREQ" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+CMSGHEX=\"00 11 22 33\"" + '\r\n')
for i in range(0,6):
    msg = ser.readline()

    
    text = msg.strip("")
    findindex = string.find(msg, "finish")
    print findindex
    if findindex != -1:
        length = len(text)
        print length
        print text

    length = len(msg)
    print length
    
    #substr = msg[length-2-6:length-2]
    #print substr
    
    #if msg[length-2-6:length-2] == "finish":
    if string.find(msg, "finish") != -1:
        print "HELLO !!!!"
    print msg
    

ser.write("AT+POWER=14" + '\r\n')
msg = ser.readline()
print msg

#ser.write("AT+REPT=3" + '\r\n')
#msg = ser.readline()
#print msg

ser.write("AT+REPT=?" + '\r\n')
msg = ser.readline()
print msg

#ser.write("AT+RXWIN2=433.3" + '\r\n')
#msg = ser.readline()
#print msg

ser.write("AT+RXWIN2=433.3, 8" + '\r\n')
msg = ser.readline()
print msg

ser.write("AT+VER?" + '\r\n')
msg = ser.readline()
print msg

#ser.write("AT+KEY=NWKSKEY" + '\r\n')
#msg = ser.readline()
#print msg

#ser.write("AT+KEY=APPSKEY" + '\r\n')
#msg = ser.readline()
#print msg

ser.write("AT+HELP?" + '\r\n')
for i in range(0,22):
    msg = ser.readline()
    print msg

for num in range(0,0):

    #Generate time string HHMMSS
    time_label = datetime.datetime.now()
    print num, time_label
    time_label = time_label.strftime("%H%M%S")
    seconds_string = time_label[4:6]
    
    
    if seconds_string [1:2] == "5" :
        minutes_string = time_label[2:4]
        hours_string = time_label[0:2]
        #transmit time string
        ser.write("AT+MSGHEX=\"" + hours_string + " " + minutes_string + " " + seconds_string + "\"" + '\r\n')
        msg = ser.readline()
        print msg

        #ser.write("AT+CMSG=00 11 22 33 44 55" + '\r\n')
        msg = ser.readline()
        print msg
        
        msg = ser.readline()
        print msg
        msg = ser.readline()
        print msg
        msg = ser.readline()
        print msg
        msg = ser.readline()
        print msg
        
    time.sleep(1)

#Exit
ser.close()
