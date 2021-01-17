#Liam Strijckers 400179278
#Python 3.6.0, pyserial 3.4
import serial
import math
s = serial.Serial("COM10", 115200) #COM port used for my computer

print("Opening: " + s.name)

s.write(b'1')  #This program will send a '1' or 0x31 
i=1
f=open("data.xyz","w")#opens xyz to write the values to
f.write("")
f.close
f=open("data.xyz","a")
x=0
while(1):#checks to see if the starting ranging was succesful because the sensor is done booting once this happens
    t =s.readline()
    input = t.decode()
    if(input == "StartRanging Successful.\r\n"):
        print("check 1 done\n")
        break

while(1):# the same message comes up twice
    t =s.readline()
    input = t.decode()
    if(input == "StartRanging Successful.\r\n"):
        print("check 2 done \n")
        break

while(1):
    for i in range(512):
        t = s.readline()        # read one byte
        input = t.decode()      # convert byte type to str
        d = int(input.strip('\n')) #strips the new line off of the incoming data
        print(d)
        y= d*math.sin(math.radians((360*i)/512))#calculations for the y and z values
        z= d*math.cos(math.radians((360*i)/512))
        f.write("{} {} {}\n".format(x,y,z))#write the xyz values to the xyz file
        #f.write("\n")
    
    x=x+200
    if(x>2000):
        break
f.close #closes the xyz file
print("Closing: " + s.name)
s.close()
f = open("data.xyz","r") #opens the file for reading so the user can look through it to make sure the data looks correct
print(f.read())