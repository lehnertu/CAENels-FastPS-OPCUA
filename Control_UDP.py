#!/usr/bin/python
import socket, numpy, struct
import argparse

UDP_IP = "10.66.67.10"
UDP_PORT = 16665

parser = argparse.ArgumentParser()
parser.add_argument('set', type=int, help='0/1 set the output value')
parser.add_argument('-i', type=float, dest='current', default=0.0, help='current value [A]')
parser.add_argument('-v', type=float, dest='voltage', default=0.0, help='voltage value [V]')
args = parser.parse_args()

curr_set = int(round(1e6*args.current))
volt_set = int(round(1e6*args.voltage))

# send a udp packet with the control input
block = bytearray('CeUL') + struct.pack('<i',args.set) + struct.pack('<q',curr_set) + struct.pack('<q',volt_set)
# for b in block: print b
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind(("", UDP_PORT))
sock.sendto(block, (UDP_IP, UDP_PORT))

# now receive the answer
record = numpy.dtype([('status','<u4'),('is','<q'),('vs','<q'),('i','<q'),('v','<q')])
block, addr = sock.recvfrom(1024)
block = block[28:]
# print 'block size %d' % len(block)
# for b in block: print ord(b)
data=numpy.frombuffer(block,dtype=record,count=1)
print data
print 'status = %d' % data['status']
print 'current setpoint = %f A' % (1e-6*data['is'])
print 'voltage setpoint = %f V' % (1e-6*data['vs'])
print 'current readback = %f A' % (1e-6*data['i'])
print 'voltage readback = %f V' % (1e-6*data['v'])
