import socket

ip = "192.168.10.1"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Finding connection to target")
s.connect((ip, 8889))
s.send(b'command')
s.send(b'motoron')
print("Connected to " + ip)
s.close()