import threading
import socket
import queue


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.254.254.254', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

class UDPReceiver (threading.Thread):
    """Handles UDP socket receiving logic"""
    def __init__(self, buffersize, ip, port):
        """Initialize configuration"""
        threading.Thread.__init__(self)
        
        self.q = queue.Queue()
        self.close_thread = False
        self.bs = buffersize
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(3)
        self.udp.bind((ip, port))
        self.host_ip=get_ip()
      
    def run(self):
        """Thread run function handles receiving datagram and putting in queue"""
        while not self.close_thread:
            try:
                data, _ = self.udp.recvfrom(self.bs)
            except socket.timeout:
                continue
            if 'data' in locals():
                self.q.put(data)
                del data
        
        self.udp.close()
