import socket

ESP_IP = "192.168.4.1"  # ESP8266's IP address
ESP_PORT = 4210         # UDP port

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    message = input("Enter message (or 'quit' to exit): ")
    if message.lower() == 'quit':
        break
        
    # Send message
    sock.sendto(message.encode(), (ESP_IP, ESP_PORT))
    
    # Wait for response
    data, addr = sock.recvfrom(1024)
    print(f"ESP Response: {data.decode()}")

sock.close()