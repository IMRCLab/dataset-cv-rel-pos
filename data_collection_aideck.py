import argparse
import time
import socket,os,struct, time
import numpy as np
import cv2
import shutil

# Args for setting IP/port of AI-deck. Default settings are for when
# AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument("-path", help="Path to save images and .csv file")
parser.add_argument("-id", default="cf231", help="ID of the robot")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n
path =  args.path
robot_id = args.id

# shutil.rmtree(path, ignore_errors=True)
os.makedirs(path, exist_ok=True)
# os.mkdir(path)

print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((deck_ip, deck_port))
print("Socket connected")

imgdata = None
data_buffer = bytearray()

def rx_bytes(size):
  data = bytearray()
  while len(data) < size:
    data.extend(client_socket.recv(size-len(data)))
  return data

start = time.time()
count = 0
n = 0
image_name = 'cf' + robot_id + '_{0:05d}.jpg'

with open(os.path.join(path , 'cf'+ robot_id + '_wifi.csv'), "w") as file:
  file.write("image_name,timestamp,x,y,z,qw,qx,qy,qz\n")
  file.flush()

  while(1):
    # First get the info
    packetInfoRaw = rx_bytes(4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

    imgHeader = rx_bytes(length - 2)
    #print(imgHeader)
    # print("Length of data is {}".format(len(imgHeader)))
    [magic, width, height, depth, format, size, timestamp, x, y, z, qx, qy, qz, qw] = struct.unpack('<BHHBBIIfffffff', imgHeader)
    if magic == 0xBC:
      #print("Magic is good")
      #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
      #print("Image format is {}".format(format))
      #print("Image size is {} bytes".format(size))
      print("CF State {},{},{} m at time {} s".format(x, y, z, timestamp/1000))

      # Now we start rx the image, this will be split up in packages of some size
      imgStream = bytearray()

      while len(imgStream) < size:
          packetInfoRaw = rx_bytes(4)
          [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
          #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
          chunk = rx_bytes(length - 2)
          imgStream.extend(chunk)
     
      count = count + 1
      meanTimePerImage = (time.time()-start) / count
      # print("{}".format(meanTimePerImage))
      print("Rate: {} Hz".format(1/meanTimePerImage))

      if format == 0:
          # with open("img.raw", "wb") as f:
          #     f.write(imgStream)
          bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
          #img2 = np.fromfile("img.raw", dtype=np.uint8)
          bayer_img.shape = (324, 324)
          # crop to 320x320
          bayer_img = bayer_img[2:322,2:322]
          cv2.imshow('Bayer', bayer_img)
          # cv2.imshow('Color', cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA))
          file.write("{},{},{},{},{},{},{},{},{}\n".format(image_name.format(n),timestamp/1000,x,y,z,qw,qx,qy,qz)) # separate them for yolo labeling
          file.flush()
          cv2.imwrite(os.path.join(path, image_name.format(n)), bayer_img)
          n += 1
          cv2.waitKey(1)
      else:
          # with open("img.jpeg", "wb") as f:
          #     f.write(imgStream)
          nparr = np.frombuffer(imgStream, np.uint8)
          decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
          # crop to 320x320
          decoded = decoded[2:322,2:322]
          cv2.imshow('JPEG', decoded)
          file.write("{},{},{},{},{},{},{},{},{}\n".format(image_name.format(n),timestamp/1000,x,y,z,qw,qx,qy,qz)) # separate them for yolo labeling
          file.flush()
          cv2.imwrite(os.path.join(path , image_name.format(n)), decoded)
          n += 1
          cv2.waitKey(1)
