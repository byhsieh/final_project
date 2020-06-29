import pyb
import sensor, image, time, math, os, tf
BLUE_LED_PIN = 3

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)
tmp = ""

def identification():
   sensor.reset()                         # Reset and initialize the sensor.
   sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
   sensor.set_framesize(sensor.QVGA)      # Set frame size to QVGA (?x?)
   sensor.set_windowing((240, 240))       # Set 240x240 window.
   sensor.skip_frames(time=2000)          # Let the camera adjust.

   labels = ['3', '4', '0', 'other']
   pyb.LED(BLUE_LED_PIN).on()
   img = sensor.snapshot().save("identification.jpg")
   for obj in tf.classify('/model_demo.tflite',img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
      img.draw_rectangle(obj.rect())
      img.draw_string(obj.x()+3, obj.y()-1, labels[obj.output().index(max(obj.output()))], mono_space = False)
   pyb.LED(BLUE_LED_PIN).off()
   return labels[obj.output().index(max(obj.output()))]

def matrix():
   sensor.reset()
   sensor.set_pixformat(sensor.RGB565)
   sensor.set_framesize(sensor.QVGA)
   sensor.skip_frames(time = 2000)
   sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
   sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
   pyb.LED(BLUE_LED_PIN).on()
   img = sensor.snapshot()
   img.lens_corr(1.8) # strength of 1.8 is good for the 2.8mm lens.

   matrices = img.find_datamatrices()
   for matrix in matrices:
      rotation_var = ""
      rotation_var = str(int((180 * matrix.rotation()) / math.pi))
      pyb.LED(BLUE_LED_PIN).off()
      return rotation_var

pyb.LED(BLUE_LED_PIN).off()

while(1):
   a = uart.readline()
   if a is not None:
     tmp += a.decode()
   if tmp == "matrix":
      tmp =""
      rotation_data = matrix()
      pyb.LED(BLUE_LED_PIN).off()
      if str(type(rotation_data))=="<class 'NoneType'>":
        uart.write("no need\r".encode())
      else:
        sender = "cal:"+rotation_data+"\r"
        uart.write(sender.encode())

   if tmp == "identification":
      tmp =""
      label = identification()
      uart.write(label.encode())
