import math
import time
import RPi.GPIO as GPIO  
GPIO.setmode(GPIO.BCM)  
  
# GPIOs set up as inputs, pulled up to avoid false detection.  
# Ports are wired to connect to GND on magnet proximity.  
# Set up for up falling edge detection   
GPIO.setup(3, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(1, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIOR_TRIGGER = 17
GPIOR_ECHO    = 18
GPIOL_TRIGGER = 22
GPIOL_ECHO    = 23


# Set pins as output and input
GPIO.setup(GPIOR_TRIGGER,GPIO.OUT)  # Right Trigger
GPIO.setup(GPIOR_ECHO,GPIO.IN)      #       Echo
GPIO.setup(GPIOL_TRIGGER,GPIO.OUT)  # Left  Trigger
GPIO.setup(GPIOL_ECHO,GPIO.IN)      #       Echo

# Set trigger to False (Low)
GPIO.output(GPIOR_TRIGGER, False)
GPIO.output(GPIOL_TRIGGER, False)



right_encoder = 0
left_encoder = 0

def measureR():
  # This function measures the R distance

  GPIO.output(GPIOR_TRIGGER, True)
  time.sleep(0.00001)
  GPIO.output(GPIOR_TRIGGER, False)
  start = time.time()
  
  while GPIO.input(GPIOR_ECHO)==0:
    start = time.time()

  while GPIO.input(GPIOR_ECHO)==1:
    stop = time.time()

  elapsed = stop-start
  Rdistance = (elapsed * 34300)/2

  return Rdistance

def measureL():
  # This function measures the L distance

  GPIO.output(GPIOL_TRIGGER, True)
  time.sleep(0.00001)
  GPIO.output(GPIOL_TRIGGER, False)
  start = time.time()
  
  while GPIO.input(GPIOL_ECHO)==0:
    start = time.time()

  while GPIO.input(GPIOL_ECHO)==1:
    stop = time.time()

  elapsed = stop-start
  Ldistance = (elapsed * 34300)/2

  return Ldistance

def measure_averageR():
  # This function takes 3 measurements and
  # returns the average.

  distance1=measureR()
  time.sleep(0.1)
  distance2=measureR()
  time.sleep(0.1)
  distance3=measureR()
  Rdistance = distance1 + distance2 + distance3
  Rdistance = Rdistance / 3
  print "Right Distance is %f" % Rdistance
  return Rdistance
  
  
  

def measure_averageL():
  # This function takes 3 measurements and
  # returns the average.

  distance1=measureL()
  time.sleep(0.1)
  distance2=measureL()
  time.sleep(0.1)
  distance3=measureL()
  Ldistance = distance1 + distance2 + distance3
  Ldistance = Ldistance / 3
  print "Left Distance is %f" % Ldistance
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
# define threaded callback functions  
# these will run in another thread when our events are detected  
def my_callback(self):
    global right_encoder
    right_encoder +=1
    #print "falling edge detected on HES 1"  
  
#def my_callback1(channel):  
    #print "falling endge detected on HES 2"
    
def my_callback2(self):
    global left_encoder
    left_encoder +=1
    #print "falling edge detected on HES 3"  
  
#def my_callback3(channel):  
    #print "falling edge detected on HES 4"

# define constants
LEFT_CLICKS_PER_METER = 10.985	 # the wheels might not be the same size
RIGHT_CLICKS_PER_METER = 10.985
WHEEL_BASE = .518  # distance between wheels  in meters*

#define encoders
def encoders():
    
    #Setup GPIO interrupts
    GPIO.add_event_detect(3, GPIO.FALLING, callback=my_callback, bouncetime=100)

    #GPIO.add_event_detect(4, GPIO.FALLING, callback=my_callback1, bouncetime=300)

    GPIO.add_event_detect(27, GPIO.FALLING, callback=my_callback2, bouncetime=100)
  
    #GPIO.add_event_detect(24, GPIO.FALLING, callback=my_callback3, bouncetime=300)
    while True:

        # sample the left and right encoder counts as close together time as possible
        
        L_ticks = left_encoder
        R_ticks = right_encoder
        

        left_meters = L_ticks/LEFT_CLICKS_PER_METER
        #print "Left meters: %f" % left_meters
        right_meters = R_ticks/RIGHT_CLICKS_PER_METER
        #print "Right meters: %f" % right_meters

        
	
        meters = (left_meters + right_meters) / 2.0
        inches = meters*39.37007874
        print "%f Inches Motherfucker, %s on the left %s on the right" % (inches,
            left_encoder, right_encoder)


	#accumulate total rotation around our center 
        theta = (left_meters - right_meters) / WHEEL_BASE

        x_position = meters*math.sin(theta*(180/math.pi))
        y_position = meters*math.cos(theta*(180/math.pi))
        
	measure_averageR()
        time.sleep(1)
        measure_averageL()
        time.sleep(1)
	#print x_position
	#print y_position
	#print right_encoder
	#print left_encoder

	
        time.sleep(1)


encoders()
      
      
  
if KeyboardInterrupt:  
    GPIO.cleanup()       # clean up GPIO on CTRL+C exit  
