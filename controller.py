'''
A python controller for Pixelink vision cameras (WIP).
'''

import sys
import tty
import termios
import numpy
import cv2
import time
import os
import pickle
from pixelinkWrapper import*
from ctypes import* # This allows using the various objects (variables, classes, methods...) from the imported module without prefixing them with the module's name.
                    # Python’s built-in ctypes feature allows you to interact with C code from Python.
import threading
import datetime
from datetime import timedelta
import loog

# libraries for mqtt broker management
from paho.mqtt import client as mqtt_client
import ctypes
import threading
import base64

global flag_stream_overlapped
flag_stream_overlapped = False

global flag_preview
flag_preview = False

global flag_remote_streaming
flag_remote_streaming = False

'''
For camera file save snapshots.
'''
SUCCESS = 0
FAILURE = 1

'''
For opencv snapshots.
'''
# A few useful defines
EXIT_SUCCESS = 0
EXIT_FAILURE = 1

PT_COLOR = 0
PT_MONO = 1
PT_OTHERWISE = 2 # Special cameras, like polarized and interleaved HDR

class camera():
    """ Camera class that contains pixelink camera functions. This API contains functions which are nucleated in main functions (e.g: px.start_preview; px.main_snapshot).
    """
    def __init__(self): 
        """ Camera initialization section.
        """
        # A few useful defines
        self.EXIT_SUCCESS = 0
        self.EXIT_FAILURE = 1

        self.PT_COLOR = 0
        self.PT_MONO = 1
        self.PT_OTHERWISE = 2 # Special cameras, like polarized and interleaved HDR

        # Set ROI for sensor image and digital pattern
        self.ROI_LEFT = int(0) # initial ROI Xo
        self.ROI_TOP = int(0) # initial ROI Yo
        self.X_PIXEL_RANGE = int(3840) # initial ROI Xf; max = 3840 - ROI_LEFT
        self.Y_PIXEL_RANGE = int(2484) # initial ROI Yf; max = 2484 - ROI_TOP

        self.ROI_WIDTH = int(self.ROI_LEFT + self.X_PIXEL_RANGE) # max = 3840
        self.ROI_HEIGHT = int(self.ROI_TOP + self.Y_PIXEL_RANGE) # max = 2484

        self.exposure = 1 # Hardcoded initial exposure sensor time (ms)
        self.time = 1 # Hardcoded initial operation time in s

        self.MQTT_FLAG = False

        # Initialize the camera
        try:
            self.ret = PxLApi.initialize(0)
            if not PxLApi.apiSuccess(self.ret[0]):
                print("Error: Unable to initialize a camera")
                return None
            self.hCamera = self.ret[1]
            print("self.hCamera: ",self.hCamera)
            if self.hCamera == 1:
                print("Success interfacing with camera.")
        except Exception as e:
            print(e)
    
    def get_comm(self):
        ret = PxLApi.getNumberCameras()
        if not(PxLApi.apiSuccess(ret[0])):
            print("Attempt to get number cameras returned %i!" % ret[0])
            return
        print("Get number cameras attached: ",ret)
        ret = PxLApi.getCameraInfo(self.hCamera)
        if not(PxLApi.apiSuccess(ret[0])):
            print("Attempt to get camera info returned %i!" % ret[0])
            return
        print("Get camera info: ",ret)

    def connectToBroker(self, parameters:dict):
        # http://www.steves-internet-guide.com/into-mqtt-python-client/
        
        ip = parameters["ip"]
        port = parameters["port"]
        self.topic = parameters["topic"]
        user = parameters["user"]
        passw = parameters["passw"]
        client = parameters["clientName"]
          
        def on_connect(client:str, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
                client.subscribe(self.topic)
            else:
                print("Failed to connect, return code %d\n", rc)

        def on_disconnect(client, userdata, rc):
            print("Disconnected from MQTT Broker")

        client = mqtt_client.Client(client, clean_session=False)  # client id
        client.username_pw_set(user, passw)
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect

        print("Connecting to MQTT Broker...")
        try:
            port = int(port)
            client.connect(ip, port)
        except ValueError:
            print(ValueError)

        client.loop_start()

        return client

    def set_mqtt(self, ip:str, port:int, topic:str, user:str, passw:str, clientName:str):
        print("Setting MQTT parameters...")

        parameters = {
        'ip': ip,
        'port': port,
        'topic': topic,
        'user': user,
        'passw': passw,
        'clientName': clientName
        }

        print("mqtt_parameters = ", parameters)

        print("parameters: ",parameters)
        self.clientmqtt = self.connectToBroker(parameters)
        self.MQTT_FLAG = True
        
    def kill_mqtt(self):
        self.clientmqtt.loop_stop()
        self.clientmqtt.disconnect()
        self.MQTT_FLAG = False

    # -------------------------------------------------------- FUNCTIONS FOR SENSOR CONFIG ------------------------------------------------------------------
    
    def set_exp(self,N):
        """ Sets camera sensor exposure.

        Args:
            N (integer): this variable controls the exposure time of the sensor in millis. 
            Increasing the shutter integration time makes the image brighter. 
            
        Return:
            exposure: adjusted camera sensor exposure in seconds.
        """

        print("...........................")

        self.exposure = float(N)

        # Set the sensor exposure
        ret = PxLApi.getFeature(self.hCamera, PxLApi.FeatureId.EXPOSURE)
        if not(PxLApi.apiSuccess(ret[0])):
            print("Attempt to get exposure returned %i!" % ret[0])
            return
        
        print("previous ret: ",ret)
        params = ret[2]
        print("params: ",params)
        exposure = params[0]
        print("previous exposure: ",exposure)
        exposure = (self.exposure/1000) #  updating sensor exposure
        params[0] = exposure
        print("future exposure to be configured now: ",params[0])

        ret = PxLApi.setFeature(self.hCamera, PxLApi.FeatureId.EXPOSURE, PxLApi.FeatureFlags.PRESENCE, params)
        
        if (not PxLApi.apiSuccess(ret[0])):
            print("Attempt to set exposure returned %i!" % ret[0])
            print("ERROR")

        if ret[0] == 0:
            print("hardware config success")

        print("theoretical limit for fps config <= ",int(1/params[0]))
        self.get_fps()
        print("...........................")

        return ret[0] # if value = 0 -> SUCCESS

    def get_exp(self):
        """ Gets camera sensor exposure configuration.

        Return:
            exposure: adjusted camera sensor exposure in seconds.
        """
        ret = PxLApi.getFeature(self.hCamera, PxLApi.FeatureId.EXPOSURE)
        if not(PxLApi.apiSuccess(ret[0])):
            print("Attempt to get exposure returned %i!" % ret[0])
            return
        
        params = ret[2]
        exposure = params[0]
        print("actual exposure: ",exposure)
    
        return exposure

    def set_fps(self,N):
        """ Sets camera frames per second.

        Args:
            N (float): this variable controls the fps of sensor. 
            Increasing the fps makes the image darker.
            
        Return:
            self.fps: adjusted camera sensor fps.
        """
        # reset camera just to be sure of good operation
        PxLApi.uninitialize(self.hCamera)

        # reinitialize the camera
        try:
            self.ret = PxLApi.initialize(0)
            if not PxLApi.apiSuccess(self.ret[0]):
                print("Error: Unable to initialize a camera")
                return None
            self.hCamera = self.ret[1]
        except Exception as e:
            print(e)

        self.fps = float(N)

        # Set the sensor fps
        ret = PxLApi.getFeature(self.hCamera, PxLApi.FeatureId.FRAME_RATE)
        if not(PxLApi.apiSuccess(ret[0])):
            print("Attempt to get fps returned %i!" % ret[0])
            return
        
        print("previous ret: ",ret)
        params = ret[2]
        print("params: ",params)
        fps = params[0]
        print("previous fps: ",fps)
        fps = self.fps #  updating sensor fps
        params[0] = fps
        print("future fps to be configured now: ",params[0])

        ret = PxLApi.setFeature(self.hCamera, PxLApi.FeatureId.FRAME_RATE, PxLApi.FeatureFlags.MANUAL, params)
        time.sleep(1)
        print("new ret: ",ret)
        
        if (not PxLApi.apiSuccess(ret[0])):
            print("Attempt to set fps returned %i!" % ret[0])

        return ret[0]
       
    def get_fps(self):
        """ Gets camera sensor fps configuration.

        Return:
            fps: adjusted camera sensor fps. 
        """
        ret = PxLApi.getFeature(self.hCamera, PxLApi.FeatureId.ACTUAL_FRAME_RATE)
        if not(PxLApi.apiSuccess(ret[0])):
            print("Attempt to get fps returned %i!" % ret[0])
            return
        
        params = ret[2]
        fps = params[0]
        print("actual fps: ",fps)
        return fps

    def set_roi(self,a,b,c,d):
        """ Sets camera sensor rectangular region of interest (ROI). This is a feature of most CMOS sensors that 
        allows only a portion of the active sensor to be selected and read out.
        The benefit of this is a reduction in the total number of pixels and an increase in the readout speed.
        
        Args:
            a (integer): Xo coordinate for the left side of ROI.
            b (integer): Yo coordinate for the upper side of ROI.
            c (integer): sets width for ROI.
            d (integer): sets height for ROI.

        Return:
            (ROI_LEFT, ROI_TOP, X_PIXEL_RANGE, Y_PIXEL_RANGE)

        - self.ROI_LEFT = int(a) # ROI Xo
        - self.ROI_TOP = int(b) # ROI Yo
        - self.X_PIXEL_RANGE = int(c) # ROI Xf; max = 3840 - ROI_LEFT
        - self.Y_PIXEL_RANGE = int(d) # ROI Yf; max = 2484 - ROI_TOP
        - self.ROI_WIDTH = int(self.ROI_LEFT + self.X_PIXEL_RANGE) # max = 3840
        - self.ROI_HEIGHT = int(self.ROI_TOP + self.Y_PIXEL_RANGE) # max = 2484
        """
        global flag_preview
        global flag_remote_streaming

        flag_recent_preview = False
        flag_recent_remote_streaming = False

        if flag_preview == True:
            self.kill_preview()
            flag_recent_preview = True
            print("flag_recent_preview: ",flag_recent_preview)
        
        if flag_remote_streaming == True:
            self.kill_remote_streaming()
            flag_recent_remote_streaming = True
            print("flag_recent_remote_streaming: ",flag_recent_remote_streaming)

        time.sleep(1)

        self.ROI_LEFT = int(a) # ROI Xo
        self.ROI_TOP = int(b) # ROI Yo
        self.X_PIXEL_RANGE = int(c) # ROI Xf; max = 3840 - ROI_LEFT
        self.Y_PIXEL_RANGE = int(d) # ROI Yf; max = 2484 - ROI_TOP
        self.ROI_WIDTH = int(self.X_PIXEL_RANGE) 
        self.ROI_HEIGHT = int(self.Y_PIXEL_RANGE)
        # reset camera just to be sure of good operation
        PxLApi.uninitialize(self.hCamera)

        # reinitialize the camera
        try:
            self.ret = PxLApi.initialize(0)
            if not PxLApi.apiSuccess(self.ret[0]):
                print("Error: Unable to initialize a camera")
                return None
            self.hCamera = self.ret[1]
        except Exception as e:
            print(e)

        # Set the sensor ROI to a fixed size (https://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python)
        roi = [self.ROI_LEFT, self.ROI_TOP, self.ROI_WIDTH, self.ROI_HEIGHT]
        print("desired new roi: ",roi)
        
        # Step 2
        # Call getCameraFeatures for FeatuerId.ROI. We do this so we can determine 
        # the ROI limits. Like all pixelinkWrapper functions, getCameraFeatures 
        # returns a tuple with number of elements. More specifically, 
        # getCameraFeatures will return:
        # ret[0] - Return code
        # ret[1] - A list of features. Given that we are requesting on a specific
        #          feature, there should only be one element in this list
        ret = PxLApi.getCameraFeatures(self.hCamera, PxLApi.FeatureId.ROI)
        if not PxLApi.apiSuccess(ret[0]):
            print ("Could not getCameraFeatures on ROI, ret: %d!" % ret[0])

        cameraFeatures = ret[1]
        assert 1 == cameraFeatures.uNumberOfFeatures
        maxWidth  = cameraFeatures.Features[0].Params[PxLApi.RoiParams.WIDTH].fMaxValue
        maxHeight = cameraFeatures.Features[0].Params[PxLApi.RoiParams.HEIGHT].fMaxValue
        print ("This camera has a max ROI of %d x %d" % (maxWidth, maxHeight))

        # Step 3
        # Call setFeature for Feature.ROI.  
        # We will set the ROI to be the input of set_roi.
        params = []
        
        params.insert(PxLApi.RoiParams.LEFT, self.ROI_LEFT)
        params.insert(PxLApi.RoiParams.TOP, self.ROI_TOP)
        params.insert(PxLApi.RoiParams.WIDTH, self.ROI_WIDTH)
        params.insert(PxLApi.RoiParams.HEIGHT, self.ROI_HEIGHT)
        
        print("new ROI to set: ",params)

        ret = PxLApi.setFeature(self.hCamera, PxLApi.FeatureId.ROI, PxLApi.FeatureFlags.MANUAL, params)
        if not PxLApi.apiSuccess(ret[0]):
            print ("Could not setFeature on ROI, ret: %d!" % ret[0])
            return

        # Step 4
        # Read back the ROI, as the camera may have had to adjust the ROI slightly
        # to accomodate sensor restrictions
        roundingRequired = PxLApi.ReturnCode.ApiSuccessParametersChanged == ret[0]

        # Call getFeature. Like all pixelinkWrapper functions, getFeature 
        # returns a tuple with number of elements. More specifically, 
        # getFeature will return:
        # ret[0] - Return code
        # ret[1] - A bit mask of PxLApi.FeatureFlags
        # ret[2] - A list of parameters. The number of elements in the list varies
        #          with the feature
        ret = PxLApi.getFeature(self.hCamera, PxLApi.FeatureId.ROI)
        if PxLApi.apiSuccess(ret[0]):
            flags = ret[1]
            print(flags)
            updatedParams = ret[2]
            a = updatedParams[PxLApi.RoiParams.LEFT]                        
            b = updatedParams[PxLApi.RoiParams.TOP]
            c = updatedParams[PxLApi.RoiParams.WIDTH]
            d = updatedParams[PxLApi.RoiParams.HEIGHT]
            print("configured actual roi = ",a,b,c,d)

        if roundingRequired:
            print ("Warning -- The camera had to make a small adjustment to the ROI")
        
        if flag_recent_preview == True:
            self.start_preview()
        
        if flag_recent_remote_streaming == True:
            self.start_remote_streaming()

        return updatedParams[PxLApi.RoiParams.LEFT], updatedParams[PxLApi.RoiParams.TOP], updatedParams[PxLApi.RoiParams.WIDTH], updatedParams[PxLApi.RoiParams.HEIGHT]

    def get_roi(self):
        ret = PxLApi.getFeature(self.hCamera, PxLApi.FeatureId.ROI)
        if PxLApi.apiSuccess(ret[0]):
            flags = ret[1]
            print(flags)
            updatedParams = ret[2]
            a = updatedParams[PxLApi.RoiParams.LEFT]                        
            b = updatedParams[PxLApi.RoiParams.TOP]
            c = updatedParams[PxLApi.RoiParams.WIDTH]
            d = updatedParams[PxLApi.RoiParams.HEIGHT]
            print("configured actual roi = ",a,b,c,d)

    # ------------------------------------------------------- FUNCTIONS FOR MAIN OVERLAPPED NUMPY STREAMING ------------------------------------------------------- #
    
    def no_camera(self,rc):
        if rc == PxLApi.ReturnCode.ApiNoCameraError or rc == PxLApi.ReturnCode.ApiNoCameraAvailableError:
            return True
        return False

    def getPixelType(self,hCamera):
        """ Gets Pixelink camera configured pixel type. 
        IMPORTANT NOTE: this function will only return a meaningful value 
        if called while NOT streaming.
        
        Return:
            pixelType
        """

        pixelType = self.PT_OTHERWISE
        
        savedPixelFormat = 0
        newPixelFormat = 0
        ret = PxLApi.getFeature(hCamera, PxLApi.FeatureId.PIXEL_FORMAT)
        if not PxLApi.apiSuccess(ret[0]):
            print("ERROR with pixelType")
            return pixelType

        params = ret[2]
        savedPixelFormat = int(params[0])

        # Is it mono?
        newPixelFormat = PxLApi.PixelFormat.MONO16 # PxLApi.PixelFormat.MONO8
        print("newPixelFormat MONO16 parameters: ", newPixelFormat)

        params = [newPixelFormat,]
        print("Pixelink parameters: ", params)

        ret = PxLApi.setFeature(hCamera, PxLApi.FeatureId.PIXEL_FORMAT, PxLApi.FeatureFlags.MANUAL, params)
        if not PxLApi.apiSuccess(ret[0]):
            # Nope, so is it color?
            newPixelFormat = PxLApi.PixelFormat.BAYER8
            params = [newPixelFormat,]
            ret = PxLApi.setFeature(hCamera, PxLApi.FeatureId.PIXEL_FORMAT, PxLApi.FeatureFlags.MANUAL, params)
            if PxLApi.apiSuccess(ret[0]):
                # Yes, it IS color
                pixelType = self.PT_COLOR
        else:
            # Yes, it IS mono
            pixelType = self.PT_MONO

        # Restore the saved pixel format
        params = [savedPixelFormat,]
        PxLApi.setFeature(hCamera, PxLApi.FeatureId.PIXEL_FORMAT, PxLApi.FeatureFlags.MANUAL, params)

        return pixelType

    def showFrame(self,image_array):  
        """ This function creates a GUI window for display.
        """ 
        try:
            image = cv2.resize(image_array, (1920,1080))
            cv2.imshow('Pixelink Vision live-stream', image)
            cv2.waitKey(1) & 0xFF # this allows image window to update according to camera fps - refresh time  
        except Exception as e:
            print("Error showing image: ", e)
            cv2.destroyAllWindows() 

    def set_time(self,N):
        """ This function sets camera acquisition time [s] for the main_stream_overlapped function.
        """
        self.time = int(N)
        return self.time

    # ------------------------------------------------------- FUNCTIONS FOR REMOTE STREAMING ------------------------------------------------------- #
    def remote_stream(self):
        """ Sends a numpy stream to a mqtt broker.
        """
        global flag_remote_streaming

        # reset camera just to be sure of good operation
        PxLApi.uninitialize(self.hCamera)

        # reinitialize the camera
        try:
            self.ret = PxLApi.initialize(0)
            if not PxLApi.apiSuccess(self.ret[0]):
                print("Error: Unable to initialize a camera")
                return None
            self.hCamera = self.ret[1]
        except Exception as e:
            print(e)

        # Step 1 
        # Prepare the camera
        
        # Set the sensor ROI to a fixed size (https://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python)
        roi = [self.ROI_LEFT, self.ROI_TOP, self.ROI_WIDTH, self.ROI_HEIGHT]
        ret = PxLApi.setFeature(self.hCamera, PxLApi.FeatureId.ROI, PxLApi.FeatureFlags.MANUAL, roi)
        # assert PxLApi.apiSuccess(ret[0])
        if not PxLApi.apiSuccess(ret[0]):
            print("----------------------------------------------------")
            print("Error: Unable to set requested ROI in camera sensor.")
            print("----------------------------------------------------")
            return self.EXIT_FAILURE
            
        # Step 2
        # Figure out if this is a mono or color camera, so that we know the type of image we will be working with.
        pixelType = self.getPixelType(self.hCamera)
        if self.PT_OTHERWISE == pixelType:
            print("Error: We can't deal with this type of camera")
            PxLApi.uninitialize(self.hCamera)
            return self.EXIT_FAILURE

        # Just going to declare a very large buffer here
        # One that's large enough for any PixeLINK 4.0 camera
        rawFrame = create_string_buffer(5000 * 5000 * 2) # -> ctypes pointer tutorial: https://dbader.org/blog/python-ctypes-tutorial

        # Step 3
        # Start the stream and grab an image from the camera.
        ret = PxLApi.setStreamState(self.hCamera, PxLApi.StreamState.START)
        if not PxLApi.apiSuccess(ret[0]):
            print("Error: Unable to start the stream on the camera")
            PxLApi.uninitialize(self.hCamera)
            return self.EXIT_FAILURE

        getNextFramesPerItteration = 1 

        frameCount = badFrameCount = 0
        startTime = currentTime = time.time() * 1000.0 # in milliseconds!

        result_arr = []
        result_bytes = [] 
        
        print("Entering preview + overlap while loop...")

        while 1:

            for i in range(getNextFramesPerItteration): 
                ret = PxLApi.getNextFrame(self.hCamera, rawFrame)
                frameDesc = ret[1]
                
                if PxLApi.apiSuccess(ret[0]):

                    frameCount += 1
                    print("Frame number: ", frameCount)

                    # Convert it to a formatedImage. Note that frame can be in any one of a large number of pixel formats, so we will convert it to mono16, and all color to rgb24.
                    if self.PT_MONO == pixelType:
                        # ret = PxLApi.formatImage(rawFrame, frameDesc, PxLApi.ImageFormat.RAW_MONO8) # with MONO16 RAW_MONO8 with numpy.uint8 (generates 480000 values that can be reshaped to 800*600 = 480000 pixels)
                        ret = PxLApi.formatImage(rawFrame, frameDesc, PxLApi.ImageFormat.TIFF) # with MONO16 TIFF with numpy.uint8 (generates (960146 values that  that can't be reshaped to 800*600 = 480000 pixels)
                        # with MONO16 TIFF with numpy.uint16 (generates (480073 values that can't be reshaped to ->  800*600 = 480000 pixels)
                        print("PT_MONO formatImage.")
                    else:
                        ret = PxLApi.formatImage(rawFrame, frameDesc, PxLApi.ImageFormat.RAW_RGB24)

                    if PxLApi.apiSuccess(ret[0]):
                        formatedImage = ret[1]
                        print("Image bytes: ", formatedImage) # variable that hold the memory address of ctypes.c_char_Array ...> -> {c_string_address}
                        print("formatedImage value: ", formatedImage.value) # see the same every loop
                        
                        print("ret[0]: ", ret[0])
                        print("ret[1]: ", ret[1])

                        print("formatedImage (first 146 values): ", formatedImage[:146])
                        print("formatedImage (next 146 values): ", formatedImage[147:292]) 
                        # The b'...' notation is somewhat confusing in that it allows the bytes 0x01-0x7F to be specified with ASCII characters instead of hex numbers.
                        print("formatedImage data type: ", type(formatedImage))

                        # Image conditioning for image preview in opencv2 (https://stackoverflow.com/questions/17170752/python-opencv-load-image-from-byte-string)
                        img_as_np = numpy.frombuffer(formatedImage, dtype=numpy.uint8)
                        img_1 = cv2.imdecode(img_as_np, flags=1)
                        print("matrix size of Pixelink livestream: ",numpy.shape(img_1))
                    
                        self.showFrame(img_1) # showing the image in a local display

                        # ---------------------------------- PROCESSING FOR MQTT STREAMING --------------------------------------------------
                        
                        retval, buffer = cv2.imencode('.jpg', img_1)

                        jpg_as_text = base64.b64encode(buffer)
                        print("jpg_as_text: ",jpg_as_text)
                        
                        self.clientmqtt.publish(self.topic, jpg_as_text, qos = 2)
                        
                        # ----------------------------------------------------------------------------------------
                        
                else:
                    print("Error: Could not grab an image from the camera")
                    badFrameCount += 1                
                    if self.no_camera(ret[0]):
                        print("Camera is Gone!! -- Aborting")
                        return 1 # No point is continuing
                    break # Do a time check to see if we are done.
            currentTime = time.time() * 1000.0

            if flag_remote_streaming == False:
                break

            if currentTime >= (startTime + self.time*1000):
                break
            if currentTime <= (startTime + 200):
                getNextFramesPerItteration = getNextFramesPerItteration << 1

        # Step 5
        # Cleanup
        ret = PxLApi.setStreamState(self.hCamera, PxLApi.StreamState.STOP)

        print("Received %i frames (%i bad), or %8.2f frames/second.  %i getNextFrames/timecheck" %
            (frameCount+badFrameCount, badFrameCount, (frameCount+badFrameCount)/((currentTime - startTime) / 1000.0),
            getNextFramesPerItteration))

        assert PxLApi.apiSuccess(ret[0])
        
        cv2.destroyAllWindows() 
        
        flag_remote_streaming = False # for redundance

        return self.EXIT_SUCCESS

    def start_remote_streaming(self):
        """ Starts a thread that generates a numpy stream (for a limited time) for mqtt.
        """
        global flag_remote_streaming
        global flag_preview
        global aa
        if flag_preview == False:
            if self.MQTT_FLAG == True:
                aa = threading.Thread(target=self.remote_stream, daemon=True)
                aa.start()
                flag_remote_streaming = True
                return aa
            else:
                print("can't stream remote, check mqtt connection first")
        
    def kill_remote_streaming(self):
        """ Kills start_remote_streaming thread.
        """
        global flag_remote_streaming
        flag_remote_streaming = False
        global aa
        
        return aa

    # ------------------------------------------------------- FUNCTIONS FOR MAIN SNAPSHOT ------------------------------------------------------- #
    
    def get_snapshot(self,hCamera, imageFormat, fileName):
        """ Get a snapshot from the camera, and save to a file.
        """
        assert 0 != hCamera
        assert fileName
        
        # Determine the size of buffer we'll need to hold an image from the camera
        rawImageSize = self.determine_raw_image_size(hCamera)
        if 0 == rawImageSize:
            return FAILURE

        # Create a buffer to hold the raw image
        rawImage = create_string_buffer(rawImageSize)

        if 0 != len(rawImage):
            # Capture a raw image. The raw image buffer will contain image data on success. 
            ret = self.get_raw_image(hCamera, rawImage)
            if PxLApi.apiSuccess(ret[0]):
                frameDescriptor = ret[1]
                
                assert 0 != len(rawImage)
                assert frameDescriptor
                
                # Encode the raw image into something displayable
                ret = PxLApi.formatImage(rawImage, frameDescriptor, imageFormat)
                if SUCCESS == ret[0]:
                    formatedImage = ret[1]

                    # ------------ Do any image processing here ------------
                    print("Image bytes: ", formatedImage)
                    npFormatedImage = numpy.full_like(formatedImage, formatedImage, order="C") # an array
                    
                    # Reshape the numpy ndarray into multidimensional array
                    imageHeight = int(frameDescriptor.Roi.fHeight)
                    imageWidth = int(frameDescriptor.Roi.fWidth)

                    img_as_np = numpy.frombuffer(formatedImage, dtype=numpy.uint8)
                    img_1 = cv2.imdecode(img_as_np, flags=1)
                    print("Array of acquired image: ", npFormatedImage)

                    # ----------------- ----------------------------- -----------------
                    # Save formated image into a file                    
                    if self.save_image_to_file(fileName, formatedImage) == SUCCESS:
                        return SUCCESS    
                
        return FAILURE

    def determine_raw_image_size(self,hCamera):
        """ Query the camera for region of interest (ROI), decimation, and pixel format
        Using this information, we can calculate the size of a raw image

        Raises:
            return a 0 if error.     
        """
        assert 0 != hCamera

        # Get region of interest (ROI)
        ret = PxLApi.getFeature(hCamera, PxLApi.FeatureId.ROI)
        params = ret[2]
        roiWidth = params[PxLApi.RoiParams.WIDTH]
        roiHeight = params[PxLApi.RoiParams.HEIGHT]

        # Query pixel addressing
            # assume no pixel addressing (in case it is not supported)
        pixelAddressingValueX = 1
        pixelAddressingValueY = 1

        ret = PxLApi.getFeature(hCamera, PxLApi.FeatureId.PIXEL_ADDRESSING)
        if PxLApi.apiSuccess(ret[0]):
            params = ret[2]
            if PxLApi.PixelAddressingParams.NUM_PARAMS == len(params):
                # Camera supports symmetric and asymmetric pixel addressing
                pixelAddressingValueX = params[PxLApi.PixelAddressingParams.X_VALUE]
                pixelAddressingValueY = params[PxLApi.PixelAddressingParams.Y_VALUE]
            else:
                # Camera supports only symmetric pixel addressing
                pixelAddressingValueX = params[PxLApi.PixelAddressingParams.VALUE]
                pixelAddressingValueY = params[PxLApi.PixelAddressingParams.VALUE]

        # We can calulate the number of pixels now.
        numPixels = (roiWidth / pixelAddressingValueX) * (roiHeight / pixelAddressingValueY)
        ret = PxLApi.getFeature(hCamera, PxLApi.FeatureId.PIXEL_FORMAT)

        # Knowing pixel format means we can determine how many bytes per pixel.
        params = ret[2]
        pixelFormat = int(params[0])

        # And now the size of the frame
        pixelSize = PxLApi.getBytesPerPixel(pixelFormat)

        return int(numPixels * pixelSize)

    def get_raw_image(self,hCamera, rawImage):
        """ Capture an image from the camera.

        NOTE: PxLApi.getNextFrame is a blocking call. 

        i.e. PxLApi.getNextFrame won't return until an image is captured.
        So, if you're using hardware triggering, it won't return until the camera is triggered.

        Returns a return code with success and frame descriptor information or API error
        """
        assert 0 != hCamera
        assert 0 != len(rawImage)

        MAX_NUM_TRIES = 10

        result_arr = []
        result_bytes = []

        # Put camera into streaming state so we can capture an image
        ret = PxLApi.setStreamState(hCamera, PxLApi.StreamState.START)
        if not PxLApi.apiSuccess(ret[0]):
            return FAILURE
        
        # Get an image
        # NOTE: PxLApi.getNextFrame can return ApiCameraTimeoutError on occasion.
        # How you handle this depends on your situation and how you use your camera. 
        # For this sample app, we'll just retry a few times.
        ret = (PxLApi.ReturnCode.ApiUnknownError,)

        for i in range(MAX_NUM_TRIES):
            ret = PxLApi.getNextFrame(hCamera, rawImage)
            if PxLApi.apiSuccess(ret[0]):
                break # do not break if you want for loop

        # Done capturing, so no longer need the camera streaming images.
        # Note: If ret is used for this call, it will lose frame descriptor information.
        PxLApi.setStreamState(hCamera, PxLApi.StreamState.STOP)

        return ret # returns acquired image data

    def save_image_to_file(self,fileName, formatedImage):
        """ Save the encoded image buffer to a file
        This overwrites any existing file

        Returns SUCCESS or FAILURE.
        """            
        assert fileName
        assert 0 != len(formatedImage)

        # Create a folder to save snapshots if it does not exist 
        if not os.path.exists("getSnapshot"):
            os.makedirs("getSnapshot")

        filepath = "getSnapshot/" + fileName
        # Open a file for binary write
        file = open(filepath, "wb")
        if None == file:
            return FAILURE
        
        numBytesWritten = file.write(formatedImage)
        file.close()

        if numBytesWritten == len(formatedImage):
            return SUCCESS

        return FAILURE

    def main_snapshot(self):
        """ This function triggers camera snapshots.
        """
        global flag_stream_overlapped
        global flag_preview
        
        flag_recent_stream_overlapped = False
        flag_recent_preview = False

        if flag_stream_overlapped == True:
            self.kill_stream_overlapped()
            flag_recent_stream_overlapped = True

        if flag_preview == True:
            self.kill_preview()
            flag_recent_preview = True

        try:
            ret = PxLApi.initialize(0)
            if PxLApi.apiSuccess(ret[0]):
                self.hCamera = ret[1]
        except Exception as e:
            print(e)

        time_of_snapshot = str(datetime.datetime.today().timestamp())
        print("snapshot timestamp: ",time_of_snapshot)
        
        filenameJpeg = "snapshot.jpg"
        filenameBmp = "snapshot.bmp"
        filenameTiff = "snapshot_" + time_of_snapshot + ".tiff"        
        filenamePsd = "snapshot.psd"
        filenameRgb24 = "snapshot.rgb24.bin"
        filenameRgb24Nondib = "snapshot.rgb24nondib.bin"
        filenameRgb48 = "snapshot.rgb48.bin"
        filenameMono8 = "snapshot.mono8.bin"

        # Get a snapshot and save it to a folder as a file
        '''
        retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.JPEG, filenameJpeg)
        if SUCCESS == retVal:
            print("Saved image to 'getSnapshot/%s'" % filenameJpeg)
            retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.BMP, filenameBmp)
            if SUCCESS == retVal:
                print("Saved image to 'getSnapshot/%s'" % filenameBmp)
                retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.TIFF, filenameTiff) # Changing for experimentation: PxLApi.ImageFormat.TIFF <-> PxLApi.ImageFormat.RAW_MONO8
                if SUCCESS == retVal:
                    print("Saved image to 'getSnapshot/%s'" % filenameTiff)
                    retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.PSD, filenamePsd)
                    if SUCCESS == retVal:
                        print("Saved image to 'getSnapshot/%s'" % filenamePsd)
                        retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.RAW_BGR24, filenameRgb24)
                        if SUCCESS == retVal:
                            print("Saved image to 'getSnapshot/%s'" % filenameRgb24)
                            retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.RAW_BGR24_NON_DIB, filenameRgb24Nondib)
                            if SUCCESS == retVal:
                                print("Saved image to 'getSnapshot/%s'" % filenameRgb24Nondib)
                                retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.RAW_RGB48, filenameRgb48)
                                if SUCCESS == retVal:
                                    print("Saved image to 'getSnapshot/%s'" % filenameRgb48)
                                    # Only capture MONO8 for monochrome cameras
                                    
                                    retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.RAW_MONO8, filenameMono8)
                                    if SUCCESS == retVal:
                                        print("Saved image to 'getSnapshot/%s'" % filenameMono8)             
        '''
        retVal = self.get_snapshot(self.hCamera, PxLApi.ImageFormat.TIFF, filenameTiff)
        if SUCCESS == retVal:
            print("Saved image to 'getSnapshot/%s'" % filenameTiff)

        if SUCCESS != retVal:
            print("ERROR: Unable to capture an image")
            return FAILURE

        if flag_recent_preview == True:
            self.start_preview()

        if flag_recent_stream_overlapped == True:
            self.start_stream_overlapped()

        return SUCCESS

    # ------------------------------------------------------- FUNCTIONS + FOR MAIN PREVIEW ------------------------------------------------------- #

    def set_preview_state(self,hCamera):
        """ This function set the camera state to trigger preview.
        """
        global flag_preview

        # Start the preview (NOTE: camera must be streaming)
        ret = PxLApi.setPreviewState(hCamera, PxLApi.PreviewState.START)
        if ret[0] == PxLApi.ReturnCode.ApiNotSupportedOnLiteVersion:
            return ret
        assert PxLApi.apiSuccess(ret[0]), "%i" % ret[0]

        while True:
            if flag_preview == False:
                break
    
        # Stop the preview
        ret = PxLApi.setPreviewState(hCamera, PxLApi.PreviewState.STOP)
        assert PxLApi.apiSuccess(ret[0]), "%i" % ret[0]

        return ret

    def main_preview(self):
        """ This function creates a camera preview window. This function blocks the API. Must be triggered with a thread (see start_preview function in this API).
        """
        # Just use all of the camera's default settings.
        # Start the stream
        try:
            ret = PxLApi.setStreamState(self.hCamera, PxLApi.StreamState.START)
        except Exception as e:
            print(e)
            return self.EXIT_FAILURE
        assert PxLApi.apiSuccess(ret[0]), "%i" % ret[0]

        # Start and stop preview using setPreviewState
        ret = self.set_preview_state(self.hCamera) # blocking function until broken
        
        if ret[0] == PxLApi.ReturnCode.ApiNotSupportedOnLiteVersion:
            print("ERROR: Api Lite detected -- this application requires the standard Pixelink API")
            
            # Clean up by stopping the stream and uninitialize the camera
            ret = PxLApi.setStreamState(self.hCamera, PxLApi.StreamState.STOP)
            assert PxLApi.apiSuccess(ret[0]), "%i" % ret[0]
            PxLApi.uninitialize(self.hCamera)
            return 1
        
        return ret
        
    def start_preview(self):
        """ This function creates a thread for the camera window function.
        """
        global flag_preview
        global flag_stream_overlapped
        global b
        
        if flag_stream_overlapped == False:
            b = threading.Thread(target=self.main_preview, daemon=True)
            b.start()
            flag_preview = True
        else:
            b = "overlapped streaming is ongoing"
        return b

    def kill_preview(self):
        """ This function kills the thread responsible for the camera window preiew.
        """
        global flag_preview 
        global b

        flag_preview = False
        return b

    # --------------------------------------- FUNCTIONS + FOR MAIN OPENCVSNAPSHOT ------------------------------------------------------- #

    def main_openCVsnapshot(self):
        """ Returns an image in numerical matrix openCV format.
        """
        global flag_stream_overlapped
        global flag_preview
        
        flag_recent_stream_overlapped = False
        flag_recent_preview = False

        if flag_stream_overlapped == True:
            self.kill_stream_overlapped()
            flag_recent_stream_overlapped = True
            time.sleep(0.3)

        if flag_preview == True:
            self.kill_preview()
            flag_recent_preview = True

        time.sleep(0.3)

        # Figure out if this is a mono or color camera, so that we know the type of
        # image we will be working with.
        pixelType = self.getPixelType(self.hCamera)
        
        # Just going to declare a very large buffer here
        # One that's large enough for any PixeLINK 4.0 camera
        rawFrame = create_string_buffer(5000 * 5000 * 2)

        # Start the stream and Grab an image from the camera.
        ret = PxLApi.setStreamState(self.hCamera, PxLApi.StreamState.START)
        if not PxLApi.apiSuccess(ret[0]):
            print("Error: Unable to start the stream on the camera")
            PxLApi.uninitialize(self.hCamera)
            return EXIT_FAILURE

        ret = PxLApi.getNextFrame(self.hCamera, rawFrame)
        frameDesc = ret[1]
        
        if PxLApi.apiSuccess(ret[0]):
            # Convert it to a formatedImage. Note that frame can be in any one of a large number of pixel
            # formats, so we will simplify things by converting all mono to mono8, and all color to rgb24
            if PT_MONO == pixelType:
                ret = PxLApi.formatImage(rawFrame, frameDesc, PxLApi.ImageFormat.RAW_MONO8)
            else:
                ret = PxLApi.formatImage(rawFrame, frameDesc, PxLApi.ImageFormat.RAW_RGB24)
            
            if PxLApi.apiSuccess(ret[0]):
                formatedImage = ret[1]
        
                # 'convert' the formatedImage buffer to a numpy ndarray that OpenCV can manipulate
                npFormatedImage = numpy.full_like(formatedImage, formatedImage, order="C") # a numpy ndarray
                npFormatedImage.dtype = numpy.uint8
                # Reshape the numpy ndarray into multidimensional array
                imageHeight = int(frameDesc.Roi.fHeight)
                imageWidth = int(frameDesc.Roi.fWidth)

                # color has 3 channels, mono just 1
                if PT_MONO == pixelType:
                    newShape = (imageHeight, imageWidth)
                else:
                    newShape = (imageHeight, imageWidth, 3)
                npFormatedImage = numpy.reshape(npFormatedImage, newShape)

                # Do OpenCV manipulations on the numpy ndarray here.
                # We will simply use OpenCV to save the image as a BMP
                # Create a folder to save snapshots if it does not exist 
                if not os.path.exists("openCVSnapshot"):
                    os.makedirs("openCVSnapshot")

                time_of_snapshot = str(datetime.datetime.today().timestamp())
                print("snapshot timestamp: ",time_of_snapshot)
                filenameTiff = "openCVSnapshot/snapshot_" + time_of_snapshot + ".tiff"        

                retVal = cv2.imwrite(filenameTiff, npFormatedImage)
                if retVal:
                    print("Saved image to 'openCVSnapshot' folder")
                else:
                    print("Error: Could not save image to 'openCVSnapshot' folder")
        else:
            print("Error: Could not grab an image from the camera")

        # Cleanup
        ret = PxLApi.setStreamState(self.hCamera, PxLApi.StreamState.STOP)
        assert PxLApi.apiSuccess(ret[0])
        print("acquired image matrix shape: ",numpy.shape(npFormatedImage))
        print("acquired image matrix: ",npFormatedImage)

        if flag_recent_preview == True:
            self.start_preview()

        if flag_recent_stream_overlapped == True:
            self.start_stream_overlapped()

        return EXIT_SUCCESS, npFormatedImage