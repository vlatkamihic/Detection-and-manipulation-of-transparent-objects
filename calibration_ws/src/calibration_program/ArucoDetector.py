import os
import aruco

class ArucoDetector:
  def __init__(self):
    self.camparam = aruco.CameraParameters()
    paramPath = os.path.join(os.path.dirname(__file__), "kamera-parameters.yml")
    dictPath = os.path.join(os.path.dirname(__file__), "4x4_1000.dict")
    
    self.camparam.readFromXMLFile(paramPath)
    # create detector and get parameters
    self.detector = aruco.MarkerDetector()
    self.detector.setDictionary(dictPath)
    #self.params = self.detector.getParameters()
