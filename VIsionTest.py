from Vision import Vision
import time

VS = Vision()

VS.Tracking()
start = time.time()



while True:

    if time.time() - start > 5:
        VS.Running = False
        print("Stopped by user")
