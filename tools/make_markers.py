import os
import cv2
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
IDS = {"Ward": 1, "ICU": 2, "Pharmacy": 3, "Surgery": 4}
os.makedirs("markers", exist_ok=True)
for n, i in IDS.items():
    try:
        m = cv2.aruco.generateImageMarker(D, i, 400)
    except AttributeError:
        m = cv2.aruco.drawMarker(D, i, 400)
    m = cv2.copyMakeBorder(m, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(f"markers/{n}_{i}.png", m)
    print("saved", n, i)