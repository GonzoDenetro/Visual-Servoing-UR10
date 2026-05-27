import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

# Diccionario ArUco
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)

# Parámetros
marker_id = 10
marker_size = 70  # pixeles

# Generar marcador (CORRECTO en OpenCV 4.5.x)
marker_img = cv.aruco.drawMarker(aruco_dict, marker_id, marker_size)

# Agregar borde blanco
border = 50
marker_with_border = cv.copyMakeBorder(
    marker_img,
    border, border, border, border,
    cv.BORDER_CONSTANT,
    value=255
)

# Guardar
cv.imwrite('marker.png', marker_with_border)

# Mostrar
plt.imshow(marker_with_border, cmap='gray')
plt.axis('off')
plt.title(f'ArUco Marker {marker_id}')
plt.show()