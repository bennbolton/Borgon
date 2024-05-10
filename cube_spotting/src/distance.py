

focalLength_mm = 3.67
imageHeight = 480
realHeight = 53
D = 16/2.88
pixelHeight = 160

sensorHeight = 3 * (D**2 / 25)**0.5
distance = (focalLength_mm*realHeight*imageHeight) / (pixelHeight * sensorHeight)
print(distance)