import csv
import time


csvfile = open("lat_long.csv", 'r')
csvread = csv.reader(csvfile)
for lines in csvread:
    lon = float(lines[1])
    lat = float(lines[2])
    canvas = iface.mapCanvas()
    pnt = QgsPointXY(lon, lat)
    m = QgsVertexMarker(canvas)
    m.setCenter(pnt)
    m.setColor(QColor('Black'))
    m.setIconType(QgsVertexMarker.ICON_CIRCLE)
    m.setIconSize(12)
    m.setPenWidth(1)
    m.setFillColor(QColor(200, 00, 0))

