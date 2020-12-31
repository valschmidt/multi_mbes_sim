#!/usr/bin/env python

import gdal
import numpy as np
import scipy.interpolate

class BathyGrid:
    def __init__(self, fname, mapOriginLatitude=None, mapOriginLongitude = None):

        self.dataset = gdal.Open(fname, gdal.GA_ReadOnly)
        # print 'opened',self.dataset.GetDescription()
        self.band = self.dataset.GetRasterBand(1)
        self.geoTransform = self.dataset.GetGeoTransform()
        self.inverseGeoTransorm = gdal.InvGeoTransform(self.geoTransform)
        # print self.geoTransform
        self.data = self.band.ReadAsArray()
        # print self.data.shape

        sourceSR = gdal.osr.SpatialReference()
        sourceSR.SetWellKnownGeogCS("WGS84")

        targetSR = gdal.osr.SpatialReference()
        targetSR.ImportFromWkt(self.dataset.GetProjection())

        self.coordinateTransformation = gdal.osr.CoordinateTransformation(sourceSR, targetSR)

        self.setMapOrigin()


        # Returns a 2D inteprolation function over the grid.
        # 'bounds_error' and 'fill_value' are left unset, which will result
        # in values extrapolated via nearest-neighbor extrapolation when
        # requests are made outside the domain.
        self.getDepthAtMapXY = scipy.interpolate.interp2d(x=self.BathyDataXXmap,
                                                          y=self.BathyDataYYmap,
                                                          z=self.data,
                                                          kind='linear',
                                                          copy=False)

    def setMapOrigin(self,mapOriginLatitude = None, mapOriginLongitude = None):

        # If the ROS "map" CRS origin is not specified, then set the map origin to
        # the bathy data origin.
        if mapOriginLatitude == None:
            mapOriginLatitude = self.geoTransform[0]
            mapOriginLongitude = self.geoTransform[3]

        # Get the Latitude and Longitude of the vessel's "map" frame origin in the
        # bathy data's coordinate reference system.
        mapOriginEasting, mapOriginNorthing, mapOriginUp = \
            self.coordinateTransformation.TransformPoint(mapOriginLongitude,
                                                         mapOriginLatitude)

        # Get indices for each pixel. Add 0.5 to these so the resulting
        # calculation returns the position of the center of each pixel.
        rows, cols = self.data.shape
        ii = np.arange(cols) + 0.5
        jj = np.arange(rows) + 0.5

        if self.geoTransform[2] != 0 or self.geoTransform[4] != 0:
            print("ERROR: This data file has rotation, which is not currently supported.")
            return

        # Get the coordinate of the centers of each X/Y pixel.
        BathyDataXX = self.geoTransform[0] + ii * self.geoTransform[1]
        BathyDataYY = self.geoTransform[3] + jj * self.geoTransform[5]

        # Calculate the centers of each pixel in the map coordinate reference frame.
        # These are vectors, so the map coordiantes of pixel i,j are
        # (self.BathyDataXXmap[i], self.BathyDataYYmap[j])
        self.BathyDataXXmap = BathyDataXX - mapOriginEasting
        self.BathyDataYYmap = BathyDataYY - mapOriginNorthing

    def getXY(self, lat, lon):
        return self.coordinateTransformation.TransformPoint(lon, lat)[:2]

    def getDepthAtLatLon(self, lat, lon):
        x, y, z = self.coordinateTransformation.TransformPoint(lon, lat)
        return self.getDepth(x, y)

    def getDepth(self, x, y):
        xi = self.inverseGeoTransorm[0] + x * self.inverseGeoTransorm[1] + y * self.inverseGeoTransorm[2]
        yi = self.inverseGeoTransorm[3] + x * self.inverseGeoTransorm[4] + y * self.inverseGeoTransorm[5]
        # print xi,yi
        try:
            return self.data[int(yi), int(xi)]
        except IndexError:
            return None