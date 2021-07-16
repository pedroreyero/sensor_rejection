### -----------------------------------------------------------
# Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
# 
# East Longitudes are positive, West longitudes are negative.
# North latitudes are positive, South latitudes are negative
# Lat and Long are in fractional degrees
#
# Adapted from the code written by Chuck Gantz- chuck.gantz@globalstar.com
# Link to source:
# https://github.com/rosbook/effective_robotics_programming_with_ros/blob/master/chapter8_tutorials/src/c8_fixtoUTM.cpp
### -----------------------------------------------------------

import math

def LLtoUTM(Lat, Long):

    RADIANS_PER_DEGREE = math.pi/180.0
    DEGREES_PER_RADIAN = 180.0/math.pi

    ### WGS84 Parameters
    WGS84_A = 6378137.0                # major axis
    WGS84_B = 6356752.31424518        # minor axis
    WGS84_F = 0.0033528107                # ellipsoid flattening
    WGS84_EP = 0.0820944379                # second eccentricity

    WGS84_E = 0.0818191908                # first eccentricity

    ### UTM Parameters
    UTM_K0 = 0.9996                        # scale factor
    UTM_FE = 500000.0                # false easting
    UTM_FN_N = 0.0                        # false northing on north hemisphere
    UTM_FN_S = 10000000.0                # false northing on south hemisphere
    UTM_E2 = (WGS84_E*WGS84_E)        # e^2
    UTM_E4 = (UTM_E2*UTM_E2)                # e^4
    UTM_E6 = (UTM_E4*UTM_E2)                # e^6
    UTM_EP2 = (UTM_E2/(1-UTM_E2))        # e'^2

    a = WGS84_A
    eccSquared = UTM_E2
    k0 = UTM_K0
    
    ### Make sure the longitude is between -180.00 .. 179.9
    LongTemp = (Long+180)-int((Long+180)/360)*360-180

    LatRad = Lat*RADIANS_PER_DEGREE
    LongRad = LongTemp*RADIANS_PER_DEGREE
    
    ZoneNumber = int((LongTemp + 180)/6) + 1;
    
    if ( Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0 ):
        ZoneNumber = 32
    # print( "ZoneNumber: ", ZoneNumber )
    
    ### Special zones for Svalbard
    if( Lat >= 72.0 and Lat < 84.0 ):
        if( LongTemp >= 0.0  and LongTemp <  9.0 ):
            ZoneNumber = 31
        elif( LongTemp >= 9.0  and LongTemp < 21.0 ):
            ZoneNumber = 33
        elif( LongTemp >= 21.0 and LongTemp < 33.0 ):
                ZoneNumber = 35
        elif( LongTemp >= 33.0 and LongTemp < 42.0 ):
                ZoneNumber = 37
    ### +3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE
    # print( "Letter: ", UTMLetterDesignator(Lat) )
    ### compute the UTM Zone from the latitude and longitude
    UTMZone = str(ZoneNumber) + UTMLetterDesignator(Lat)
    
    eccPrimeSquared = (eccSquared)/(1-eccSquared)
    N = a/math.sqrt(1-eccSquared*math.sin(LatRad)*math.sin(LatRad))
    T = math.tan(LatRad)*math.tan(LatRad)
    C = eccPrimeSquared*math.cos(LatRad)*math.cos(LatRad)
    A = math.cos(LatRad)*(LongRad-LongOriginRad)
    M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64 - 5*eccSquared*eccSquared*eccSquared/256)*LatRad \
                                - (3*eccSquared/8        + 3*eccSquared*eccSquared/32        + 45*eccSquared*eccSquared*eccSquared/1024)*math.sin(2*LatRad) \
                                                                        + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*math.sin(4*LatRad) \
                                                                        - (35*eccSquared*eccSquared*eccSquared/3072)*math.sin(6*LatRad))
    UTMEasting = (k0*N*(A+(1-T+C)*A*A*A/6 \
                                        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120) \
                                        + 500000.0)
    UTMNorthing = (k0*(M+N*math.tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24 \
                                 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)))
    
    if(Lat < 0):
        UTMNorthing = UTMNorthing + 10000000.0 # 10000000 meter offset for southern hemisphere

    return UTMNorthing, UTMEasting, UTMZone



### -----------------------------------------------------------
# Determine the correct UTM letter designator for the given latitude
# 
# @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
#
# Adapted from the code written by Chuck Gantz- chuck.gantz@globalstar.com
### -----------------------------------------------------------

def UTMLetterDesignator(Lat):

    if ((84 >= Lat) and (Lat >= 72)):  LetterDesignator = 'X'
    elif ((72 > Lat) and (Lat >= 64)):  LetterDesignator = 'W'
    elif ((64 > Lat) and (Lat >= 56)):  LetterDesignator = 'V'
    elif ((56 > Lat) and (Lat >= 48)):  LetterDesignator = 'U'
    elif ((48 > Lat) and (Lat >= 40)):  LetterDesignator = 'T'
    elif ((40 > Lat) and (Lat >= 32)):  LetterDesignator = 'S'
    elif ((32 > Lat) and (Lat >= 24)):  LetterDesignator = 'R'
    elif ((24 > Lat) and (Lat >= 16)):  LetterDesignator = 'Q'
    elif ((16 > Lat) and (Lat >= 8)):   LetterDesignator = 'P'
    elif (( 8 > Lat) and (Lat >= 0)):   LetterDesignator = 'N'
    elif (( 0 > Lat) and (Lat >= -8)):  LetterDesignator = 'M'
    elif ((-8 > Lat) and (Lat >= -16)): LetterDesignator = 'L'
    elif((-16 > Lat) and (Lat >= -24)): LetterDesignator = 'K'
    elif((-24 > Lat) and (Lat >= -32)): LetterDesignator = 'J'
    elif((-32 > Lat) and (Lat >= -40)): LetterDesignator = 'H'
    elif((-40 > Lat) and (Lat >= -48)): LetterDesignator = 'G'
    elif((-48 > Lat) and (Lat >= -56)): LetterDesignator = 'F'
    elif((-56 > Lat) and (Lat >= -64)): LetterDesignator = 'E'
    elif((-64 > Lat) and (Lat >= -72)): LetterDesignator = 'D'
    elif((-72 > Lat) and (Lat >= -80)): LetterDesignator = 'C'
    # 'Z' is an error flag, the Latitude is outside the UTM limits
    else: LetterDesignator = 'Z'

    return LetterDesignator
