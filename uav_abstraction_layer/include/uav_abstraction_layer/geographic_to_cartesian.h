/**
 * Author: Ángel Manuel Montes Romero
 *
 * Geographic to Cartesian coordinate conversion and vice versa, with an intermediate conversion to UTM.
 *
 * This library suppose that the path of the robot cannot cross more than 2 different UTM zones.
 *
 */

#ifndef GEOGRAPHIC_TO_CARTESIAN
#define GEOGRAPHIC_TO_CARTESIAN

#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>

#include <geodesy/utm.h>                // IMPORTANT TO INSTALL GEODESY: sudo apt-get install ros-kinetic-geographic-info


inline geometry_msgs::Point32 geographic_to_cartesian (const geographic_msgs::GeoPoint& _actual_coordinate_geo, const geographic_msgs::GeoPoint& _origin_geo) {

    geodesy::UTMPoint actual_coordinate_UTM(_actual_coordinate_geo); // Conversion from geographic coordinates to UTM.
    geodesy::UTMPoint origin_UTM(_origin_geo);                       // Conversion from geographic coordinates to UTM.

    geometry_msgs::Point32 actual_coordinate_cartesian;              // Cartesian coordinate that this function will return.

    // The problem with UTM is that if there are coordinates in more than one zone, it's difficult to merge the coordinates of the different zones.
    // This is because each zone has 6 degrees of longitude, and the width in meters of the zone is variable from equator to the poles.
    // Something similar when the coordinates are in different hemispheres, separated by the equator. Each hemisphere has a different origin for the y axis.
    // These are the reasons why the x and y assignation for the Cartesian conversion isn't a simple UTM substraction.

    // Assignating "actual_coordinate_cartesian.x" is tricky when the actual coordinate and the origin of the Cartesian coordinates are on different zones.
    if ( int(actual_coordinate_UTM.zone)==60 && origin_UTM.zone==1 ) {         // Coordinate and origin separated by the +-180º longitude
        geographic_msgs::GeoPoint geo_180_w;
        geographic_msgs::GeoPoint geo_180_e;
        geo_180_w.longitude = 179.9999999;
        geo_180_w.latitude = _actual_coordinate_geo.latitude;
        geo_180_w.altitude = _actual_coordinate_geo.altitude;
        geo_180_e.longitude = -179.9999999;
        geo_180_e.latitude = _actual_coordinate_geo.latitude;
        geo_180_e.altitude = _actual_coordinate_geo.altitude;
        geodesy::UTMPoint utm_180_w(geo_180_w);
        geodesy::UTMPoint utm_180_e(geo_180_e);
        actual_coordinate_cartesian.x = actual_coordinate_UTM.easting - utm_180_w.easting + utm_180_e.easting - origin_UTM.easting; // Transformation of the x coordinate taking into account the different zones. " - utm_180_w.easting + utm_180_e.easting " computes the x step in both sides of the border longitude.
    } else if ( origin_UTM.zone==60 && int(actual_coordinate_UTM.zone)==1 ) {  // Coordinate and origin separated by the +-180º longitude
        geographic_msgs::GeoPoint geo_180_w;
        geographic_msgs::GeoPoint geo_180_e;
        geo_180_w.longitude = 179.9999999;
        geo_180_w.latitude = _actual_coordinate_geo.latitude;
        geo_180_w.altitude = _actual_coordinate_geo.altitude;
        geo_180_e.longitude = -179.9999999;
        geo_180_e.latitude = _actual_coordinate_geo.latitude;
        geo_180_e.altitude = _actual_coordinate_geo.altitude;
        geodesy::UTMPoint utm_180_w(geo_180_w);
        geodesy::UTMPoint utm_180_e(geo_180_e);
        actual_coordinate_cartesian.x = actual_coordinate_UTM.easting - utm_180_e.easting + utm_180_w.easting - origin_UTM.easting; // Transformation of the x coordinate taking into account the different zones. " - utm_180_w.easting + utm_180_e.easting " computes the x step in both sides of the border longitude.
    } else if ( int(actual_coordinate_UTM.zone) < origin_UTM.zone ) {
        int quotient_from_int_division = (int) ( std::max(std::abs(_actual_coordinate_geo.longitude),std::abs(_origin_geo.longitude))/6 );                    // int division of the max longitude (absolute, without sign)
        double border_longitude = quotient_from_int_division * 6.0 *pow(-1,std::signbit(std::max(_actual_coordinate_geo.longitude,_origin_geo.longitude)));   // border_longitude = quotient_from_int_division * 6 *(-1)^(1 if negative longitude, 0 if positive)
        geographic_msgs::GeoPoint geo_w;
        geographic_msgs::GeoPoint geo_e;
        geo_w.longitude = border_longitude - 0.0000001;
        geo_w.latitude = _actual_coordinate_geo.latitude;
        geo_w.altitude = _actual_coordinate_geo.altitude;
        geo_e.longitude = border_longitude + 0.0000001;
        geo_e.latitude = _actual_coordinate_geo.latitude;
        geo_e.altitude = _actual_coordinate_geo.altitude;
        geodesy::UTMPoint utm_w(geo_w);
        geodesy::UTMPoint utm_e(geo_e);
        actual_coordinate_cartesian.x = actual_coordinate_UTM.easting - utm_w.easting + utm_e.easting - origin_UTM.easting;         // Transformation of the x coordinate taking into account the different zones. " - utm_w.easting + utm_e.easting " computes the x step in both sides of the border longitude.
    } else if ( origin_UTM.zone < int(actual_coordinate_UTM.zone) ) {
        int quotient_from_int_division = (int) ( std::max(std::abs(_actual_coordinate_geo.longitude),std::abs(_origin_geo.longitude))/6 );                    // int division of the max longitude (absolute, without sign)
        double border_longitude = quotient_from_int_division * 6.0 *pow(-1,std::signbit(std::max(_actual_coordinate_geo.longitude,_origin_geo.longitude)));   // border_longitude = quotient_from_int_division * 6 *(-1)^(1 if negative longitude, 0 if positive)
        geographic_msgs::GeoPoint geo_w;
        geographic_msgs::GeoPoint geo_e;
        geo_w.longitude = border_longitude - 0.0000001;
        geo_w.latitude = _actual_coordinate_geo.latitude;
        geo_w.altitude = _actual_coordinate_geo.altitude;
        geo_e.longitude = border_longitude + 0.0000001;
        geo_e.latitude = _actual_coordinate_geo.latitude;
        geo_e.altitude = _actual_coordinate_geo.altitude;
        geodesy::UTMPoint utm_w(geo_w);
        geodesy::UTMPoint utm_e(geo_e);
        actual_coordinate_cartesian.x = actual_coordinate_UTM.easting - utm_e.easting + utm_w.easting - origin_UTM.easting;    // Transformation of the x coordinate taking into account the different zones. " - utm_w.easting + utm_e.easting " computes the x step in both sides of the border longitude.
    } else {
        // The actual coordinate is in the same zone that the origin (the first station). This is the normal situation, the assigntation of the x axis value is trivial (simple subtraction).
        actual_coordinate_cartesian.x = actual_coordinate_UTM.easting -origin_UTM.easting;
    }

    // Assignating "actual_coordinate_cartesian.y" is also tricky when the actual coordinate and the origin of the Cartesian coordinates are on different hemispheres:
    if ( origin_UTM.band=='N' && actual_coordinate_UTM.band=='M' ) {         // Coordinate and origin separated by the 0º latitude
        geographic_msgs::GeoPoint geo_0_n;
        geographic_msgs::GeoPoint geo_0_s;
        geo_0_n.longitude = _actual_coordinate_geo.longitude;
        geo_0_n.latitude = 0.0000001;
        geo_0_n.altitude = _actual_coordinate_geo.altitude;
        geo_0_s.longitude = _actual_coordinate_geo.longitude;
        geo_0_s.latitude = -0.0000001;
        geo_0_s.altitude = _actual_coordinate_geo.altitude;
        geodesy::UTMPoint utm_0_n(geo_0_n);
        geodesy::UTMPoint utm_0_s(geo_0_s);
        actual_coordinate_cartesian.y = actual_coordinate_UTM.northing - utm_0_s.northing + utm_0_n.northing - origin_UTM.northing;    // Transformation of the y coordinate taking into account the different hemispheres. " - utm_0_s.northing + utm_0_n.northing " computes the y step in both sides of the equator.
    } else if ( actual_coordinate_UTM.band=='N' && origin_UTM.band=='M' ) {  // Coordinate and origin separated by the 0º latitude
        geographic_msgs::GeoPoint geo_0_n;
        geographic_msgs::GeoPoint geo_0_s;
        geo_0_n.longitude = _actual_coordinate_geo.longitude;
        geo_0_n.latitude = 0.0000001;
        geo_0_n.altitude = _actual_coordinate_geo.altitude;
        geo_0_s.longitude = _actual_coordinate_geo.longitude;
        geo_0_s.latitude = -0.0000001;
        geo_0_s.altitude = _actual_coordinate_geo.altitude;
        geodesy::UTMPoint utm_0_n(geo_0_n);
        geodesy::UTMPoint utm_0_s(geo_0_s);
        actual_coordinate_cartesian.y = actual_coordinate_UTM.northing - utm_0_n.northing + utm_0_s.northing - origin_UTM.northing;    // Transformation of the y coordinate taking into account the different hemispheres. " - utm_0_n.northing + utm_0_s.northing " computes the y step in both sides of the equator.
    } else {
        // The actual coordinate is in the same hemisphere that the origin (the first station). This is the normal situation, the assigntation of the y axis value is trivial (simple subtraction).
        actual_coordinate_cartesian.y = actual_coordinate_UTM.northing-origin_UTM.northing;
    }

    // Assignating "actual_coordinate_cartesian.z" is trivial always, simple subtraction.
    actual_coordinate_cartesian.z = actual_coordinate_UTM.altitude-origin_UTM.altitude;

    return actual_coordinate_cartesian;
}   // end geographic_to_cartesian



inline geographic_msgs::GeoPoint cartesian_to_geographic (const geometry_msgs::Point32& _actual_coordinate_cartesian, const geographic_msgs::GeoPoint& _origin_geo) {
    geodesy::UTMPoint actual_coordinate_aux(_origin_geo);                                               // Conversion from geographic coordinates to UTM.

    actual_coordinate_aux.easting  += _actual_coordinate_cartesian.x;
    actual_coordinate_aux.northing += _actual_coordinate_cartesian.y;
    actual_coordinate_aux.altitude += _actual_coordinate_cartesian.z;

    // Convert to geographic coordinates and return the coordinate. This conversion is easier as it doesn't matter if the points are in different zones or hemispheres, but in that case the error increases if the points are very far from each other.
    return geodesy::toMsg (actual_coordinate_aux);

}   // end cartesian_to_geographic


#endif // GEOGRAPHIC_TO_CARTESIAN