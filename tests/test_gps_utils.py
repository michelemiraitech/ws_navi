"""
Tests for GPS conversion utilities.
"""

import pytest
import numpy as np
from vehicle_tools.gps.converter import GPSToUTMConverter, calculate_distance


def test_latlon_to_utm():
    """Test GPS to UTM conversion."""
    # San Francisco coordinates
    lat, lon = 37.7749, -122.4194
    easting, northing, zone_num, zone_letter = GPSToUTMConverter.latlon_to_utm(lat, lon)
    
    # Check that we get reasonable UTM coordinates
    assert isinstance(easting, float)
    assert isinstance(northing, float)
    assert isinstance(zone_num, int)
    assert isinstance(zone_letter, str)
    assert 10 <= zone_num <= 60
    assert zone_letter in 'ABCDEFGHJKLMNPQRSTUVWXYZ'


def test_utm_to_latlon():
    """Test UTM to GPS conversion."""
    # Test round-trip conversion
    lat_orig, lon_orig = 37.7749, -122.4194
    easting, northing, zone_num, zone_letter = GPSToUTMConverter.latlon_to_utm(lat_orig, lon_orig)
    lat_converted, lon_converted = GPSToUTMConverter.utm_to_latlon(easting, northing, zone_num, zone_letter)
    
    # Should be very close to original (within 1e-6 degrees)
    assert abs(lat_converted - lat_orig) < 1e-6
    assert abs(lon_converted - lon_orig) < 1e-6


def test_calculate_distance():
    """Test distance calculation between GPS points."""
    # Distance between two known points (approximately 1 degree apart)
    lat1, lon1 = 0.0, 0.0
    lat2, lon2 = 1.0, 0.0
    
    distance = calculate_distance(lat1, lon1, lat2, lon2)
    
    # 1 degree of latitude is approximately 111 km
    expected_distance = 111000  # meters
    assert abs(distance - expected_distance) < 5000  # Within 5km tolerance


def test_calculate_distance_same_point():
    """Test distance calculation for the same point."""
    lat, lon = 37.7749, -122.4194
    distance = calculate_distance(lat, lon, lat, lon)
    assert distance == 0.0


def test_calculate_distance_known_locations():
    """Test distance calculation between known locations."""
    # San Francisco to Los Angeles (approximately 560 km)
    sf_lat, sf_lon = 37.7749, -122.4194
    la_lat, la_lon = 34.0522, -118.2437
    
    distance = calculate_distance(sf_lat, sf_lon, la_lat, la_lon)
    
    # Should be approximately 560 km
    expected_distance = 560000  # meters
    assert abs(distance - expected_distance) < 50000  # Within 50km tolerance
