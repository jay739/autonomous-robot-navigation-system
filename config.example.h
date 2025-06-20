/*
 * Configuration file for Autonomous Robot Navigation System
 * 
 * IMPORTANT: Copy this file to config.h and replace the placeholder values below with your actual settings
 * This file contains all the configuration that needs to be customized.
 * 
 * Original config.h is in .gitignore to prevent committing sensitive data.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================
// Replace these with your actual WiFi credentials
#define WIFI_SSID "your_wifi_network_name"
#define WIFI_PASSWORD "your_wifi_password"

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================
// ESP32-WROVER IP address and port
// Note: You'll need to find your WROVER's IP address from the Serial Monitor
#define WROVER_IP_ADDRESS "192.168.197.10"
#define WROVER_PORT 8080

// ============================================================================
// CAMERA CONFIGURATION
// ============================================================================
// Camera calibration - adjust based on your camera height and resolution
#define PIXELS_TO_CM 0.1

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
// Motor speed settings
#define MIN_SPEED 150
#define MAX_SPEED 200

// Navigation thresholds
#define ANGLE_THRESHOLD_PRECISE 15.0
#define ANGLE_THRESHOLD_WIDE 45.0
#define TARGET_DISTANCE 10.0
#define DISTANCE_THRESHOLD 2.0

// ============================================================================
// SERVER CONFIGURATION
// ============================================================================
#define SERVER_PORT 8080

// ============================================================================
// DEBUG SETTINGS
// ============================================================================
#define DEBUG_MODE true  // Set to false to disable debug prints

#endif // CONFIG_H 