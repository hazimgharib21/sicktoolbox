/*!
 * \file SickNav350.hh
 * \brief Defines the SickNav350 class for working with the
 *        Sick NAV350.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_NAV350_HH
#define SICK_NAV350_HH

/* Macros */
#define DEFAULT_SICK_IP_ADDRESS                          "192.168.1.10"  ///< Default Sick LD INet 4 address
#define DEFAULT_SICK_TCP_PORT                                   (2111)  ///< Default TCP port
#define DEFAULT_SICK_MESSAGE_TIMEOUT                (unsigned int)(5e6)  ///< The max time to wait for a message reply (usecs)
#define DEFAULT_SICK_CONNECT_TIMEOUT                (unsigned int)(1e6)  ///< The max time to wait before considering a connection attempt as failed (usecs)
#define DEFAULT_SICK_NUM_SCAN_PROFILES                              (0)  ///< Setting this value to 0 will tell the Sick LD to stream measurements when measurement data is requested (NOTE: A profile is a single scans worth of range measurements)
#define DEFAULT_SICK_SIGNAL_SET                                     (0)  ///< Default Sick signal configuration

/**
 * \def SWAP_VALUES(x,y,t)
 * \brief A simple macro for swapping two values.
 */
#define SWAP_VALUES(x,y,t) (t=x,x=y,y=t);

/* Definition dependencies */
#include <string>
#include <vector>
#include <pthread.h>
#include <arpa/inet.h>
#include <stdlib.h>

#include "sicktoolbox/SickLIDAR.hh"
#include "sicktoolbox/SickNAV350BufferMonitor.hh"
#include "sicktoolbox/SickNAV350Message.hh"
#include "sicktoolbox/SickException.hh"
#define SICK_MAX_NUM_REFLECTORS 50

/**
 * \namespace SickToolbox
 * \brief Encapsulates the Sick NAV350 Matlab/C++ toolbox
 */
namespace SickToolbox {

  /**
   * \class SickNav350
   * \brief Provides a simple driver interface for working with the
   *        Sick Nav350 long-range models via Ethernet.
   */
struct sick_nav350_reflector_tag{
  unsigned int error;
	unsigned int filter;
	unsigned int landmarkDataFollow;
	unsigned int num_reflector;
	unsigned int cart[SICK_MAX_NUM_REFLECTORS];
	double x[SICK_MAX_NUM_REFLECTORS];
	double y[SICK_MAX_NUM_REFLECTORS];
	unsigned int polar[SICK_MAX_NUM_REFLECTORS];
	double dist[SICK_MAX_NUM_REFLECTORS];
	double phi[SICK_MAX_NUM_REFLECTORS];

	unsigned int optional[SICK_MAX_NUM_REFLECTORS];
	unsigned int LocalID[SICK_MAX_NUM_REFLECTORS];
	unsigned int GlobalID[SICK_MAX_NUM_REFLECTORS];
	unsigned int type[SICK_MAX_NUM_REFLECTORS];
	unsigned int subtype[SICK_MAX_NUM_REFLECTORS];
	unsigned int quality[SICK_MAX_NUM_REFLECTORS];
	unsigned int timestamp[SICK_MAX_NUM_REFLECTORS];
	unsigned int size [SICK_MAX_NUM_REFLECTORS];
	unsigned int hitCount[SICK_MAX_NUM_REFLECTORS];
	unsigned int meanEchoAmplitude[SICK_MAX_NUM_REFLECTORS];
	unsigned int indexStart[SICK_MAX_NUM_REFLECTORS];
	unsigned int indexEnd[SICK_MAX_NUM_REFLECTORS];
} ;

struct sick_nav350_pose_tag{
  unsigned int error;
	int x;
  int y;
	unsigned int phi;
	unsigned int optionalPoseData;
	unsigned int outputMode;
	unsigned int timeStamp;
	int meanDeviation;
	int positionMode;
	int infoState;
	int numUsedReflectors;
  int optionalLandmarkData;
};

class SickNav350 : public SickLIDAR< SickNav350BufferMonitor, SickNav350Message > {

  public:

    /* Some constants for the developer/end-user */
    static const uint16_t SICK_MAX_NUM_MEASUREMENTS = 2881;                             ///< Maximum number of measurements per sector
    static const uint16_t SICK_MAX_NUM_SECTORS = 8;                                     ///< Maximum number of scan sectors (NOTE: This value must be even)
    static const uint16_t SICK_MAX_NUM_MEASURING_SECTORS = 4;                           ///< Maximum number of active/measuring scan sectors
    static const uint16_t SICK_MAX_SCAN_AREA = 360;                                     ///< Maximum area that can be covered in a single scan (deg)
    static const uint16_t SICK_MIN_MOTOR_SPEED = 8;                                     ///< Minimum motor speed in Hz
    static const uint16_t SICK_MAX_MOTOR_SPEED = 8;                                    ///< Maximum motor speed in Hz
    static const uint16_t SICK_MIN_VALID_SENSOR_ID = 1;                                 ///< The lowest value the Sick will accept as a Sensor ID
    static const uint16_t SICK_MAX_VALID_SENSOR_ID = 254;                               ///< The largest value the Sick will accept as a Sensor ID
    static const uint16_t SICK_MAX_MEAN_PULSE_FREQUENCY = 10800;                        ///< Max mean pulse frequence of the current device configuration (in Hz) (see page 22 of the operator's manual)
    static const uint16_t SICK_MAX_PULSE_FREQUENCY = 14400;                             ///< Max pulse frequency of the device (in Hz) (see page 22 of the operator's manual)
    static const uint16_t SICK_NUM_TICKS_PER_MOTOR_REV = 5760;                          ///< Odometer ticks per revolution of the Sick LD scan head
    static const double SICK_MAX_SCAN_ANGULAR_RESOLUTION = 0.125;                       ///< Minimum valid separation between laser pulses in active scan ares (deg)
    static const double SICK_DEGREES_PER_MOTOR_STEP = 0.0625;                           ///< Each odometer tick is equivalent to rotating the scan head this many degrees

    /* Sick NAV350 sensor modes of operation */
    static const uint8_t SICK_SENSOR_MODE_POWERDOWN = 0x00;                                  ///< The Sick LD is powered down
    static const uint8_t SICK_SENSOR_MODE_STANDBY = 0x01;                                ///< The Sick NAV350 is in standby
    static const uint8_t SICK_SENSOR_MODE_MAPPING = 0x02;                               ///< The Sick NAV350 is mapping
    static const uint8_t SICK_SENSOR_MODE_LMDETECTION = 0x03;                                 ///< The Sick NAV350 is detecting landmarks
    static const uint8_t SICK_SENSOR_MODE_NAVIGATION = 0x04;                               ///< The Sick Nav350 is in navigation mode

    /* COMMAND TYPE */
    static const std::string READBYNAME_COMMAND;
    static const std::string WRITEBYNAME_COMMAND;
    static const std::string METHODCALL_COMMAND;

    /* COMMAND */
    static const std::string DEVICEIDENT_COMMAND;
    static const std::string SERIALNUMBER_COMMAND;
    static const std::string DEVICEINFO_COMMAND;
    static const std::string FIRMWAREVERSION_COMMAND;
    static const std::string CURLAYER_COMMAND;
    static const std::string IDENTWINDOW_COMMAND;
    static const std::string CFGMAPPING_COMMAND;
    static const std::string SLIDINGMEAN_COMMAND;
    static const std::string POSDATAFORMAT_COMMAND;
    static const std::string LMDATAFORMAT_COMMAND;
    static const std::string SCANDATAFORMAT_COMMAND;
    static const std::string HWTIMESYNC_COMMAND;
    static const std::string REFLECTORSIZE_COMMAND;
    static const std::string REFLECTORTYPE_COMMAND;
    static const std::string LMMATCHING_COMMAND;
    static const std::string SECTORMUTING_COMMAND;
    static const std::string COORDORIENTATION_COMMAND;
    static const std::string CLOSESTREFL_COMMAND;
    static const std::string ACTIONRADIUS_COMMAND;
    static const std::string REFLTHRESHOLD_COMMAND;
    static const std::string SETMODE_COMMAND;
    static const std::string SETACCESSMODE_COMMAND;
    static const std::string SETPERMDATA_COMMAND;
    static const std::string SYNCTIMESTAMP_COMMAND;
    static const std::string NAVBREAK_COMMAND;
    static const std::string NAVRESET_COMMAND;
    static const std::string CFGSERIAL_COMMAND;
    static const std::string CFGIP_COMMAND;
    static const std::string CFGETH_COMMAND;
    static const std::string ENABLEDHCP_COMMAND;
    static const std::string ADDLANDMARK_COMMAND;
    static const std::string EDITLANDMARK_COMMAND;
    static const std::string DELETELANDMARK_COMMAND;
    static const std::string READLANDMARK_COMMAND;
    static const std::string READLAYER_COMMAND;
    static const std::string READLAYOUT_COMMAND;
    static const std::string ERASELAYOUT_COMMAND;
    static const std::string SAVELAYOUT_COMMAND;
    static const std::string DOMAPPING_COMMAND;
    static const std::string GETLANDMARK_COMMAND;
    static const std::string POSEREQ_COMMAND;
    static const std::string POSEDATA_COMMAND;
    static const std::string SETSPEED_COMMAND;
    static const std::string SETPOSE_COMMAND;
    static const std::string SETPOSEID_COMMAND;

    /**
     * \struct sick_nav350_config_global_tag
     * \brief A structure to aggregate the data used to configure the
     *        Sick NAV350 global parameter values.
     */
    /**
     * \typedef sick_nav350_config_global_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_nav350_config_global_tag {
      uint16_t sick_sensor_id;                                                            ///< The single word sensor ID for the Sick unit
      uint16_t sick_motor_speed;                                                          ///< Nominal motor speed value: 0x0005 to 0x0014 (5 to 20)
      double sick_angle_step;                                                             ///< Difference between two laser pulse positions in 1/16th deg. (NOTE: this value must be a divisor of 5760 and be greater than 1)
    } sick_nav350_config_global_t;

    /**
     * \struct sick_nav350_config_ethernet_tag
     * \brief A structure to aggregate the data used to configure
     *        the Sick LD unit for Ethernet.
     *
     * \todo Eventually add similar config structures for the other protocols.
     */
    /**
     * \typedef sick_nav350_config_ethernet_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_nav350_config_ethernet_tag {
      uint16_t sick_ip_address[4];                                                        ///< IP address in numerical form w/ leftmost part at sick_ip_address[0]
      uint16_t sick_subnet_mask[4];                                                       ///< Subnet mask for the network to which the Sick LD is assigned
      uint16_t sick_gateway_ip_address[4];                                                ///< The address of the local gateway
      uint16_t sick_node_id;                                                              ///< Single word address of the Sick LD
      uint16_t sick_transparent_tcp_port;                                                 ///< The TCP/IP transparent port associated with the Sick LD
    } sick_nav350_config_ethernet_t;

    /**
     * \struct sick_nav350_config_sector_tag
     * \brief A structure to aggregate data used to define the
     *        Sick LD's sector configuration.
     */
    /**
     * \typedef sick_nav350_config_sector_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_nav350_config_sector_tag {
      uint8_t sick_num_active_sectors;                                                    ///< Number of active sectors (sectors that are actually being scanned)
      uint8_t sick_num_initialized_sectors;                                               ///< Number of sectors configured w/ a function other than "not initialized"
      uint8_t sick_active_sector_ids[SICK_MAX_NUM_SECTORS];                               ///< IDs of all active sectors
      uint8_t sick_sector_functions[SICK_MAX_NUM_SECTORS];                                ///< Function values associated w/ each of the Sick LD's sectors
      double sick_sector_start_angles[SICK_MAX_NUM_SECTORS];                              ///< Start angles for each initialized sector (deg)
      double sick_sector_stop_angles[SICK_MAX_NUM_SECTORS];                               ///< Stop angles for each sector (deg)
    } sick_nav350_config_sector_t;

    /**
     * \struct sick_nav350_identity_tag
     * \brief A structure to aggregate the fields that collectively
     *        define the identity of a Sick LD unit.
     */
    /**
     * \typedef sick_nav350_identity_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_nav350_identity_tag {
      std::string sick_part_number;                                                       ///< The Sick LD's part number
      std::string sick_name;                                                              ///< The name assigned to the Sick
      std::string sick_version;                                                           ///< The Sick LD's version number
      std::string sick_serial_number;                                                     ///< The Sick LD's serial number
      std::string sick_edm_serial_number;                                                 ///< The Sick LD's edm??? serial number
      std::string sick_firmware_part_number;                                              ///< The Sick LD's firmware part number
      std::string sick_firmware_name;                                                     ///< The Sick LD's firmware name
      std::string sick_firmware_version;                                                  ///< The Sick LD's firmware version
      std::string sick_application_software_part_number;                                  ///< The Sick LD's app. software part number
      std::string sick_application_software_name;                                         ///< The Sick LD's app. software name
      std::string sick_application_software_version;                                      ///< The Sick LD's app. software version
    } sick_nav350_identity_t;

    sick_nav350_reflector_tag ReflectorData_;
    sick_nav350_pose_tag PoseData_;
    /**
     * \struct sick_nav350_sector_data_tag
     * \brief A structure to aggregate the fields that collectively
     *        define a sector in the scan area of the Sick LD unit.
     */
    /**
     * \typedef sick_nav350_sector_data_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_nav350_sector_data_tag {
      unsigned int sector_num;                                                            ///< The sector number in the scan area
      unsigned int num_data_points;                                                       ///< The number of data points in the scan area
      unsigned int timestamp_start;                                                       ///< The timestamp (in ms) corresponding to the time the first measurement in the sector was taken
      unsigned int timestamp_stop;                                                        ///< The timestamp (in ms) corresponding to the time the last measurement in the sector was taken
      unsigned int echo_values[SICK_MAX_NUM_MEASUREMENTS];                                ///< The corresponding echo/reflectivity values
      double angle_step;                                                                  ///< The angle step used for the given sector (this should be the same for all sectors)
      double angle_start;                                                                 ///< The angle at which the first measurement in the sector was acquired
      double angle_stop;                                                                  ///< The angle at which the last measurement in the sector was acquired
      double range_values[SICK_MAX_NUM_MEASUREMENTS];                                     ///< The corresponding range values (NOTE: The size of this array is intended to be large enough to accomodate various sector configs.)
      double scan_angles[SICK_MAX_NUM_MEASUREMENTS];                                      ///< The scan angles corresponding to the respective measurements
    } sick_nav350_sector_data_t;

    /**
     * \struct sick_nav350_scan_profile_tag
     * \brief A structure to aggregate the fields that collectively
     *        define the profile of a single scan acquired from the
     *        Sick LD unit.
     */
    /**
     * \typedef sick_nav350_scan_profile_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_nav350_scan_profile_tag {
      unsigned int profile_number;                                                        ///< The number of profiles sent to the host (i.e. the current profile number)
      unsigned int profile_counter;                                                       ///< The number of profiles gathered by the Sick LD
      unsigned int layer_num;                                                             ///< The layer number associated with a scan (this will always be 0)
      unsigned int sensor_status;                                                         ///< The status of the Sick LD sensor
      unsigned int motor_status;                                                          ///< The status of the Sick LD motor
      unsigned int num_sectors;                                                           ///< The number of sectors returned in the profile
      sick_nav350_sector_data_t sector_data[SICK_MAX_NUM_SECTORS];                            ///< The sectors associated with the scan profile
    } sick_nav350_scan_profile_t;

    sick_nav350_sector_data_tag* MeasuredData_;
    /** Primary constructor */
    SickNav350( const std::string sick_ip_address = DEFAULT_SICK_IP_ADDRESS,
	    const uint16_t sick_tcp_port = DEFAULT_SICK_TCP_PORT );

    /** Initializes the Sick LD unit (use scan areas defined in flash) */
    void Initialize( )  throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException );

    /** Initializes the Sick LD unit (use scan areas defined in flash) */
    void Uninitialize( ) throw (SickIOException, SickThreadException, SickTimeoutException, SickErrorException);

    /** Gets the sensor and motor mode of the unit */
    void GetSickStatus( unsigned int &sick_sensor_mode, unsigned int &sick_motor_mode )
      throw( SickIOException, SickTimeoutException );

    /** Sets the temporal scan configuration (until power is cycled) */
    void SetSickTempScanAreas( const double * active_sector_start_angles, const double * const active_sector_stop_angles,
			       const unsigned int num_active_sectors )
      throw( SickTimeoutException, SickIOException, SickConfigException );

    /** Sets the internal clock of the Sick LD unit */
    void SetSickTimeAbsolute( const uint16_t absolute_clock_time, uint16_t &new_sick_clock_time )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Sets the internal clock of the Sick LD using the relative given time value */
    void SetSickTimeRelative( const int16_t time_delta, uint16_t &new_sick_clock_time )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Gets the internal clock time of the Sick LD unit */
    void GetSickTime( uint16_t &sick_time )
      throw( SickIOException, SickTimeoutException, SickErrorException );

    /** Sets the scan data format to be used (until power is cycled). This is defined on Page 26 of NAV350 Telegram Listing **/
    void SetScanDataFormat (uint8_t dataMode, uint8_t showRSSI)
      throw( SickIOException, SickTimeoutException, SickErrorException );

    /** Sets the access mode */
    void SetAccessMode( uint8_t newMode)
      throw(SickIOException, SickTimeoutException, SickErrorException);

    /** Acquire the Sick LD's current scan resolution */
    double GetSickScanResolution( ) const;

    /** Acquire the current IP address of the Sick */
    std::string GetSickIPAddress( ) const;

    /** Acquire the subnet mask for the Sick */
    std::string GetSickSubnetMask( ) const;

    /** Acquire the IP address of the Sick gateway */
    std::string GetSickGatewayIPAddress( ) const;

    /** Acquire the Sick LD's part number */
    std::string GetSickPartNumber( ) const;

    /** Acquire the Sick LD's name */
    std::string GetSickName( ) const;

    /** Acquire the Sick LD's version number */
    std::string GetSickVersion( ) const;

    std::string GetSickSerialNumber( ) const;
    std::string GetSickFirmwareVersion( ) const;
    std::string GetSickSoftwareVersion( ) const;

    void SetCurrentLayer(uint16_t layer);
    std::string GetCurrentLayer();
    void SetReflectorWindow(uint16_t winLow, uint16_t winHigh, uint32_t distLow, uint32_t distHigh);
    std::string GetReflectorWindow();
    void SetMappingConfiguration(uint8_t mean, uint8_t negative, int x, int y, int phi);
    std::string GetMappingConfiguration();
    void SetSlidingMean(uint8_t mean);
    std::string GetSlidingMean();
    void SetPoseDataFormat(uint8_t outputMode, uint8_t showOptParam);
    std::string GetPoseDataFormat();
    void SetLandmarkDataFormat(int format, int showOptParam, int landmarkFilter);
    std::string GetLandmarkDataFormat();
    std::string GetScanDataFormat();
    void SetTimeSync(uint8_t mode, uint8_t mask);
    std::string GetTimeSync();
    void SetReflectorSize(uint16_t size);
    std::string GetReflectorSize();
    void SetReflectorType(uint8_t type);
    std::string GetReflectorType();
    void SetLandmarkMatching(uint8_t filter);
    std::string GetLandmarkMatching();
    void SetSectorMuting(uint32_t angleFrom_0, uint32_t angleTo_0, bool isActive_0,uint32_t angleFrom_1, uint32_t angleTo_1, bool isActive_1,uint32_t angleFrom_2, uint32_t angleTo_2, bool isActive_2,uint32_t angleFrom_3, uint32_t angleTo_3, bool isActive_3);
    std::string GetMutedSectors();
    void SetCoordinateOrientation(uint8_t dir);
    std::string GetCoordinateOrientation();
    void SetNClosestReflectors(uint8_t num);
    std::string GetNClosestReflectors();
    void SetActionRadius(int min, int max);
    std::string GetActionRadius();
    void SetReflectorThreshold(int percent);
    std::string GetReflectorThreshold();
    void SetDataPermenant();
    void SyncTimeStamp();
    void BreakAsyncCall();
    void ResetDevice();
    void SetSerialConfig(uint8_t baudrate, uint8_t dataBits, uint8_t parity, uint8_t stopBits);
    void SetIPConfig(uint8_t ipAddress, uint8_t subnetMask, uint8_t gateway);
    void SetEthConfig(uint8_t speedDuplex);
    void EnableDHCP(bool isEnable);
    void AddLandmark(uint16_t num, int data[][7]);
    void EditLandmark(uint16_t num, uint16_t id, int x, int y, uint8_t lmType, uint8_t reflectorType, uint16_t size, uint16_t layer, uint16_t layerID);
    void DeleteLandmark(uint16_t num, int id[]);
    void GetLandmark(uint16_t num, uint16_t id);
    void GetLayer(uint16_t id);
    void GetLayout();
    void EraseLayout(uint8_t mem);
    void SaveLayout();
    void DoMapping();
    void GetLandmarkData(bool useNewLandmark, uint8_t dataFormat);
    void GetPose(bool wait);
    void GetPoseNScan(bool wait, uint8_t data);
    void SetSpeed(double x, double y, double phi, int timestamp, int coordbase);
    void SetPose(double x, double y, double phi);
    void SetPoseID(uint16_t id);

    /** Get Sick Identity */
    void GetSickIdentity();
    /** Change to navigation mode */
    void SetOperatingMode(int mode);

    /**Get data */
    void GetPoseData(int wait,int dataset);

    void GetDataLandMark(int wait,int dataset);

    void GetDataNavigation(int wait,int dataset);

    /**Get Measurements*/
    void GetSickMeasurements(double* range_values,unsigned int *num_measurements,
    		double *sector_step_angle,
    		double *sector_start_angle,
    		double *sector_stop_angle,
    		unsigned int *sector_start_timestamp,
    		unsigned int *sector_stop_timestamp);

    void GetSickMeasurementsWithRemission(double* range_values, unsigned int *remission_values,unsigned int *num_measurements,
    		    double *sector_step_angle,
        		double *sector_start_angle,
        		double *sector_stop_angle,
        		unsigned int *sector_start_timestamp,
        		unsigned int *sector_stop_timestamp);

    /**Send custom message and get response*/
    void GetResponseFromCustomMessage(uint8_t *req,int req_size,uint8_t *res,int *res_size);



    /** Destructor */
    ~SickNav350();

  private:
	  std::string* arg;
	  int argumentcount_;

    /** The Sick LD IP address */
    std::string _sick_ip_address;

    /** The Sick LD TCP port number */
    uint16_t _sick_tcp_port;

 //   /** Sick LD socket structure */
//    unsigned int _socket;

    /** Sick LD socket address structure */
    struct sockaddr_in _sick_inet_address_info;

    /** Indicates whether the Sick LD is currently streaming range data */
    bool _sick_streaming_range_data;

    /** Indicates whether the Sick LD is currently streaming range and echo data */
    bool _sick_streaming_range_and_echo_data;

    /** The identity structure for the Sick */
    sick_nav350_identity_t _sick_identity;

    /** The current global configuration for the unit */
    sick_nav350_config_global_t _sick_global_config;

    /** The current Ethernet configuration for the unit */
    sick_nav350_config_ethernet_t _sick_ethernet_config;

    /** The current sector configuration for the unit */
    sick_nav350_config_sector_t _sick_sector_config;

    /** Setup the connection parameters and establish TCP connection! */
    void _setupConnection( ) throw( SickIOException, SickTimeoutException );

    /** Synchronizes the driver state with the Sick LD (used for initialization) */
    void _syncDriverWithSick( ) throw( SickIOException, SickTimeoutException, SickErrorException );

    /** Set the function for a particular scan secto */
    void _setSickSectorFunction( const uint8_t sector_number, const uint8_t sector_function,
				 const double sector_angle_stop, const bool write_to_flash = false )
       throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Acquires the given Sector's function (i.e. current config) */
    void _getSickSectorFunction( const uint8_t sector_num, uint8_t &sector_function, double &sector_stop_angle )
      throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Sets the Sick LD to IDLE mode */
    void _setSickSensorModeToIdle( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Sets the Sick LD to ROTATE mode */
    void _setSickSensorModeToRotate( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Sets the Sick LD to MEASURE mode */
    void _setSickSensorModeToMeasure( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Sets the Sick LD's sensor mode to IDLE (laser off, motor off) */
    void _setSickSensorMode( const uint8_t new_sick_sensor_mode )
      throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Requests n range measurement profiles from the Sick LD */
    void _getSickScanProfiles( const uint16_t profile_format, const uint16_t num_profiles = DEFAULT_SICK_NUM_SCAN_PROFILES )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Parses a sequence of bytes and populates the profile_data struct w/ the results */
    void _parseScanProfile( uint8_t * const src_buffer, sick_nav350_scan_profile_t &profile_data ) const;

    /** Cancels the active data stream */
    void _cancelSickScanProfiles( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Turns nearfield suppression on/off */
    void _setSickFilter( const uint8_t suppress_code )
      throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Stores an image of the Sick LD's identity locally */
     void _getSickIdentity( ) /*throw( SickTimeoutException, SickIOException )*/;

     void _getSickSerialNumber();
     void _getSickFirmwareVersion();
     void _getSickSoftwareVersion();

    /** Query the Sick for its sensor and motor status */
    void _getSickStatus( ) throw( SickTimeoutException, SickIOException );

    /** Sets the Sick LD's global configuration (in flash) */
    void _setSickGlobalConfig( const uint8_t sick_sensor_id, const uint8_t sick_motor_speed, const double sick_angle_step )
      throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Query the Sick for its global configuration parameters */
    void _getSickGlobalConfig( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Query the Sick for its Ethernet configuration parameters */
    void _getSickEthernetConfig( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Acquires the configuration (function and stop angle) for each sector */
    void _getSickSectorConfig( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Query the Sick for ID information */
    void _getIdentificationString( const uint8_t id_request_code, std::string &id_return_string )
      throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its sensor part number */
    void _getSensorPartNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its assigned name */
    void _getSensorName( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its version number */
    void _getSensorVersion( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its serial number */
    void _getSensorSerialNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its EDM unit's serial number */
    void _getSensorEDMSerialNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the part number of its firmware */
    void _getFirmwarePartNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the name of its firmware */
    void _getFirmwareName( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the version of the firmware */
    void _getFirmwareVersion( ) throw( SickTimeoutException, SickIOException );

    /** Query the part number of the application software */
    void _getApplicationSoftwarePartNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the application name */
    void _getApplicationSoftwareName( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the application software version */
    void _getApplicationSoftwareVersion( ) throw( SickTimeoutException, SickIOException );

    /** Allows setting the global parameters and scan area definition (in flash) */
    void _setSickGlobalParamsAndScanAreas( const unsigned int sick_motor_speed, const double sick_step_angle,
					   const double * const active_sector_start_angles,
					   const double * const active_sector_stop_angles,
					   const unsigned int num_active_sectors )
       throw( SickTimeoutException, SickIOException, SickConfigException, SickErrorException );

    /** Allows setting a temporary (until a device reset) sector configuration on the device */
    void _setSickTemporaryScanAreas( const double * const active_sector_start_angles,
				     const double * const active_sector_stop_angles,
				     const unsigned int num_active_sectors )
      throw( SickTimeoutException, SickIOException, SickConfigException );

    /** Sets the sick sector configuration */
    void _setSickSectorConfig( const unsigned int * const sector_functions, const double * const sector_stop_angles,
			       const unsigned int num_sectors, const bool write_to_flash = false )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Sets the signals for the device */
    void _setSickSignals( const uint8_t sick_signal_flags = DEFAULT_SICK_SIGNAL_SET )
      throw( SickIOException, SickTimeoutException, SickErrorException );

    /** Flushed the TCP receive buffer */
    void _flushTCPRecvBuffer( ) throw ( SickIOException, SickThreadException );

    /** Generates a device-ready sector set given only an active sector spec. */
    void _generateSickSectorConfig( const double * const active_sector_start_angles,
				    const double * const active_sector_stop_angles,
				    const unsigned int num_active_sectors,
				    const double sick_step_angle,
				    unsigned int * const sector_functions,
				    double * const sector_stop_angles,
				    unsigned int &num_sectors ) const;

    /** Converts odometry ticks to an equivalent angle */
    double _ticksToAngle( const uint16_t ticks ) const;

    /** Converts angle to an equivalent representation in odometer ticks */
    uint16_t _angleToTicks( const double angle ) const;

    /** Computes the mean pulse frequency for the given config */
    double _computeMeanPulseFrequency( const double active_scan_area, const double curr_motor_speed,
				       const double curr_angular_resolution ) const;

    /** Computes the total pulse frequency for the given config */
    double _computeMaxPulseFrequency( const double total_scan_area, const double curr_motor_speed,
				      const double curr_angular_resolution ) const;

    /** Indicates whether a given sensor ID is valid for the device */
    bool _validSickSensorID( const unsigned int sick_sensor_id ) const;

    /** Indicates whether a given motor speed is valid for the device */
    bool _validSickMotorSpeed( const unsigned int sick_motor_speed ) const;

    /** Indicates whether a given motor speed is valid for the device */
    bool _validSickScanResolution( const double sick_step_angle, const double * const active_sector_start_angles,
				   const double * const active_sector_stop_angles, const unsigned int num_active_sectors ) const;

    /** Indicates whether the given configuration yields a valid max and mean pulse frequency */
     bool _validPulseFrequency( const unsigned int sick_motor_speed, const double sick_step_angle ) const;

    /** Indicates whether the given configuration yields a valid max and mean pulse frequency */
   bool _validPulseFrequency( const unsigned int sick_motor_speed, const double sick_step_angle,
			       const double * const active_sector_start_angles,
			       const double * const active_sector_stop_angles,
			       const unsigned int num_active_sectors ) const;

    /** Returns the scanning area for the device given the current sector configuration */
    double _computeScanArea( const double sick_step_angle, const double * const sector_start_angles,
			     const double * const sector_stop_angles, const unsigned int num_sectors ) const;

    /** Reorders given sector angle sets */
    void _sortScanAreas( double * const sector_start_angles, double * const sector_stop_angles,
			 const unsigned int num_sectors ) const;

    /** Checks the given sector arguments for overlapping regions yielding an invalid configuration */
    bool _validActiveSectors( const double * const sector_start_angles, const double * const sector_stop_angles,
			      const unsigned int num_active_sectors ) const;

    /** Indicates whether the supplied profile format is currently supported by the driver */
    bool _supportedScanProfileFormat( const uint16_t profile_format ) const;

    /** Prints data corresponding to a single scan sector (data obtained using GET_PROFILE) */
    void _printSectorProfileData( const sick_nav350_sector_data_t &sector_data ) const;

    /** Prints the data corresponding to the given scan profile (for debugging purposes) */
    void _printSickScanProfile( const sick_nav350_scan_profile_t profile_data, const bool print_sector_data = true ) const;

    /** Converts _sick_sensor_mode to a representative string */
    std::string _sickSensorModeToString( const uint8_t sick_sensor_mode ) const;

    /** Converts _sick_motor_mode to a representative string */
    std::string _sickMotorModeToString( const uint8_t sick_motor_mode ) const;

    /** Converts the specified trans measurement mode return value to a string */
    std::string _sickTransMeasureReturnToString( const uint8_t return_value ) const;

    /** Converts the specified reset level to a representative string */
    std::string _sickResetLevelToString( const uint16_t reset_level ) const;

    /** Converts Sick LD sector configuration word to a representative string */
    std::string _sickSectorFunctionToString( const uint16_t sick_sector_function ) const;

    /** Converts a given scan profile format to a string for friendlier output */
    std::string _sickProfileFormatToString( const uint16_t profile_format ) const;

    /** Teardown the connection to the Sick LD */
    void _teardownConnection( ) throw( SickIOException );

    /** Send a message, get the reply from the Sick Nav350 and check it */
    void _sendMessageAndGetReply( const SickNav350Message &send_message, SickNav350Message &recv_message,				  const unsigned int timeout_value = DEFAULT_SICK_MESSAGE_TIMEOUT )
    throw( SickIOException, SickTimeoutException );

    /** Split message by space symbol*/
    void _SplitReceivedMessage(SickNav350Message recv_message);

    /** Parse data gotten by GetScanData*/
    void _ParseScanData();

    void _ParseScanDataLandMark();

    void _ParseScanDataNavigation();

    /**Convert Hex to number*/
    int _ConvertHexToDec(std::string num);

  };


} //namespace SickToolbox

#endif /* SICK_NAV350_HH */
