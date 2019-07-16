/*!
 * \file SickNav350.cc
 * \brief Implements the SickNav350 driver class.
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

/* Auto-generated header */
#include "sicktoolbox/SickConfig.hh"

/* Implementation dependencies */
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <sstream>            // for converting numerical values to strings
#include <sys/socket.h>       // for socket function definitions
#include <arpa/inet.h>        // for sockaddr_in, inet_addr, and htons
#include <sys/ioctl.h>        // for using ioctl functionality for the socket input buffer
#include <unistd.h>           // for select functionality (e.g. FD_SET, etc...)
#include <sys/types.h>        // for fd data types
#include <sys/time.h>         // for select timeout parameter
#include <fcntl.h>            // for getting file flags
#include <pthread.h>          // for POSIX threads
#include <sstream>            // for parsing ip addresses
#include <vector>             // for returning the results of parsed strings
#include <errno.h>            // for timing connect()
#include <stdio.h>

#include "sicktoolbox/SickNAV350.hh"
#include "sicktoolbox/SickNAV350Message.hh"
#include "sicktoolbox/SickNAV350BufferMonitor.hh"
#include "sicktoolbox/SickNAV350Utility.hh"
 #include "sicktoolbox/SickException.hh"
using namespace std;
/* Associate the namespace */
namespace SickToolbox {

const std::string SickNav350::READBYNAME_COMMAND="sRN";
const std::string SickNav350::WRITEBYNAME_COMMAND="sWN";
const std::string SickNav350::METHODCALL_COMMAND="sMN";

// Command Accessible from all modes

// Read Device Identification
const std::string SickNav350::DEVICEIDENT_COMMAND="DeviceIdent";

// Read Serial Number
const std::string SickNav350::SERIALNUMBER_COMMAND="SerialNumber";

// Read Measurement Firmware Version
const std::string SickNav350::DEVICEINFO_COMMAND="MMDeviceInfo";

// Read Application Version
const std::string SickNav350::FIRMWAREVERSION_COMMAND="FirmwareVersion";

// Set Current Layer
const std::string SickNav350::CURLAYER_COMMAND="NEVACurrLayer";

// Define Reflector Identification Window
const std::string SickNav350::IDENTWINDOW_COMMAND="NCORIdentWindow";

// Configure Mapping
const std::string SickNav350::CFGMAPPING_COMMAND="NMAPMapCfg";

// Set Sliding Mean
const std::string SickNav350::SLIDINGMEAN_COMMAND="NPOSSlidingMean";

// Set Positioning Data Format
const std::string SickNav350::POSDATAFORMAT_COMMAND="NPOSPoseDataFormat";

// Set Landmark Data Format
const std::string SickNav350::LMDATAFORMAT_COMMAND="NLMDLandmarkDataFormat";

// Set Scan Data Format
const std::string SickNav350::SCANDATAFORMAT_COMMAND="NAVScanDataFormat";

// Set Hardware Time Sync
const std::string SickNav350::HWTIMESYNC_COMMAND="NAVHardwareTimeSync";

// Set Reflector Size
const std::string SickNav350::REFLECTORSIZE_COMMAND="NLMDReflSize";

// Set Reflector Type
const std::string SickNav350::REFLECTORTYPE_COMMAND="NLMDReflType";

// Set Landmark Matching
const std::string SickNav350::LMMATCHING_COMMAND="NLMDLandmarkMatching";

// Set Sector Muting
const std::string SickNav350::SECTORMUTING_COMMAND="NLMDMutedSectors";

// Set Orientation Coordinate System
const std::string SickNav350::COORDORIENTATION_COMMAND="NEVACordOrientation";

// Set N Closest Reflectors
const std::string SickNav350::CLOSESTREFL_COMMAND="NLMDnClosest";

// Set Action Radius
const std::string SickNav350::ACTIONRADIUS_COMMAND="NLMDActionRadius";

// Set Reflector Threshold
const std::string SickNav350::REFLTHRESHOLD_COMMAND="NLMDReflThreshold";

// Methods Accessible from all modes

// Set Operating Mode
const std::string SickNav350::SETMODE_COMMAND="mNEVAChangeState";

// Set User Level
const std::string SickNav350::SETACCESSMODE_COMMAND="SetAccessMode";

// Store Data Permanent
const std::string SickNav350::SETPERMDATA_COMMAND="mEEwriteall";

// Synchronize Timestamp
const std::string SickNav350::SYNCTIMESTAMP_COMMAND="mNAVGetTimestamp";

// Break Asynchrounous Method
const std::string SickNav350::NAVBREAK_COMMAND="mNAVBreak";

// Device Reset
const std::string SickNav350::NAVRESET_COMMAND="mNAVReset";

// Methods in STANDBY Mode

// Serial Interface Configuration
const std::string SickNav350::CFGSERIAL_COMMAND="mChangeSerialCfg";

// Change IP Configuration
const std::string SickNav350::CFGIP_COMMAND="mChangeIPCfg";

// Change Ethernet Configuration
const std::string SickNav350::CFGETH_COMMAND="mChangeEthCfg";

// DHCP Enabled / Disabled
const std::string SickNav350::ENABLEDHCP_COMMAND="mEnableDHCP";

// Add Landmark
const std::string SickNav350::ADDLANDMARK_COMMAND="mNLAYAddLandmark";

// Edit Landmark
const std::string SickNav350::EDITLANDMARK_COMMAND="mNLAYSetLandmark";

// Delete Landmark
const std::string SickNav350::DELETELANDMARK_COMMAND="mNLAYDelLandmark";

// Read Landmark
const std::string SickNav350::READLANDMARK_COMMAND="mNLAYGetLandmark";

// Read Layer
const std::string SickNav350::READLAYER_COMMAND="mNLAYGetLayer";

// Read Layout
const std::string SickNav350::READLAYOUT_COMMAND="mNLAYGetLayout";

// Erase Layout
const std::string SickNav350::ERASELAYOUT_COMMAND="mNLAYEraseLayout";

// Store Layout Permanent
const std::string SickNav350::SAVELAYOUT_COMMAND="mNLAYStoreLayout";

// Methods in MAPPING Mode

// Do Mapping
const std::string SickNav350::DOMAPPING_COMMAND="mNMAPDoMapping";

// Methods in LANDMARK Mode

// Get Landmark Data
const std::string SickNav350::GETLANDMARK_COMMAND="mNLMDGetData";

// Methods in NAVIGATION Mode

// Position Request
const std::string SickNav350::POSEREQ_COMMAND="mNPOSGetPose";

// Position Request + Scan
const std::string SickNav350::POSEDATA_COMMAND="mNPOSGetData";

// Velocity Input
const std::string SickNav350::SETSPEED_COMMAND="mNPOSSetSpeed";

// Set Current Position
const std::string SickNav350::SETPOSE_COMMAND="mNPOSSetPose";

// Set Current Position by Landmark ID
const std::string SickNav350::SETPOSEID_COMMAND="mNPOSSetPoseID";


 /**
   * \brief A standard constructor
   * \param sick_ip_address The ip address of the Sick Nav350
   * \param sick_tcp_port The TCP port associated w/ the Sick Nav350 server
   */
  SickNav350::SickNav350( const std::string sick_ip_address, const uint16_t sick_tcp_port ) :
    SickLIDAR< SickNav350BufferMonitor, SickNav350Message >( ),
    _sick_ip_address(sick_ip_address),
    _sick_tcp_port(sick_tcp_port),
    _sick_streaming_range_data(false),
    _sick_streaming_range_and_echo_data(false)
  {
	  arg=new std::string[5000];
	  argumentcount_=0;
	  MeasuredData_=new sick_nav350_sector_data_tag;
	  /* Initialize the global configuration structure */
  }

  /**
   * A standard destructor
   */
  SickNav350::~SickNav350( ) { }

  /**
   * \brief Initializes the driver and syncs it with Sick Nav350 unit. Uses sector config given in flash.
   */
  void SickNav350::Initialize( ) throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException ) {

    std::cout << "\t*** Attempting to initialize the Sick Nav350..." << std::endl;

    try {

      /* Attempt to connect to the Sick Nav350 */
      std::cout << "\tAttempting to connect to Sick Nav350 @ " << _sick_ip_address << ":" << _sick_tcp_port << std::endl;
      _setupConnection();
      std::cout << "\t\tConnected to Sick Nav350!" << std::endl;

      /* Start the buffer monitor */
      std::cout << "\tAttempting to start buffer monitor..." << std::endl;
      _startListening();
      std::cout << "\t\tBuffer monitor started!" << std::endl;


    }

    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    catch(...) {
      std::cerr << "SickNav350::Initialize - Unknown exception!" << std::endl;
      throw;
    }

    std::cout << "\t\tSynchronized!" << std::endl;

    _sick_initialized = true;


    /* Success */
  }

  void SickNav350::Uninitialize( )
  {
	  delete []arg;
	  delete MeasuredData_;
  }

  std::string SickNav350::GetSickName() const {
	  return _sick_identity.sick_name;
  }

  std::string SickNav350::GetSickVersion( ) const {
    return _sick_identity.sick_version;
  }

  std::string SickNav350::GetSickSerialNumber( ) const {
    return _sick_identity.sick_serial_number;
  }

  std::string SickNav350::GetSickFirmwareVersion( ) const {
    return _sick_identity.sick_firmware_version;
  }

  std::string SickNav350::GetSickSoftwareVersion( ) const {
    return _sick_identity.sick_application_software_version;
  }

  /**
   * \brief Acquire the current IP address of the Sick
   * \return The Sick Nav350 IP (Inet4) address
   */
  std::string SickNav350::GetSickIPAddress( ) const {

    /* Declare the string stream */
	std::cout << "Getting Sick Ip address..." << std::endl;
	std::cout << _sick_ethernet_config.sick_ip_address[0] << "."
	       << _sick_ethernet_config.sick_ip_address[1] << "."
	       << _sick_ethernet_config.sick_ip_address[2] << "."
	       << _sick_ethernet_config.sick_ip_address[3] << std::endl;


    std::ostringstream str_stream;

    str_stream << _sick_ethernet_config.sick_ip_address[0] << "."
	       << _sick_ethernet_config.sick_ip_address[1] << "."
	       << _sick_ethernet_config.sick_ip_address[2] << "."
	       << _sick_ethernet_config.sick_ip_address[3];

    /* Return the std string representation */
    return str_stream.str();

  }

  /**
   * \brief Acquire the subnet mask for the Sick
   * \return The Sick Nav350 subnet mask
   */
  std::string SickNav350::GetSickSubnetMask( ) const {

    /* Declare the string stream */
    std::ostringstream str_stream;

    str_stream << _sick_ethernet_config.sick_subnet_mask[0] << "."
	       << _sick_ethernet_config.sick_subnet_mask[1] << "."
	       << _sick_ethernet_config.sick_subnet_mask[2] << "."
	       << _sick_ethernet_config.sick_subnet_mask[3];

    /* Return the std string representation */
    return str_stream.str();

  }

  /**
   * \brief Acquire the IP address of the Sick gateway
   * \return The Sick Nav350 gateway IP address
   */
  std::string SickNav350::GetSickGatewayIPAddress( ) const {

    /* Declare the string stream */
    std::ostringstream str_stream;

    str_stream << _sick_ethernet_config.sick_gateway_ip_address[0] << "."
	       << _sick_ethernet_config.sick_gateway_ip_address[1] << "."
	       << _sick_ethernet_config.sick_gateway_ip_address[2] << "."
	       << _sick_ethernet_config.sick_gateway_ip_address[3];

    /* Return the std string representation */
    return str_stream.str();

  }

  /**
   * \brief Acquire the Sick Nav350's part number
   * \return The Sick Nav350 part number
   */
  std::string SickNav350::GetSickPartNumber( ) const {
    return _sick_identity.sick_part_number;
  }

  /**
   * \brief Establish a TCP connection to the unit
   */
  void SickNav350::_setupConnection( ) throw( SickIOException, SickTimeoutException ) {

    /* Create the TCP socket */
    if ((_sick_fd = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0) {
      throw SickIOException("SickNav350::_setupConnection: socket() failed!");
    }


    /* Setup the Sick Nav350 inet address structure */
    _sick_inet_address_info.sin_family = AF_INET;                                  // Internet protocol address family
    _sick_inet_address_info.sin_port = htons(_sick_tcp_port);                      // TCP port number
    _sick_inet_address_info.sin_addr.s_addr = inet_addr(_sick_ip_address.c_str()); // Convert ip string to numerical address

    try {

      /* Set to non-blocking so we can time connect */
      _setNonBlockingIO();

      /* Try to connect to the Sick Nav350 */
      int conn_return;
      if ((conn_return = connect( _sick_fd, (struct sockaddr *) &_sick_inet_address_info,sizeof(struct sockaddr_in))) < 0) {

	/* Check whether it is b/c it would block */
	if (errno != EINPROGRESS) {
	  throw SickIOException("SickNav350::_setupConnection: connect() failed!");
	}

	/* Use select to wait on the socket */
	int valid_opt = 0;
	int num_active_files = 0;
	struct timeval timeout_val;                          // This structure will be used for setting our timeout values
	fd_set file_desc_set;                                // File descriptor set for monitoring I/O

	/* Initialize and set the file descriptor set for select */
	FD_ZERO(&file_desc_set);
	FD_SET(_sick_fd,&file_desc_set);

	/* Setup the timeout structure */
	timeout_val.tv_sec=0;
	timeout_val.tv_usec=0;
	timeout_val.tv_usec = DEFAULT_SICK_CONNECT_TIMEOUT;  // Wait for specified time before throwing a timeout

	/* Wait for the OS to tell us that the connection is established! */
	num_active_files = select(getdtablesize(),0,&file_desc_set,0,&timeout_val);

	/* Figure out what to do based on the output of select */
	if (num_active_files > 0) {

	  /* This is just a sanity check */
	  if (!FD_ISSET(_sick_fd,&file_desc_set)) {
  	    throw SickIOException("SickNav350::_setupConnection: Unexpected file descriptor!");
	  }

	  /* Check for any errors on the socket - just to be sure */
	  socklen_t len = sizeof(int);
	  if (getsockopt(_sick_fd,SOL_SOCKET,SO_ERROR,(void*)(&valid_opt),&len) < 0) {
  	    throw SickIOException("SickNav350::_setupConnection: getsockopt() failed!");
	  }

	  /* Check whether the opt value indicates error */
	  if (valid_opt) {
	    throw SickIOException("SickNav350::_setupConnection: socket error on connect()!");
	  }

  	}
	else if (num_active_files == 0) {

	  /* A timeout has occurred! */
	  throw SickTimeoutException("SickNav350::_setupConnection: select() timeout!");

	}
	else {

	  /* An error has occurred! */
	  throw SickIOException("SickNav350::_setupConnection: select() failed!");

	}

      }

      /* Restore blocking IO */
      _setBlockingIO();

    }

    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    catch(...) {
      std::cerr << "SickNav350::_setupConnection - Unknown exception occurred!" << std::endl;
      throw;
    }

    /* Success */
  }




  /**
   * \brief Sets the Sick Nav350 to the requested sensor mode
   * \param new_sick_sensor_mode The desired sensor mode
   */
  void SickNav350::_setSickSensorMode( const uint8_t new_sick_sensor_mode )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* If the new mode matches the current mode then just return */

    try {



    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }

    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* The payload length */
    uint32_t payload_length = 2;

    /* Set the service IDs */
    payload_buffer[0] = 0;//SICK_WORK_SERV_CODE;                                       // Requested service type
    payload_buffer[1] = 0;//_sickSensorModeToWorkServiceSubcode(new_sick_sensor_mode); // Requested service subtype


    /* Define the send/receive message objects */
    SickNav350Message send_message(payload_buffer,payload_length);
    SickNav350Message recv_message;

    try {
      //_sendMessageAndGetReply(send_message,recv_message);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }


    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);



    /* Success */

  }



  /**
   * \brief Get the status of the Sick Nav350
   */
  void SickNav350::_getSickStatus( ) throw( SickTimeoutException, SickIOException ) {

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = 0;//SICK_STAT_SERV_CODE;       // Requested service type
    payload_buffer[1] = 0;//SICK_STAT_SERV_GET_STATUS; // Requested service subtype

    /* Create the Sick messages */
    SickNav350Message send_message(payload_buffer,2);
    SickNav350Message recv_message;

    /* Send the message and check the reply */
    try {
      //_sendMessageAndGetReply(send_message,recv_message);
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << "sick_timeout_exception" << std::endl;
      throw;
    }

    catch(SickIOException &sick_io_exception) {
      std::cerr << "sick_io_exception" << std::endl;
      throw;
    }

    catch(...) {
      std::cerr << "SickNav350::_getSickStatus - Unknown exception!" << std::endl;
      throw;
    }



    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);



    /* Success */
  }


  /**
   * \brief Teardown TCP connection to Sick LD
   */
  void SickNav350::_teardownConnection( ) throw( SickIOException ) {

    /* Close the socket! */
//    if (close(_sick_fd) < 0) {
 //     throw SickIOException("SickLD::_teardownConnection: close() failed!");
  //  }

  }


  void SickNav350::_sendMessageAndGetReply( const SickNav350Message &send_message,
                                          SickNav350Message &recv_message,
                                          const unsigned int timeout_value ) throw( SickIOException, SickTimeoutException ) {

      uint8_t byte_sequence[1] = {0};

      byte_sequence[0] = 's';//send_message.GetServiceCode() | 0x80;

      /* Send message and get reply using parent's method */
      try {
        SickLIDAR< SickNav350BufferMonitor, SickNav350Message >::_sendMessageAndGetReply(send_message,recv_message,byte_sequence,1,0,DEFAULT_SICK_MESSAGE_TIMEOUT,1);
      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
        std::cerr << sick_timeout_exception.what() << std::endl;
        throw;
      }

      /* Handle write buffer exceptions */
      catch (SickIOException &sick_io_exception) {
        std::cerr << sick_io_exception.what() << std::endl;
        throw;
      }

      /* A safety net */
      catch (...) {
        std::cerr << "SickLMS::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
        throw;
      }

    }

  void SickNav350::GetSickIdentity()
  {
	  _getSickIdentity();
	  _getSickSerialNumber();
	  _getSickFirmwareVersion();
	  _getSickSoftwareVersion();
  }

  void SickNav350::_getSickIdentity( )
  {
	    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	    int count=0;
	    std::string command_type=this->READBYNAME_COMMAND;
	    std::string command=this->DEVICEIDENT_COMMAND;

	    for (int i=0;i<command_type.length();i++)
	    {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }

	    payload_buffer[count]=' ';
	    count++;

	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }


	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;

	    /* Send the message and check the reply */
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
		  _SplitReceivedMessage(recv_message);

		  _sick_identity.sick_name = arg[3];
		  _sick_identity.sick_version = arg[5];
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_exception" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_getSickStatus - Unknown exception!" << std::endl;
	      throw;
	    }

  }

  void SickNav350::_getSickSerialNumber( )
  {
	    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	    int count=0;
	    std::string command_type=this->READBYNAME_COMMAND;
	    std::string command=this->SERIALNUMBER_COMMAND;

	    for (int i=0;i<command_type.length();i++)
	    {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }

	    payload_buffer[count]=' ';
	    count++;

	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }

	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;

	    /* Send the message and check the reply */
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
		  _SplitReceivedMessage(recv_message);

		  _sick_identity.sick_serial_number = arg[3];
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_exception" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_getSickStatus - Unknown exception!" << std::endl;
	      throw;
	    }

  }

  void SickNav350::_getSickFirmwareVersion( )
  {
	    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	    int count=0;
	    std::string command_type=this->READBYNAME_COMMAND;
	    std::string command=this->DEVICEINFO_COMMAND;

	    for (int i=0;i<command_type.length();i++)
	    {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }

	    payload_buffer[count]=' ';
	    count++;

	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }


	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;

	    /* Send the message and check the reply */
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
		  _SplitReceivedMessage(recv_message);

		  _sick_identity.sick_firmware_version = arg[2];
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_exception" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_getSickStatus - Unknown exception!" << std::endl;
	      throw;
	    }

  }

  void SickNav350::_getSickSoftwareVersion( )
  {
	    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	    int count=0;
	    std::string command_type=this->READBYNAME_COMMAND;
	    std::string command=this->FIRMWAREVERSION_COMMAND;

	    for (int i=0;i<command_type.length();i++)
	    {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }

	    payload_buffer[count]=' ';
	    count++;

	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }


	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;

	    /* Send the message and check the reply */
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
		  _SplitReceivedMessage(recv_message);

		  _sick_identity.sick_application_software_version = arg[3];
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_exception" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_getSickStatus - Unknown exception!" << std::endl;
	      throw;
	    }

  }

  void SickNav350::SetOperatingMode(int mode)
  {
	  std::cout<<"set operating_mode_command"<<std::endl;
	    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	    int count=0;
	    std::string command_type=this->METHODCALL_COMMAND;
	    std::string command=this->SETMODE_COMMAND;
	    for (int i=0;i<command_type.length();i++)
	    {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    payload_buffer[count]=48+mode;
	    count++;

	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;


	    uint8_t byte_sequence[] = {115,65,78,32,109,78,69,86,65,67,104,97,110,103,101,83,116,97,116,101};
	    int byte_sequence_length=20;


	    /* Send the message and check the reply */
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
	      _recvMessage(recv_message,byte_sequence,byte_sequence_length,DEFAULT_SICK_MESSAGE_TIMEOUT);
	      //sick_nav350_sector_data_t.
//	      _SplitReceivedMessage(recv_message);

	      std::cout<<"Set operating mode"<<std::endl;
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_exception" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_set operating mode - Unknown exception!" << std::endl;
	      throw;
		}

  }

  void SickNav350::SetScanDataFormat (uint8_t dataMode, uint8_t showRSSI)throw( SickIOException, SickTimeoutException, SickErrorException )
  {
	  this->SetAccessMode(3);

	  std::cout<<"set scan data format: " << dataMode<< ", " << showRSSI <<std::endl;
	  uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	  int count=0;
	  std::string command_type=this->WRITEBYNAME_COMMAND;
	  std::string command=this->SCANDATAFORMAT_COMMAND;
	  for (int i=0;i<command_type.length();i++)
	  {
		  payload_buffer[count]=command_type[i];
		  count++;
	  }
	  payload_buffer[count]=' ';
	  count++;
	  for (int i=0;i<command.length();i++)
	  {
		  payload_buffer[count]=command[i];
		  count++;
	  }
	  payload_buffer[count]=' ';
	  count++;
	  payload_buffer[count]=48+dataMode;
	  count++;
	  payload_buffer[count]=' ';
	  count++;
	  payload_buffer[count]=48+showRSSI;
      count++;
	  /* Create the Sick messages */
	  SickNav350Message send_message(payload_buffer,count);
	  SickNav350Message recv_message;


	  uint8_t byte_sequence[] = {'s','W','A',' ','N','A','V','S','c','a','n','D','a','t','a','F','o','r','m','a','t'};
	  int byte_sequence_length=21;


	  /* Send the message and check the reply */
	  try {
		  _sendMessageAndGetReply(send_message,recv_message);
		  //_recvMessage(recv_message,byte_sequence,byte_sequence_length,DEFAULT_SICK_MESSAGE_TIMEOUT);
		  //sick_nav350_sector_data_t.
		  //	      _SplitReceivedMessage(recv_message);

		  std::cout<<"Set data scan format response:" <<std::endl;
		  //recv_message.Print();
	  }

	  catch(SickTimeoutException &sick_timeout_exception) {
		  std::cerr << "sick_timeout_exception" << std::endl;

		  throw;
	  }

	  catch(SickIOException &sick_io_exception) {
		  std::cerr << "sick_io_exception" << std::endl;
		  throw;
	  }

	  catch(...) {
		  std::cerr << "SickNav350::_set scan data format - Unknown exception!" << std::endl;
		  throw;
	  }

  }

  void SickNav350::SetAccessMode( uint8_t newMode) throw(SickIOException, SickTimeoutException, SickErrorException)
  {
	  std::cout<<"set access mode: " << newMode<<std::endl;
	  uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	  int count=0;
	  std::string command_type=this->METHODCALL_COMMAND;
	  std::string command=this->SETACCESSMODE_COMMAND;
	  std::string password="";
	  if (newMode == 2)
		  password="B21ACE26"; //Password for Operator mode
	  else if (newMode == 3)
		  password="F4724744"; //Password for Client mode

	  for (int i=0;i<command_type.length();i++)
	  {
		  payload_buffer[count]=command_type[i];
		  count++;
	  }
	  payload_buffer[count]=' ';
	  count++;
	  for (int i=0;i<command.length();i++)
	  {
		  payload_buffer[count]=command[i];
		  count++;
	  }
	  payload_buffer[count]=' ';
	  count++;
	  payload_buffer[count]=48+newMode;
	  count++;
	  payload_buffer[count]=' ';
	  count++;
	  for (int i=0; i<password.length(); i++)
	  {
		  payload_buffer[count]=password[i];
		  count++;
	  }

	  /* Create the Sick messages */
	  SickNav350Message send_message(payload_buffer,count);
	  SickNav350Message recv_message;


	  uint8_t byte_sequence[] = {'s','A','N',' ','S','e','t','A','c','c','e','s','s','M','o','d','e'};
	  int byte_sequence_length=17;


	  /* Send the message and check the reply */
	  try {
		  _sendMessageAndGetReply(send_message,recv_message);
		  //_recvMessage(recv_message,byte_sequence,byte_sequence_length,DEFAULT_SICK_MESSAGE_TIMEOUT);
		  //sick_nav350_sector_data_t.
		  //	      _SplitReceivedMessage(recv_message);

		  std::cout<<"Set access mode response:" <<std::endl;
		  //recv_message.Print();
	  }

	  catch(SickTimeoutException &sick_timeout_exception) {
		  std::cerr << "sick_timeout_exception" << std::endl;

		  throw;
	  }

	  catch(SickIOException &sick_io_exception) {
		  std::cerr << "sick_io_exception" << std::endl;
		  throw;
	  }

	  catch(...) {
		  std::cerr << "SickNav350::_set access mode - Unknown exception!" << std::endl;
		  throw;
	  }

  }

  void SickNav350::GetData(int wait,int dataset)
  {
	  uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	  int count=0;
	  std::string command_type=this->METHODCALL_COMMAND;
	  std::string command=this->POSEDATA_COMMAND;
	  for (int i=0;i<command_type.length();i++)
	  {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    payload_buffer[count]=48+wait;
	    count++;
	    payload_buffer[count]=' ';
	    count++;
	    payload_buffer[count]=48+dataset;
	    count++;

	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;


	    uint8_t byte_sequence[] = {115,65,78,32,109,78,80,79,83,71,101,116,68,97,116,97};
	    int byte_sequence_length=5;


	    /* Send the message and check the reply */
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
	      _recvMessage(recv_message,byte_sequence,byte_sequence_length,DEFAULT_SICK_MESSAGE_TIMEOUT);
	      //sick_nav350_sector_data_t.=0;
	      _SplitReceivedMessage(recv_message);
//	      std::cout<<"argument count="<<argumentcount_<<std::endl;
	      _ParseScanData();
//	      std::cout<<"Get data"<<std::endl;
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_except=0;ion" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_get data - Unknown exception!" << std::endl;
	      throw;
	    }
  }
  void SickNav350::GetDataLandMark(int wait,int dataset)
  {
	    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	    int count=0;
	    std::string command_type=this->METHODCALL_COMMAND;
	    std::string command=this->GETLANDMARK_COMMAND;
	    for (int i=0;i<command_type.length();i++)
	    {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    payload_buffer[count]=48+wait;
	    count++;
	    payload_buffer[count]=' ';
	    count++;
	    payload_buffer[count]=48+dataset;
	    count++;

	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;


	    uint8_t byte_sequence[] = {115,65,78,32,109,78,80,79,83,71,101,116,68,97,116,97};
	    int byte_sequence_length=5;


	    /* Send the message and check the reply */
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
	      _recvMessage(recv_message,byte_sequence,byte_sequence_length,DEFAULT_SICK_MESSAGE_TIMEOUT);
	      //sick_nav350_sector_data_t.=0;
	      _SplitReceivedMessage(recv_message);
//	      std::cout<<"argument count="<<argumentcount_<<std::endl;
	      _ParseScanDataLandMark();
//	      std::cout<<"Get data"<<std::endl;
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_except=0;ion" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_get data - Unknown exception!" << std::endl;
	      throw;
	    }
  }

  void SickNav350::_SplitReceivedMessage(SickNav350Message recv_message)
  {
	  std::string str="";
	  argumentcount_=0;
	  int messagelength=recv_message.GetMessageLength();
	  uint8_t *message=new uint8_t[messagelength];
	  recv_message.GetMessage(message);
	  for (int i=0;i<messagelength;i++)
	  {
		  if (message[i]==' ')
		  {
			  arg[argumentcount_]=str;
			  argumentcount_++;
			  str="";
			  continue;
		  }
		  str=str+(char) message[i];

	  }
	  arg[argumentcount_] = str;

	  delete []message;
  }
  void SickNav350::_ParseScanData()
  {
	  int count=0;
	  if (arg[3]!="0")
	  {
		  std::cout<<"Scan data unsuccesfull"<<std::endl;
		  return;
	  }
	  if (arg[5]<"1")
	  {
		 std::cout<<"Wrong selected signals"<<std::endl;
		 return;
	  }
	  count=6;
	  if (arg[count++]=="1")
	  {
//		  std::cout<<"Pose data follow"<<std::endl;
		  std::string str=arg[count++]+" "+arg[count++]+" "+arg[count++];
		  if (arg[count++]=="1")
		  {
		  }
	  }
	  if (arg[count++]=="1")
	  {
//		  std::cout<<"Landmark data follow"<<std::endl;
//		  for ()
	  }

	  switch (atoi(arg[count++].c_str()))
	  {
	  case 0:
		  std::cout<<"No scan data"<<std::endl;
		  break;
	  case 1:
//		  std::cout<<"One output channel"<<std::endl;
		  if (arg[count++]=="DIST1")
		  {
			  count++; //scalefactor=1
			  count++; //offset=0
			  MeasuredData_->angle_start=(double) _ConvertHexToDec(arg[count++])/1000;
//			  std::cout<<"Start angle(grad):"<<MeasuredData_->angle_start<<std::endl;
			  std::string str=arg[count++];
			  MeasuredData_->angle_step=(double) _ConvertHexToDec(str)/1000;
//			  std::cout<<"Resolution (deg):"<<MeasuredData_->angle_step<<std::endl;
			  MeasuredData_->timestamp_start=_ConvertHexToDec(arg[count++]);
//			  std::cout<<"Timestamp start (ms)"<<MeasuredData_->timestamp_start<<std::endl;
			  MeasuredData_->num_data_points=_ConvertHexToDec(arg[count++]);
//			  std::cout<<"Number of data points "<<MeasuredData_->num_data_points<<std::endl;
			  MeasuredData_->angle_stop=MeasuredData_->angle_start+(MeasuredData_->num_data_points-1)*(MeasuredData_->angle_step);
			  for (int i=0;i<MeasuredData_->num_data_points;i++)
			  {
				  MeasuredData_->range_values[i]=_ConvertHexToDec(arg[count++]);
			  }
//			  std::cout<<"Data read: "<<count<<std::endl;
//			  std::cout<<"data received "<<argumentcount_<<std::endl;

		  }
		  else
		  {

		  }
		  break;
	  case 2:
		  std::cout<<"Two output channels"<<std::endl;
		  break;
	  }


  }
  void SickNav350::_ParseScanDataLandMark()
  {
/*	  for (int i=0;i<this->argumentcount_;i++)
	  {
		  std::cout<<" "<<arg[i];
	  }
	  std::cout<<std::endl;*/
	  int count=0;
	  if (arg[3]!="0")
	  {
		  std::cout<<"Scan data unsuccessful"<<std::endl;
		  return;
	  }
	  if (arg[5]<"1")
	  {
		 std::cout<<"Wrong selected signals"<<std::endl;
		 return;
	  }
	  count=6;
	  /*if (arg[count++]=="1")
	  {
		  std::cout<<"Pose data follow"<<std::endl;
		  std::cout<<arg[count++]+" "+arg[count++]+" "+arg[count++]<<std::endl;
		  if (arg[count++]=="1")
		  {
		  }
	  }*/
	  if (arg[count++]=="1")
	  {
		  std::cout<<"\nLandmark data follow"<<std::endl;
		  std::cout<<"Landmark filter "<<
				 arg[count++] <<std::endl;
		  int refcount=atoi(arg[count++].c_str());
		  std::cout<<"reflector count: "<<refcount<<"\n" <<std::endl;
		  for (int i=0;i<refcount;i++)
		  {
			  if (arg[count++]=="0")
			  {
				  //std::cout<<"Not Cartesian"<<std::endl;
			  }
			  else
			  {
				  std::cout<<"Cartesian"<<std::endl;
          int x = atoi(arg[count++].c_str());
          int y = atoi(arg[count++].c_str());
				  //arg[count++];
				  //arg[count++];
          std::cout << "[" << x << "," << y << "]" << std::endl;

			  }
			  if (arg[count++]=="0")
			  {
				  //std::cout<<"Not Polar"<<std::endl;
			  }
			  else
			  {
				  //std::cout<<"Polar"<<std::endl;
				  arg[count++];
				  arg[count++];
			  }
			  if (arg[count++]=="1")
			  {
				  //std::cout<<"optional reflector data"<<std::endl;
			  }
			  else
			  {
				  //std::cout<<"no optional reflector data"<<std::endl;
			  }

		  }
//		  for ()
	  }

	  switch (atoi(arg[count++].c_str()))
	  {
	  case 0:
		  std::cout<<"No scan data"<<std::endl;
		  break;
	  case 1:
		  //std::cout<<"One output channel"<<std::endl;
		  if (arg[count++]=="DIST1")
		  {
			  count++; //scalefactor=1
			  count++; //offset=0
			  MeasuredData_->angle_start=(double) _ConvertHexToDec(arg[count++])/1000;
			  //std::cout<<"Start angle(grad):"<<MeasuredData_->angle_start<<std::endl;
			  std::string str=arg[count++];
			  MeasuredData_->angle_step=(double) _ConvertHexToDec(str)/1000;
			  //std::cout<<"Resolution (deg):"<<MeasuredData_->angle_step<<std::endl;
			  MeasuredData_->timestamp_start=_ConvertHexToDec(arg[count++]);
			  //std::cout<<"Timestamp start (ms)"<<MeasuredData_->timestamp_start<<std::endl;
			  MeasuredData_->num_data_points=_ConvertHexToDec(arg[count++]);
			  //std::cout<<"Number of data points "<<MeasuredData_->num_data_points<<std::endl;
			  MeasuredData_->angle_stop=MeasuredData_->angle_start+(MeasuredData_->num_data_points-1)*(MeasuredData_->angle_step);
			  for (int i=0;i<MeasuredData_->num_data_points;i++)
			  {
				  MeasuredData_->range_values[i]=_ConvertHexToDec(arg[count++]);
			  }
			  //std::cout<<"Data read: "<<count<<std::endl;
			  //std::cout<<"data received "<<argumentcount_<<std::endl;

		  }
		  else
		  {

		  }
		  break;
	  case 2:
		  std::cout<<"Two output channels"<<std::endl;
		  break;
	  }

	  if (arg[count++] == "1")
	  {
		  //std::cout << "Includes remission data" << std::endl;
		  count+=6;
		  uint16_t num_data = _ConvertHexToDec(arg[count++]);
		  //std::cout << num_data << std::endl;
		  for (int i=0; i<num_data; i++)
		  {
			  MeasuredData_->echo_values[i]=_ConvertHexToDec(arg[count++]);
		  }
	  }

  }

  int SickNav350::_ConvertHexToDec(std::string num)
  {
	  int suma=0;
	  for (int i=0;i<num.length();i++)
	  {
		   if (num[i]>=65)
		  {
			  suma=suma*16+num[i]-65+10;
		  }
		  else
		  {
			  suma=suma*16+num[i]-48;
		  }
	  }
	  return suma;

  }
  void SickNav350::GetSickMeasurements(double* range_values,unsigned int *num_measurements,
  		double *sector_step_angle,
  		double *sector_start_angle,
  		double *sector_stop_angle,
  		unsigned int *sector_start_timestamp,
  		unsigned int *sector_stop_timestamp)
  {
	  for (int i=0;i<MeasuredData_->num_data_points;i++)
	  {
		  range_values[i]=MeasuredData_->range_values[i];
	  }
	  *num_measurements=MeasuredData_->num_data_points;
	  *sector_step_angle=MeasuredData_->angle_step;
	  *sector_start_angle=MeasuredData_->angle_start;
	  *sector_stop_angle=MeasuredData_->angle_stop;
	  *sector_start_timestamp=MeasuredData_->timestamp_start;
	  *sector_stop_timestamp=MeasuredData_->timestamp_start;

  }
  void SickNav350::GetSickMeasurementsWithRemission(double* range_values, unsigned int *remission_values,unsigned int *num_measurements,
      		    double *sector_step_angle,
          		double *sector_start_angle,
          		double *sector_stop_angle,
          		unsigned int *sector_start_timestamp,
          		unsigned int *sector_stop_timestamp)
  {
	  for (int i=0;i<MeasuredData_->num_data_points;i++)
	  {
		  range_values[i]=MeasuredData_->range_values[i];
		  remission_values[i]=MeasuredData_->echo_values[i];
	  }
	  *num_measurements=MeasuredData_->num_data_points;
	  *sector_step_angle=MeasuredData_->angle_step;
	  *sector_start_angle=MeasuredData_->angle_start;
	  *sector_stop_angle=MeasuredData_->angle_stop;
	  *sector_start_timestamp=MeasuredData_->timestamp_start;
	  *sector_stop_timestamp=MeasuredData_->timestamp_start;
  }
  void SickNav350::GetResponseFromCustomMessage(uint8_t *req,int req_size,uint8_t *res,int* res_size)
  {
	    SickNav350Message send_message(req,req_size);
	    SickNav350Message recv_message;

	    uint8_t byte_sequence[] = {115,65,78,32,109,78,80,79,83,71,101,116,68,97,116,97};
	    int byte_sequence_length=5;


	    /* Send the message and check the reply */
	    *res_size=0;
	    try {
	      _sendMessageAndGetReply(send_message,recv_message);
	      *res_size=recv_message.GetMessageLength();
	      recv_message.GetMessage(res);
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_except=0;isector_data_tagon" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_get data - Unknown exception!" << std::endl;
	      throw;
	    }

  }
  void SickNav350::GetDataNavigation(int wait,int dataset)
  {
	    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
	    int count=0;
	    std::string command_type=this->METHODCALL_COMMAND;
	    std::string command=this->POSEDATA_COMMAND;
	    for (int i=0;i<command_type.length();i++)
	    {
	    	payload_buffer[count]=command_type[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    for (int i=0;i<command.length();i++)
	    {
	    	payload_buffer[count]=command[i];
	    	count++;
	    }
	    payload_buffer[count]=' ';
	    count++;
	    payload_buffer[count]=48+wait;
	    count++;
	    payload_buffer[count]=' ';
	    count++;
	    payload_buffer[count]=48+dataset;
	    count++;

	    /* Create the Sick messages */
	    SickNav350Message send_message(payload_buffer,count);
	    SickNav350Message recv_message;


	    uint8_t byte_sequence[] = {115,65,78,32,109,78,80,79,83,71,101,116,68,97,116,97};
	    int byte_sequence_length=5;


	    /* Send the message and check the reply */
	    try {
//		      std::cout<<"before first message"<<std::endl;

 	      _sendMessageAndGetReply(send_message,recv_message);
//	      std::cout<<"first message"<<std::endl;
	      _recvMessage(recv_message,byte_sequence,byte_sequence_length,DEFAULT_SICK_MESSAGE_TIMEOUT);
//	      std::cout<<"second message"<<std::endl;

	      //sick_nav350_sector_data_t.=0;
	      _SplitReceivedMessage(recv_message);
//	       std::cout<<"argument count="<<argumentcount_<<std::endl;
	      _ParseScanDataNavigation();
//	      std::cout<<"Get data"<<std::endl;
	    }

	    catch(SickTimeoutException &sick_timeout_exception) {
	      std::cerr << "sick_timeout_except=0;ion" << std::endl;

	      throw;
	    }

	    catch(SickIOException &sick_io_exception) {
	      std::cerr << "sick_io_exception" << std::endl;
	      throw;
	    }

	    catch(...) {
	      std::cerr << "SickNav350::_get data - Unknown exception!" << std::endl;
	      throw;
	    }
  }

  void SickNav350::SetSpeed(double x, double y, double phi, int timestamp, int coordbase){

    uint8_t payload_buffer[SickNav350Message::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    int count = 0;
    std::string command_type=this->METHODCALL_COMMAND;
    std::string command=this->SETSPEED_COMMAND;

    for(int i = 0; i < command_type.length(); i++){

      payload_buffer[count] = command_type[i];
      count++;

    }

    payload_buffer[count] = ' ';
    count++;

    for(int i = 0; i < command.length(); i++){

      payload_buffer[count] = command[i];
      count++;

    }

    payload_buffer[count] = ' ';
    count++;
    char c[100];

    sprintf(c, "%d",(int)(x*1000));

    if(c[0] != '-'){

      payload_buffer[count] = '+';
      count++;

    }

    for(int i = 0; i < strlen(c); i++){

      payload_buffer[count] = c[i];
      count++;

    }

    payload_buffer[count] = ' ';
    count++;

    sprintf(c, "%d", (int)(y*1000));
    if(c[0] != '-'){

      payload_buffer[count] = '+';
      count++;

    }

    for(int i = 0; i < strlen(c); i++){

      payload_buffer[count]=c[i];
      count++;

    }

    payload_buffer[count]=' ';
    count++;

    sprintf(c, "%d", (int)(phi/3.14159*180*1000));
    if(c[0] != '-'){

      payload_buffer[count] = '+';
      count++;

    }

    for(int i = 0; i < strlen(c); i++){

      payload_buffer[count]=c[i];
      count++;

    }

    payload_buffer[count]=' ';
    count++;

    sprintf(c, "%d", timestamp);
    if(c[0] != '-'){

      payload_buffer[count] = '+';
      count++;

    }

    for(int i = 0; i < strlen(c); i++){

      payload_buffer[count]=c[i];
      count++;

    }

    payload_buffer[count]=' ';
    count++;

    payload_buffer[count]=48+coordbase;
    count++;
/*
    for(int i = 0; i< count; i++){
      std::cout << payload_buffer[i];
    }
    std::cout << std::endl;
*/
    SickNav350Message send_message(payload_buffer,count);
    SickNav350Message recv_message;

    try{

      _sendMessageAndGetReply(send_message,recv_message);

    }catch(...){
      std::cerr << "SickNav350::setSpeed - Unknow Exception!" << std::endl;
      throw;
    }


  }
  void SickNav350::_ParseScanDataNavigation()
  {
/*	  for (int i=0;i<this->argumentcount_;i++)
	  {
		  std::cout<<" "<<arg[i];
	  }
	  std::cout<<std::endl;*/
	  int count=0;
	  if (arg[3]!="0")
	  {
		  std::cout<<"Scan data unsuccessful"<<std::endl;
		  return;
	  }
	  if (arg[5]<"1")
	  {
		 std::cout<<"Wrong selected signals"<<std::endl;
		 return;
	  }
	  count=6;
/*	  if (arg[count++]=="1")
	  {
		  std::cout<<"Pose data follow"<<std::endl;
		  std::cout<<arg[count++]+" "+arg[count++]+" "+arg[count++]<<std::endl;
		  if (arg[count++]=="1")
		  {
		  }
	  }*/
	  if (arg[count++]=="1")
	  {
//		  std::cout<<"Pose data follow"<<std::endl;

		  PoseData_.x=_ConvertHexToDec(arg[count++]);
		  PoseData_.y=_ConvertHexToDec(arg[count++]);
		  PoseData_.phi=_ConvertHexToDec(arg[count++]);
		  PoseData_.optionalPoseData=_ConvertHexToDec(arg[count++]);
		  if (PoseData_.optionalPoseData==1)
		  {
			  PoseData_.outputMode=_ConvertHexToDec(arg[count++]);
			  PoseData_.timeStamp=_ConvertHexToDec(arg[count++]);
			  PoseData_.meanDeviation=_ConvertHexToDec(arg[count++]);
			  PoseData_.positionMode=_ConvertHexToDec(arg[count++]);
			  PoseData_.infoState=_ConvertHexToDec(arg[count++]);
			  PoseData_.numUsedReflectors=_ConvertHexToDec(arg[count++]);
		  }

	  }
	  if (arg[count++]=="1")
	  {
	//	  std::cout<<"Landmark data follow"<<std::endl;
		  ReflectorData_.filter=_ConvertHexToDec(arg[count++]);
		//  std::cout<<"Landmark filter "<<std::endl;
		  int refcount=atoi(arg[count++].c_str());
		  ReflectorData_.num_reflector=refcount;
//		  std::cout<<"reflector count: "<<refcount<<std::endl;
		  for (int i=0;i<refcount;i++)
		  {
			  if (arg[count++]=="0")
			  {
				  ReflectorData_.cart[i]=0;
//				  std::cout<<"Not Cartesian"<<std::endl;
			  }
			  else
			  {
//				  std::cout<<"Cartesian"<<std::endl;
				  ReflectorData_.cart[i]=1;
				  ReflectorData_.x[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.y[i]=_ConvertHexToDec(arg[count++]);

			  }
			  if (arg[count++]=="0")
			  {
				  ReflectorData_.polar[i]=0;

//				  std::cout<<"Not Polar"<<std::endl;
			  }
			  else
			  {
//				  std::cout<<"Polar"<<std::endl;
				  ReflectorData_.polar[i]=1;
				  ReflectorData_.dist[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.phi[i]=_ConvertHexToDec(arg[count++]);
			  }
			  if (arg[count++]=="1")
			  {
				  ReflectorData_.optional[i]=1;

//				  std::cout<<"optional reflector data"<<std::endl;
				  ReflectorData_.LocalID[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.GlobalID[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.type[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.subtype[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.quality[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.timestamp[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.size[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.hitCount[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.meanEchoAmplitude[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.indexStart[i]=_ConvertHexToDec(arg[count++]);
				  ReflectorData_.indexEnd[i]=_ConvertHexToDec(arg[count++]);

			  }
			  else
			  {
				  ReflectorData_.optional[i]=0;
				  std::cout<<"no optional reflector data"<<std::endl;
			  }

		  }
//		  for ()
	  }

	  switch (atoi(arg[count++].c_str()))
	  {
	  case 0:
		  std::cout<<"No scan data"<<std::endl;
		  break;
	  case 1:
//		  std::cout<<"One output channel"<<std::endl;
		  if (arg[count++]=="DIST1")
		  {
			  count++; //scalefactor=1
			  count++; //offset=0
			  MeasuredData_->angle_start=(double) _ConvertHexToDec(arg[count++])/1000;
//			  std::cout<<"Start angle(grad):"<<MeasuredData_->angle_start<<std::endl;
			  std::string str=arg[count++];
			  MeasuredData_->angle_step=(double) _ConvertHexToDec(str)/1000;
//			  std::cout<<"Resolution (deg):"<<MeasuredData_->angle_step<<std::endl;
			  MeasuredData_->timestamp_start=_ConvertHexToDec(arg[count++]);
//			  std::cout<<"Timestamp start (ms)"<<MeasuredData_->timestamp_start<<std::endl;
			  MeasuredData_->num_data_points=_ConvertHexToDec(arg[count++]);
//			  std::cout<<"Number of data points "<<MeasuredData_->num_data_points<<std::endl;
			  MeasuredData_->angle_stop=MeasuredData_->angle_start+(MeasuredData_->num_data_points-1)*(MeasuredData_->angle_step);
			  for (int i=0;i<MeasuredData_->num_data_points;i++)
			  {
				  MeasuredData_->range_values[i]=_ConvertHexToDec(arg[count++]);
			  }
//			  std::cout<<"Data read: "<<count<<std::endl;
	//		  std::cout<<"data received "<<argumentcount_<<std::endl;

		  }
		  else
		  {

		  }
		  break;
	  case 2:
		  std::cout<<"Two output channels"<<std::endl;
		  break;
	  }


  }

} //namespace SickToolbox
