/** @mainpage OpcUaServer for the CAENels FAST-PS power supplies
 *
 *  Version 1.1  18.2.2021
 *
 *  @author U. Lehnert, Helmholtz-Zentrum Dresden-Rossendorf
 *
 *  This is an OPC UA server running on the CAENels
 *  [FAST-PS](http://www.caenels.com/products/fast-ps/) power supplies
 *  for remote control of the devices.
 *  
 *  It is based on the [Open62541](https://github.com/open62541/open62541/)
 *  open source implementation of OPC UA.
 *  
 *  @section Functionality
 *  - Provides an OPC-UA server at TCP/IP port 16664.
 *  - Access to device data is handled via the provided TCP server (port 10001).
 *  - Server configuration is loadad from file /etc/opcua.xml
 *
 *  The OPC UA server compiles and runs stabily on all power supplies tested.
 *  All functionality necessary to user the supllies to power corrector coils
 *  in an accelerator control system environment is provided. This does not
 *  cover the whole functionality provided by the devices, only the essentials.
 *
 *  For faster control an UDP server was implemented listening at port 16665.
 *  This is now provided as a separate binary.
 *  
 *  @section Build
 *  The server is built with a cross-compiler running on a Linux system
 *  for the ARM target CPU of the power supplies.
 *  
 *  The OPC UA stack needs to be downloaded and built. This can be done on
 *  the development system - there is no binary code produced at this stage.
 *  The complete stack is obtained in two (amalgamated) files.
 *  - open62541.h
 *  - open62541.c
 *  
 *  In addition the libxml2 library is required. It needs to be built
 *  and installed into the cross-target tool chain.
 *  
 *  A makefile is not yet provided, just a few lines are required to build the server.
 *  - source ../tools/environment
 *  - $CC -std=c99 -c open62541.c
 *  - $CC -std=c99 -c -I $SDKTARGETSYSROOT/usr/include/libxml2/ OpcUaServer.c
 *  - $CXX -o opcuaserver OpcUaServer.o open62541.o -lxml2
 *
 *  @section Installation
 *  For istallation a few files need to be copied onto the device:
 *  - opcuaserver binary installed in /tmp/ for testing
 *  - opcua.xml configuration file in /etc/
 *  - libxml2.so.2 in /usr/lib/
 *
 *  The server can then be run by executing /tmp/opcuaserver.
 * 
 *  @section Testing
 *  For a first test of the server an universal OPC UA client like
 *  [UaExpert](https://www.unified-automation.com/products/development-tools/uaexpert.html) is recommended.
 *
 *  @section TODO
 *  - evaluate the AK/NAK responses
 *  - error handling when reading the device response, don't just die
 *  - VER
 *  - LOOP
 *  - MSAVE
 *
 */

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>		     // for flags
#include <stdlib.h>		     // for exit()
#include <signal.h>		     // for signal()
#include <errno.h>		     // for error messages
#include <math.h>

#include <sys/socket.h>      // for TCP/IP communication
#include <netinet/udp.h>	 // declarations for udp header
#include <netinet/ip.h>		 // declarations for ip header
#include <arpa/inet.h>

#include <libxml/parser.h>
#include <libxml/tree.h>

#include "open62541.h"       // the OPC-UA library

/***********************************/
/* Server-related variables        */
/***********************************/

// the OPC-UA server
UA_Server *server;
unsigned short serverPortNumber;
// log to the console
UA_Logger logger = Logger_Stdout;

// Overview of the OPC-UA variables hosted by this server.
// all parameters are double-valued registers accessed with MRG/MWG
// the list of parameters is read from the XML configuration file
// all parameters are stored with a register number
#define maxreg 40           // the maximum number of registers
/*
    Server
    |   ...
    Device
    |   Name
    |   Status
    |   OutputOn
    |   MReset
    SetPoint
    |   Voltage
    |   Current
    |   VoltageSetpoint
    |   CurrentSetpoint
    Parameters
    |   PID_I_Kp_v
    |   ...
*/

// this variable is a flag for the running server
// when set to false the server stops
UA_Boolean running = true;

/***********************************/
/* interrupt and error handling    */
/***********************************/

// print error message and abort the running program
void Die(char *mess)
{
    UA_LOG_FATAL(logger, UA_LOGCATEGORY_SERVER, mess);
    exit(1);
}

// handle SIGINT und SIGTERM
static void stopHandler(int signal)
{
    UA_LOG_INFO(logger, UA_LOGCATEGORY_SERVER, "received ctrl-c");
    running = 0;
}

/***********************************/
/* TCP/IP communication            */
/***********************************/

int sock;
struct sockaddr_in tcpserver;

#define BUFSIZE 80
char command[BUFSIZE];			// command string buffer
char response[BUFSIZE];			// receive buffer

// send the string in command to the device
// receive the answer in response
// return the number of read characters
unsigned int TcpSendReceive() {
    unsigned int buflen = strlen(command);
    if (send(sock, command, buflen, 0) != buflen)
        Die("Mismatch in number of sent bytes");
    // receive the answer from the server
    unsigned int reclen;
    reclen = recv(sock, response, BUFSIZE-1, 0);
    response[reclen] = '\0';			// assure null terminated string
    return reclen;
}

/***********************************/
/* specialized read/write methods  */
/* for PS specific variables       */
/***********************************/

// switch the output on/off
// handle (pointing at DeviceOutputOn) is interpreted as a boolean on/off information
UA_StatusCode writeDeviceOutputOn(void *handle, const UA_NodeId nodeid,
            const UA_Variant *data, const UA_NumericRange *range) {
    if(UA_Variant_isScalar(data) && data->type == &UA_TYPES[UA_TYPES_BOOLEAN] && data->data) {
        *(UA_Boolean*)handle = *(UA_Boolean*)data->data;
    }
    if (*(bool *)handle) {
        // switch on the output
        UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, "MON");
        strcpy(command,"MON\r\n");
        TcpSendReceive();
    } else {
        // switch off the output
        UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, "MOFF");
        strcpy(command,"MOFF\r\n");
        TcpSendReceive();
    };
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode readDeviceOutputOn( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // send status request to server    
    strcpy(command,"MST\r\n");
    unsigned int reclen = TcpSendReceive();
    // convert the answer into a number
    // first 5 charecters are #MST:
    unsigned int status;
    sscanf(response+5,"%x",&status);
    *(bool *)handle = (status & 1 == 1);
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Boolean *)handle, &UA_TYPES[UA_TYPES_BOOLEAN]);
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode readDeviceStatus( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // send request to server    
    strcpy(command,"MST\r\n");
    unsigned int reclen = TcpSendReceive();
    // convert the answer into a number
    // first 5 charecters are #MST:
    unsigned int status;
    int result = sscanf(response+5,"%x",&status);
    if (result==0)
        *(unsigned int *)handle = -1;
    else
        *(unsigned int *)handle = status;
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (unsigned int *)handle, &UA_TYPES[UA_TYPES_UINT32]);
    return UA_STATUSCODE_GOOD;
}

// write an MRESET command when set to true
UA_StatusCode writeMReset(void *handle, const UA_NodeId nodeid,
            const UA_Variant *data, const UA_NumericRange *range) {
    if(UA_Variant_isScalar(data) && data->type == &UA_TYPES[UA_TYPES_BOOLEAN] && data->data) {
        *(UA_Boolean*)handle = *(UA_Boolean*)data->data;
    }
    if (*(bool *)handle) {
        // report to log
        UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, "MRESET");
        // send command
        strcpy(command,"MRESET\r\n");
        TcpSendReceive();
    };
    return UA_STATUSCODE_GOOD;
}

// switch to SFP update mode and back
UA_StatusCode writeDeviceModeSFP(void *handle, const UA_NodeId nodeid,
            const UA_Variant *data, const UA_NumericRange *range) {
    if(UA_Variant_isScalar(data) && data->type == &UA_TYPES[UA_TYPES_BOOLEAN] && data->data) {
        *(UA_Boolean*)handle = *(UA_Boolean*)data->data;
    }
    if (*(bool *)handle) {
        // switch on the output
        UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, "UPMODE:SFP");
        strcpy(command,"UPMODE:SFP\r\n");
        TcpSendReceive();
    } else {
        // switch off the output
        UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, "UPMODE:NORMAL");
        strcpy(command,"UPMODE:NORMAL\r\n");
        TcpSendReceive();
    };
    return UA_STATUSCODE_GOOD;
}

// read the SFP output mode
UA_StatusCode readDeviceModeSFP( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // send status request to server    
    strcpy(command,"UPMODE\r\n");
    unsigned int reclen = TcpSendReceive();
    // first 8 charecters are #UPMODE:
    if (strncmp(response+8,"SFP",3)==0)
        *(bool *)handle = true;
    else
        *(bool *)handle = false;
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Boolean *)handle, &UA_TYPES[UA_TYPES_BOOLEAN]);
    return UA_STATUSCODE_GOOD;
}

// callback routine for reading the current value
UA_StatusCode readCurrent( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // handle is supposed to point to CurrentReadback
    // send request to server    
    strcpy(command,"MRI\r\n");
    unsigned int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MRI:
    if(strncmp(response,"#MRI:",5)==0)
        sscanf(response+5,"%lf",(double *)handle);
    else
    {
        // printf("wrong MRI response : %s",response);
        *(double *)handle = 999.999;
    }
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)handle, &UA_TYPES[UA_TYPES_DOUBLE]);
    return UA_STATUSCODE_GOOD;
}

// callback routine for writing the current value
UA_StatusCode writeCurrent(void *handle, const UA_NodeId nodeid,
            const UA_Variant *data, const UA_NumericRange *range) {
    // handle is supposed to point to CurrentReadback
    if(UA_Variant_isScalar(data) && data->type == &UA_TYPES[UA_TYPES_DOUBLE] && data->data) {
        *(UA_Double*)handle = *(UA_Double*)data->data;
    }
    // send request to server
    sprintf(command,"MWI:%9.6f\r\n",*(double *)handle);
    // UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, command);
    TcpSendReceive();
    return UA_STATUSCODE_GOOD;
}

// callback routine for reading the current setpoint
// this only works if the output is on, otherwise #NAK:13 is answered
UA_StatusCode readCurrentSetpoint( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // send request to server    
    strcpy(command,"MWI:?\r\n");
    unsigned int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MWI:
    if(strncmp(response,"#MWI:",5)==0)
        sscanf(response+5,"%lf",(double *)handle);
    else
        *(double *)handle = 999.999;
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)handle, &UA_TYPES[UA_TYPES_DOUBLE]);
    return UA_STATUSCODE_GOOD;
}

// callback routine for reading the voltage value
UA_StatusCode readVoltage( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // handle is supposed to point to VoltageReadback
    // send request to server    
    strcpy(command,"MRV\r\n");
    unsigned int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MRV:
    if(strncmp(response,"#MRV:",5)==0)
        sscanf(response+5,"%lf",(double *)handle);
    else
        *(double *)handle = 999.999;
    int result = sscanf(response+5,"%lf",(double *)handle);
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)handle, &UA_TYPES[UA_TYPES_DOUBLE]);
    return UA_STATUSCODE_GOOD;
}

// callback routine for writing the voltage value
UA_StatusCode writeVoltage(void *handle, const UA_NodeId nodeid,
            const UA_Variant *data, const UA_NumericRange *range) {
    // handle is supposed to point to VoltageReadback
    if(UA_Variant_isScalar(data) && data->type == &UA_TYPES[UA_TYPES_DOUBLE] && data->data) {
        *(UA_Double*)handle = *(UA_Double*)data->data;
    }
    // send request to server
    sprintf(command,"MWV:%9.6f\r\n",*(double *)handle);
    // UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, command);
    TcpSendReceive();
    return UA_STATUSCODE_GOOD;
}

// callback routine for reading the voltage setpoint
// this only works if the output is on, otherwise #NAK:13 is answered
UA_StatusCode readVoltageSetpoint( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // send request to server    
    strcpy(command,"MWV:?\r\n");
    unsigned int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MWV:
    if(strncmp(response,"#MWV:",5)==0)
        sscanf(response+5,"%lf",(double *)handle);
    else
        *(double *)handle = 999.999;
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)handle, &UA_TYPES[UA_TYPES_DOUBLE]);
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode readRegister( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // handle is supposed to point to the (unsigned short) register number
    unsigned short index = *((unsigned short *)handle);
    double value;
    sprintf(command,"MRG:%d\r\n",index);
    // UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, command);
    unsigned int reclen = TcpSendReceive();
    // TODO: check reclen
    // convert buffer to numerical value
    // first 8 charecters are #MRG:??:
    int result = sscanf(response+8,"%lf",&value);
    // TODO: check result
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)&value, &UA_TYPES[UA_TYPES_DOUBLE]);
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode writeRegister(void *handle, const UA_NodeId nodeid,
            const UA_Variant *data, const UA_NumericRange *range) {
    // handle is supposed to point to the (unsigned short) register number
    unsigned short index = *((unsigned short *)handle);
    double value;
    if(UA_Variant_isScalar(data) && data->type == &UA_TYPES[UA_TYPES_DOUBLE] && data->data) {
        value = *(double *)data->data;
        // register writes are logged
        sprintf(command,"MWG:%d:%lf",index,value);
        UA_LOG_INFO(logger, UA_LOGCATEGORY_USERLAND, command);
        // send request to server
        sprintf(command,"MWG:%d:%lf\r\n",index,value);
        TcpSendReceive();
        printf("MWG response : %s",response);
    }
    return UA_STATUSCODE_GOOD;
}

/***********************************/
/* generic read/write methods      */
/* for server-internal variables   */
/***********************************/

UA_StatusCode readBoolean( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
        const UA_NumericRange *range, UA_DataValue *dataValue) {
    // set the variable value
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Boolean*)handle, &UA_TYPES[UA_TYPES_BOOLEAN]);
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode readUInt64( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
            const UA_NumericRange *range, UA_DataValue *dataValue) {
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_UInt64*)handle, &UA_TYPES[UA_TYPES_UINT64]);
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode readDouble( void *handle, const UA_NodeId nodeid, UA_Boolean sourceTimeStamp,
            const UA_NumericRange *range, UA_DataValue *dataValue) {
    dataValue->hasValue = true;
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)handle, &UA_TYPES[UA_TYPES_DOUBLE]);
    return UA_STATUSCODE_GOOD;
}

UA_StatusCode writeDouble(void *handle, const UA_NodeId nodeid,
            const UA_Variant *data, const UA_NumericRange *range) {
    if(UA_Variant_isScalar(data) && data->type == &UA_TYPES[UA_TYPES_DOUBLE] && data->data) {
        *(UA_Double*)handle = *(UA_Double*)data->data;
    }
    return UA_STATUSCODE_GOOD;
}

/***********************************/
/* main program                    */
/***********************************/

int main(int argc, char *argv[])
{

    // server will be running until we receive a SIGINT or SIGTERM
    signal(SIGINT,  stopHandler);
    signal(SIGTERM, stopHandler);

    //***********************************
    // parse configuration XML-file
    //***********************************
    LIBXML_TEST_VERSION                 // initialize the XML library and check potential ABI mismatches
    char buf[80];                       // buffer for reading strings
    int buflen;                         // number of valid characters in the buffer
    xmlDocPtr doc;                      // the resulting document tree
    doc = xmlReadFile("/etc/opcua.xml", NULL, 0);
    if (doc == NULL)
        Die("OpcUaServer : Failed to parse XML config file\n");
    // get the root element node
    xmlNode *rootNode = xmlDocGetRootElement(doc);
    // get the conficuration root node (top-level)
    xmlNode *configurationNode = NULL;
    for (xmlNode *currNode = rootNode; currNode; currNode = currNode->next)
        if (currNode->type == XML_ELEMENT_NODE)
            if (! strcmp(currNode->name, "configuration"))
                configurationNode = currNode;
    if (configurationNode == NULL)
        Die("OpcUaServer : Failed to find XML <configuration> root node\n");
    // find the opcua node
    xmlNode *opcuaNode = NULL;
    for (xmlNode *currNode = configurationNode->children; currNode; currNode = currNode->next)
        if (currNode->type == XML_ELEMENT_NODE)
            if (! strcmp(currNode->name, "opcua"))
                opcuaNode = currNode;
    if (opcuaNode == NULL)
        Die("OpcUaServer : Failed to find XML <opcua> node\n");
    // read the port number
    xmlChar *portProp = xmlGetProp(opcuaNode,"port");
    buflen = xmlStrPrintf(buf, 80, "%s", portProp);
    if (buflen == 0)
        Die("OpcUaServer : Failed to read XML <opcua> port property\n");
    buf[buflen] = '\0';         // string termination
    if (sscanf(buf,"%d",&serverPortNumber)<1)
        Die("OpcUaServer : Failed to interpret <opcua> port property\n");
    printf("OpcUaServer : OPC-UA port=%d\n", serverPortNumber);
    // find the device node
    xmlNode *deviceNode = NULL;
    for (xmlNode *currNode = configurationNode->children; currNode; currNode = currNode->next)
        if (currNode->type == XML_ELEMENT_NODE)
            if (! strcmp(currNode->name, "device"))
                deviceNode = currNode;
    if (deviceNode == NULL)
        Die("OpcUaServer : Failed to find XML <device> node\n");
    // read the device name
    xmlChar *devicenameProp = xmlGetProp(deviceNode,"name");
    buflen = xmlStrPrintf(buf, 80, "%s", devicenameProp);
    if (buflen == 0)
        Die("OpcUaServer : Failed to read XML <opcua/device> name property\n");
    buf[buflen] = '\0';         // string termination
    printf("OpcUaServer : DeviceName=%s\n", buf);
    UA_String DeviceName = UA_STRING(buf);
    // find the parameters node
    xmlNode *parametersNode = NULL;
    for (xmlNode *currNode = configurationNode->children; currNode; currNode = currNode->next)
        if (currNode->type == XML_ELEMENT_NODE)
            if (! strcmp(currNode->name, "parameters"))
                parametersNode = currNode;
    if (parametersNode == NULL)
        Die("OpcUaServer : Failed to find XML <parameters> node\n");

    //***********************************
    // connect to the internal TCP/IP server
    //***********************************
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        Die("ERROR : Failed to create socket");
    UA_LOG_INFO(logger, UA_LOGCATEGORY_NETWORK, "TCP/IP socket opened.");
    // Construct the server sockaddr_in structure
    memset(&tcpserver, 0, sizeof(tcpserver));			   // clear struct
    tcpserver.sin_family = AF_INET;				           // Internet/IP
    tcpserver.sin_addr.s_addr = inet_addr("127.0.0.1");	   // IP address
    tcpserver.sin_port = htons(10001);				       // server port
    // Establish connection
    if (connect(sock, (struct sockaddr *) &tcpserver, sizeof(tcpserver)) < 0)
        Die("ERROR : Failed to connect to TCP/IP server");
    UA_LOG_INFO(logger, UA_LOGCATEGORY_NETWORK, "Connected to internal TCP/IP server.");

    //***********************************
    // configure the UA server
    //***********************************
    UA_ServerConfig config = UA_ServerConfig_standard;
    UA_ServerNetworkLayer nl = UA_ServerNetworkLayerTCP(UA_ConnectionConfig_standard, serverPortNumber);
    config.logger = logger;
    config.networkLayers = &nl;
    config.networkLayersSize = 1;
    server = UA_Server_new(config);

    UA_ObjectAttributes object_attr;   // attributes for folders
    UA_VariableAttributes attr;        // attributes for variable nodes

    /**************************
    Device
    |   Name
    |   Status
    |   OutputOn
    |   MReset
    |   SFP-upmode
    **************************/

    // create the folder
    UA_ObjectAttributes_init(&object_attr);
    object_attr.description = UA_LOCALIZEDTEXT("en_US","Device");
    object_attr.displayName = UA_LOCALIZEDTEXT("en_US","Device");
    UA_NodeId DeviceFolder;
    UA_Server_addObjectNode(server,                                        // UA_Server *server
                            UA_NODEID_NUMERIC(1, 0),                       // UA_NodeId requestedNewNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),  // UA_NodeId parentNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),      // UA_NodeId referenceTypeId
                            UA_QUALIFIEDNAME(1, "Device"),                 // UA_QualifiedName browseName
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),     // UA_NodeId typeDefinition
                            object_attr,                                   // UA_ObjectAttributes attr
                            NULL,                                          // UA_InstantiationCallback *instantiationCallback
                            &DeviceFolder);                                // UA_NodeId *outNewNodeId

    // create the DeviceName variable
    // read-only value defined in the configuration file
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","device name");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","DeviceName");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_Variant_setScalarCopy(&attr.value, &DeviceName, &UA_TYPES[UA_TYPES_STRING]);
    UA_Server_addVariableNode(server,                                       // UA_Server *server
                              UA_NODEID_NUMERIC(1, 0),                      // UA_NodeId requestedNewNodeId
                              DeviceFolder,                                 // UA_NodeId parentNodeId
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),  // UA_NodeId referenceTypeId
                              UA_QUALIFIEDNAME(1, "DeviceName"),            // UA_QualifiedName browseName
                              UA_NODEID_NULL,                               // UA_NodeId typeDefinition
                              attr,                                         // UA_VariableAttributes attr
                              NULL,                                         // UA_InstantiationCallback *instantiationCallback
                              NULL);                                        // UA_NodeId *outNewNodeId

    // create the DeviceStatus variable
    // read-only
    unsigned int DeviceStatus = 0;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","power supply internal status");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","DeviceStatus");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_DataSource DeviceStatusDataSource = (UA_DataSource)
        {
            .handle = &DeviceStatus,
            .read = readDeviceStatus,
            .write = 0
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "DeviceStatus"),
            UA_NODEID_NULL,
            attr,
            DeviceStatusDataSource,
            NULL);

    // writing OutputOn as true switches on the device power output
    // reading returns the value obtained from the status word
    bool DeviceOutputOn = 0;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","on/off state of the device output");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","OutputOn");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource OutputOnDataSource = (UA_DataSource)
        {
            .handle = &DeviceOutputOn,
            .read = readDeviceOutputOn,
            .write = writeDeviceOutputOn
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "OutputOn"),
            UA_NODEID_NULL,
            attr,
            OutputOnDataSource,
            NULL);

    // create the Reset variable
    // boolean value - writing true performs the reset
    // read will always return false
    // TODO: the value can actually become true - wrong!
    bool DeviceMResetValue = false;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","reset the module status register");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","MReset");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_DataSource DeviceMResetDataSource = (UA_DataSource)
        {
            .handle = &DeviceMResetValue,
            .read = readBoolean,
            .write = writeMReset
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "MReset"),
            UA_NODEID_NULL,
            attr,
            DeviceMResetDataSource,
            NULL);

    // writing SFP-upmode as true switches to setpoint input from the SFP port
    // setting it to false switches back to normal mode of operation
    bool UpmodeSFP = 0;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","on/off state of the SFP setpoint input");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","SFP-upmode");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource SFPmodeDataSource = (UA_DataSource)
        {
            .handle = &UpmodeSFP,
            .read = readDeviceModeSFP,
            .write = writeDeviceModeSFP
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "SFP-upmode"),
            UA_NODEID_NULL,
            attr,
            SFPmodeDataSource,
            NULL);

    /**************************
    SetPoint
    |   Voltage
    |   Current
    |   VOltageSetpoint
    |   CurrentSetpoint
    **************************/

    UA_ObjectAttributes_init(&object_attr);
    object_attr.description = UA_LOCALIZEDTEXT("en_US","output settings");
    object_attr.displayName = UA_LOCALIZEDTEXT("en_US","SetPoint");
    UA_NodeId SetPointFolder;
    UA_Server_addObjectNode(server,                                        // UA_Server *server
                            UA_NODEID_NUMERIC(1, 0),                       // UA_NodeId requestedNewNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),  // UA_NodeId parentNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),      // UA_NodeId referenceTypeId
                            UA_QUALIFIEDNAME(1, "SetPoint"),               // UA_QualifiedName browseName
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),     // UA_NodeId typeDefinition
                            object_attr,                                   // UA_ObjectAttributes attr
                            NULL,                                          // UA_InstantiationCallback *instantiationCallback
                            &SetPointFolder);                                // UA_NodeId *outNewNodeId

    double VoltageReadback = 0.0;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","voltage readback [V]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","Voltage");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_DataSource VoltageDataSource = (UA_DataSource)
        {
            .handle = &VoltageReadback,
            .read = readVoltage,
            .write = 0
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "Voltage"),
            UA_NODEID_NULL,
            attr,
            VoltageDataSource,
            NULL);

    double CurrentReadback = 0.0;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","current readback [A]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","Current");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_DataSource CurrentDataSource = (UA_DataSource)
        {
            .handle = &CurrentReadback,
            .read = readCurrent,
            .write = 0
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "Current"),
            UA_NODEID_NULL,
            attr,
            CurrentDataSource,
            NULL);

    // when the setpoint is written, the voltage setting in the device is updated
    // (the special writeVoltage() callback is used for that)
    // reading the setpoint returns the active setpoint value
    // as read from the device
    // (the special readVoltageSetpoint() callback is used for that)
    double VoltageSetpoint = 0.0;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","voltage setpoint [V]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","VoltageSetpoint");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource VoltageSetpointDataSource = (UA_DataSource)
        {
            .handle = &VoltageSetpoint,
            .read = readVoltageSetpoint,
            .write = writeVoltage
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "VoltageSetpoint"),
            UA_NODEID_NULL,
            attr,
            VoltageSetpointDataSource,
            NULL);

    // when the setpoint is written, the current setting in the device is updated
    // (the special writeCurrent() callback is used for that)
    // reading the setpoint returns the active setpoint value
    // as read from the device
    double CurrentSetpoint = 0.0;
    UA_VariableAttributes_init(&attr);
    attr.description = UA_LOCALIZEDTEXT("en_US","current setpoint [A]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","CurrentSetpoint");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource CurrentSetpointDataSource = (UA_DataSource)
        {
            .handle = &CurrentSetpoint,
            .read = readCurrentSetpoint,
            .write = writeCurrent
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
            UA_QUALIFIEDNAME(1, "CurrentSetpoint"),
            UA_NODEID_NULL,
            attr,
            CurrentSetpointDataSource,
            NULL);

    /**************************
    Parameters
    |   define OPCUA variables for configuration registers
    |   all parameters are listed in opcua.xml
    **************************/

    UA_ObjectAttributes_init(&object_attr);
    object_attr.description = UA_LOCALIZEDTEXT("en_US","parameter settings");
    object_attr.displayName = UA_LOCALIZEDTEXT("en_US","Registers");
    UA_NodeId RegistersFolder;
    UA_Server_addObjectNode(server,                                        // UA_Server *server
                            UA_NODEID_NUMERIC(1, 0),                       // UA_NodeId requestedNewNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),  // UA_NodeId parentNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),      // UA_NodeId referenceTypeId
                            UA_QUALIFIEDNAME(1, "Registers"),              // UA_QualifiedName browseName
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),     // UA_NodeId typeDefinition
                            object_attr,                                   // UA_ObjectAttributes attr
                            NULL,                                          // UA_InstantiationCallback *instantiationCallback
                            &RegistersFolder);                             // UA_NodeId *outNewNodeId

    // here we store the register numbers
    unsigned short RegNr[maxreg];
    // these are structs for acessing the data
    UA_DataSource RegDS[maxreg];
    int loopindex=0;
    for (xmlNode *currNode = parametersNode->children; currNode; currNode = currNode->next)
        if (currNode->type == XML_ELEMENT_NODE)
            if (! strcmp(currNode->name, "register"))
            {
                // set the variable attributes as they are read from the config file
                UA_VariableAttributes_init(&attr);
                // first the register number
                unsigned short regNumber;
                xmlChar *numberProp = xmlGetProp(currNode,"number");
                buflen = xmlStrPrintf(buf, 80, "%s", numberProp);
                if (buflen == 0)
                    Die("OpcUaServer : Failed to read XML <register> number property\n");
                buf[buflen] = '\0';         // string termination
                if (sscanf(buf,"%d",&regNumber)<1)
                    Die("OpcUaServer : Failed to interpret <register> number property\n");
                RegNr[loopindex] = regNumber;
                // second the node name
                char nodeName[80];
                xmlChar *nameProp = xmlGetProp(currNode,"name");
                buflen = xmlStrPrintf(buf, 80, "%s", nameProp);
                if (buflen == 0)
                    Die("OpcUaServer : Failed to read XML <register> name property\n");
                buf[buflen] = '\0';         // string termination
                strcpy(nodeName,buf);
                printf("OpcUaServer : Register=%d %s\n", regNumber, nodeName);
                attr.displayName = UA_LOCALIZEDTEXT("en_US",nodeName);
                // thirdd the node description
                xmlChar *descProp = xmlGetProp(currNode,"description");
                buflen = xmlStrPrintf(buf, 80, "%s", descProp);
                if (buflen == 0)
                    Die("OpcUaServer : Failed to read XML <register> description property\n");
                buf[buflen] = '\0';         // string termination
                attr.description = UA_LOCALIZEDTEXT("en_US",buf);
                attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
                // get at pointer to the datasource storage
                UA_DataSource *ds = RegDS+loopindex;
                // the handle points to the register number
                ds->handle = RegNr+loopindex;
                // we have special routines for reading/writing registers
                ds->read = readRegister;
                ds->write = writeRegister;
                UA_Server_addDataSourceVariableNode(
                        server,
                        UA_NODEID_NUMERIC(1, 0),
                        RegistersFolder,
                        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                        UA_QUALIFIEDNAME(1, nodeName),
                        UA_NODEID_NULL,
                        attr,
                        *ds,
                        NULL);
                loopindex++;
                if (loopindex>=maxreg)
                    Die("OpcUaServer : too many registers\n");
            };

    // done with the XML document
    xmlFreeDoc(doc);
    xmlCleanupParser();

    // run the server (forever unless stopped with ctrl-C)
    UA_StatusCode retval = UA_Server_run(server, &running);

    // the server has stopped running
    UA_LOG_INFO(logger, UA_LOGCATEGORY_SERVER, "server stopped running.");
    UA_Server_delete(server);
    nl.deleteMembers(&nl);

    close(sock);

    printf("OpcUaServer : graceful exit\n");
    return 0;

}
