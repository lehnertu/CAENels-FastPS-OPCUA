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
 *  - $CXX -o opcuaserver OpcUaServer.o open62541.o $SDKTARGETSYSROOT/usr/lib/libxml2.a -lpthread
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
 *  - first register number is 0 (instead of 31) when read/write access occurs
 *  - server hangs after register write
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
static UA_Server *server;
static unsigned short serverPortNumber;

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
static volatile UA_Boolean running = true;

/***********************************/
/* interrupt and error handling    */
/***********************************/

// print error message and abort the running program
static void Die(char *mess)
{
    UA_LOG_FATAL(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, mess);
    exit(1);
}

// handle SIGINT und SIGTERM
static void stopHandler(int signal)
{
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c");
    running = 0;
}

/***********************************/
/* TCP/IP communication            */
/***********************************/

static int sock;
static struct sockaddr_in tcpserver;

#define BUFSIZE 80
static char command[BUFSIZE];			// command string buffer
static char response[BUFSIZE];			// receive buffer

// send the string in command to the device
// receive the answer in response
// return the number of read characters
static int TcpSendReceive() {
    unsigned int buflen = strlen(command);
	// printf("send %d bytes to socket %d\n",buflen,sock);
    if (send(sock, command, buflen, 0) != buflen)
        Die("Mismatch in number of sent bytes");
    // receive the answer from the server
    int reclen = recv(sock, response, BUFSIZE-1, 0);
	// printf("received %d bytes from socket %d\n",reclen,sock);
	if (reclen>0) response[reclen] = '\0';			// assure null terminated string
    return reclen;
}

/***********************************/
/* specialized read/write methods  */
/* for PS specific variables       */
/***********************************/

// switch the output on/off
static UA_StatusCode writeDeviceOutputOn(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(data->hasValue && UA_Variant_isScalar(&data->value) && (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) && (data->value.data != 0))
    {
        bool val = *(bool*)data->value.data;
        
        if(val)
        {
            // switch on the output
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "MON");
            strcpy(command,"MON\r\n");
            TcpSendReceive();
        }
        else
        {
            // switch off the output
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "MOFF");
            strcpy(command,"MOFF\r\n");
            TcpSendReceive();
        }
        return UA_STATUSCODE_GOOD;
    }
    else
    {
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid data for writeDeviceOutputOn()");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

static UA_StatusCode readDeviceOutputOn(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    // send status request to server    
    strcpy(command,"MST\r\n");
    int reclen = TcpSendReceive();
    // convert the answer into a number
    // first 5 charecters are #MST:
    unsigned int status;
    sscanf(response+5,"%x",&status);
    bool val = (status & 1 == 1);
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Boolean*)(&val), &UA_TYPES[UA_TYPES_BOOLEAN]);
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode readDeviceStatus(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    // send request to server    
    strcpy(command,"MST\r\n");
    int reclen = TcpSendReceive();
    // convert the answer into a number
    // first 5 charecters are #MST:
    unsigned int status;
    int result = sscanf(response+5,"%x",&status);
    if (result==0)
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    else
	{
        UA_Variant_setScalarCopy(&dataValue->value, (UA_UInt32*)(&status), &UA_TYPES[UA_TYPES_UINT32]);
        dataValue->hasValue = true;
        return UA_STATUSCODE_GOOD;
	}
}

// write an MRESET command when set to true
static UA_StatusCode writeMReset(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(data->hasValue && UA_Variant_isScalar(&data->value) && (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) && (data->value.data != 0))
    {
        bool val = *(bool*)data->value.data;
        if(val)
        {
		    // report to log
		    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "MRESET");
		    // send command
		    strcpy(command,"MRESET\r\n");
		    TcpSendReceive();
        }
		return UA_STATUSCODE_GOOD;
    }
    else
    {
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid data for writeMReset()");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// switch to SFP update mode and back
static UA_StatusCode writeDeviceModeSFP(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(data->hasValue && UA_Variant_isScalar(&data->value) && (data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN]) && (data->value.data != 0))
    {
        bool val = *(bool*)data->value.data;
        if(val)
        {
            // switch to te fast data interface
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "UPMODE:SFP");
            strcpy(command,"UPMODE:SFP\r\n");
            TcpSendReceive();
        }
        else
        {
            // switch off the fast data interface
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "UPMODE:NORMAL");
            strcpy(command,"UPMODE:NORMAL\r\n");
            TcpSendReceive();
        }
        return UA_STATUSCODE_GOOD;
    }
    else
    {
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid data for writeDeviceModeSFP()");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// read the SFP output mode
static UA_StatusCode readDeviceModeSFP(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    // send status request to server    
    strcpy(command,"UPMODE\r\n");
    int reclen = TcpSendReceive();
    // first 8 charecters are #UPMODE:
    bool val = (strncmp(response+8,"SFP",3)==0);
    // set the variable value
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Boolean*)(&val), &UA_TYPES[UA_TYPES_BOOLEAN]);
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}

// callback routine for reading the current value
static UA_StatusCode readCurrent(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    double val;
    // send request to server    
    strcpy(command,"MRI\r\n");
    int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MRI:
    if(strncmp(response,"#MRI:",5)==0)
    {
        sscanf(response+5,"%lf",&val);
        // set the variable value
        UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)(&val), &UA_TYPES[UA_TYPES_DOUBLE]);
        dataValue->hasValue = true;
        return UA_STATUSCODE_GOOD;
    }
    else
    {
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid MRI response");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// callback routine for reading the voltage value
static UA_StatusCode readVoltage(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    double val;
    // send request to server    
    strcpy(command,"MRV\r\n");
    int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MRV:
    if(strncmp(response,"#MRV:",5)==0)
    {
        sscanf(response+5,"%lf",&val);
        // set the variable value
        UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)(&val), &UA_TYPES[UA_TYPES_DOUBLE]);
        dataValue->hasValue = true;
        return UA_STATUSCODE_GOOD;
    }
    else
    {
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid MRV response");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// callback routine for writing the current setpoint
static UA_StatusCode writeCurrentSetpoint(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(data->hasValue && UA_Variant_isScalar(&data->value) && (data->value.type == &UA_TYPES[UA_TYPES_DOUBLE]) && (data->value.data != 0))
    {
        double val = *(double*)data->value.data;
        // send request to server
        snprintf(command,BUFSIZE,"MWI:%9.6f\r\n",val);
        TcpSendReceive();
        return UA_STATUSCODE_GOOD;
    }
    else
    {
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid data for writeCurrent()");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// callback routine for reading the current setpoint
static UA_StatusCode readCurrentSetpoint(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    double val;
    
    // send request to server    
    strcpy(command,"MWI:?\r\n");
    int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MWI:
    if(strncmp(response,"#MWI:",5)==0)
    {
        sscanf(response+5,"%lf",&val);
        // set the variable value
        UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)(&val), &UA_TYPES[UA_TYPES_DOUBLE]);
        dataValue->hasValue = true;
        return UA_STATUSCODE_GOOD;
    }
    else
    {
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid MWI:? response");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// callback routine for writing the voltage setpoint
static UA_StatusCode writeVoltageSetpoint(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(data->hasValue && UA_Variant_isScalar(&data->value) && (data->value.type == &UA_TYPES[UA_TYPES_DOUBLE]) && (data->value.data != 0))
    {
        double val = *(double*)data->value.data;
        // send request to server
        snprintf(command,BUFSIZE,"MWV:%9.6f\r\n",val);
        TcpSendReceive();
        return UA_STATUSCODE_GOOD;
    }
    else
    {
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid data for writeVoltage()");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// callback routine for reading the voltage setpoint
static UA_StatusCode readVoltageSetpoint(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    double val;
    
    // send request to server    
    strcpy(command,"MWV:?\r\n");
    int reclen = TcpSendReceive();
    // convert buffer to numerical value
    // first 5 charecters are #MWV:
    if(strncmp(response,"#MWV:",5)==0)
    {
        sscanf(response+5,"%lf",&val);
        // set the variable value
        UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)(&val), &UA_TYPES[UA_TYPES_DOUBLE]);
        dataValue->hasValue = true;
        return UA_STATUSCODE_GOOD;
    }
    else
    {
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid MWV:? response");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// callback routine for reading registers
static UA_StatusCode readRegister(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    double val;
    // the node context is supposed to point to the (unsigned short) register number
    unsigned short index = *((unsigned short *)nodeContext);
    // send request to server    
    snprintf(command,BUFSIZE,"MRG:%d\r\n",index);
    // register reads are logged
    // UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, command);
    int reclen = TcpSendReceive();
    // TODO: check reclen
    // UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, response);
    // convert buffer to numerical value
    // first 8 charecters are #MRG:??:
    if(strncmp(response,"#MRG:",5)==0)
    {
        // this only works for 2-digit register numbers
        int result = sscanf(response+8,"%lf",&val);
        // TODO: check result
        // set the variable value
        UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)(&val), &UA_TYPES[UA_TYPES_DOUBLE]);
        dataValue->hasValue = true;
        return UA_STATUSCODE_GOOD;
    }
    else
    {
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid MRG response");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

// callback routine for writing registers
static UA_StatusCode writeRegister(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(data->hasValue && UA_Variant_isScalar(&data->value) && (data->value.type == &UA_TYPES[UA_TYPES_DOUBLE]) && (data->value.data != 0))
    {
        double val = *(double*)data->value.data;
        // the node context is supposed to point to the (unsigned short) register number
        unsigned short index = *((unsigned short *)nodeContext);
        // send request to server
        snprintf(command,BUFSIZE,"MWG:%d:%lf\r\n",index,val);
	    // register writes are logged
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, command);
		// TcpSendReceive() called from here fails - everywhere else it works fine
        int reclen = TcpSendReceive();
		// typical response is #AK
		if (reclen>3)
		{
		    // UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, response);
		    return UA_STATUSCODE_GOOD;
		}
		else
		{
			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, response);
			return UA_STATUSCODE_BADCOMMUNICATIONERROR;
		}
    }
    else
    {
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "invalid data for writeRegister()");
        return UA_STATUSCODE_UNCERTAINNOCOMMUNICATIONLASTUSABLEVALUE;
    }
}

/***********************************/
/* generic read/write methods      */
/* for server-internal variables   */
/***********************************/

static UA_StatusCode readBoolean(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Boolean*)nodeContext, &UA_TYPES[UA_TYPES_BOOLEAN]);
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}

// not used - code left here for potential future use
static UA_StatusCode writeBoolean(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(UA_Variant_isScalar(&(data->value)) && data->value.type == &UA_TYPES[UA_TYPES_BOOLEAN] && data->value.data){
        *(UA_Boolean*)nodeContext = *(UA_Boolean*)data->value.data;
    }
    return UA_STATUSCODE_GOOD;
}

// not used - code left here for potential future use
static UA_StatusCode readDouble(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    UA_Boolean sourceTimeStamp,
    const UA_NumericRange *range,
    UA_DataValue *dataValue)
{
    UA_Variant_setScalarCopy(&dataValue->value, (UA_Double*)nodeContext, &UA_TYPES[UA_TYPES_DOUBLE]);
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}

// not used - code left here for potential future use
static UA_StatusCode writeDouble(
    UA_Server *server,
    const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *nodeId, void *nodeContext,
    const UA_NumericRange *range,
    const UA_DataValue *data)
{
    if(UA_Variant_isScalar(&(data->value)) && data->value.type == &UA_TYPES[UA_TYPES_DOUBLE] && data->value.data){
        *(UA_Double*)nodeContext = *(UA_Double*)data->value.data;
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
    // printf("OpcUaServer : OPC-UA port=%d\n", serverPortNumber);
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
    // printf("OpcUaServer : DeviceName=%s\n", buf);
    UA_String BufString = UA_STRING(buf);
    UA_String *DeviceName = UA_String_new();
    UA_String_copy(&BufString, DeviceName);

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
	// set a timeout of 1 second for the socket
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_NETWORK, "TCP/IP socket opened.");
    // Construct the server sockaddr_in structure
    memset(&tcpserver, 0, sizeof(tcpserver));			   // clear struct
    tcpserver.sin_family = AF_INET;				           // Internet/IP
    tcpserver.sin_addr.s_addr = inet_addr("127.0.0.1");	   // IP address
    tcpserver.sin_port = htons(10001);				       // server port
    // Establish connection
    if (connect(sock, (struct sockaddr *) &tcpserver, sizeof(tcpserver)) < 0)
        Die("ERROR : Failed to connect to TCP/IP server");
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_NETWORK, "Connected to internal TCP/IP server.");

    //***********************************
    // configure the UA server
    //***********************************
    
    // configure the UA server
    UA_ServerConfig config;
    memset(&config, 0, sizeof(UA_ServerConfig));
    UA_StatusCode res = UA_ServerConfig_setMinimal(&config, serverPortNumber, NULL);
    if(res != UA_STATUSCODE_GOOD)
    {
        // printf("UA_ServerConfig_setMinimal() error %8x\n", res);
        exit(-1);
    }
    UA_Server *server = UA_Server_newWithConfig(&config);
    if(!server)
    {
        // printf("UA_Server_newWithConfig() failed\n");
        exit(-1);
    }

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
    object_attr = UA_ObjectAttributes_default;
    object_attr.description = UA_LOCALIZEDTEXT("en_US","Device");
    object_attr.displayName = UA_LOCALIZEDTEXT("en_US","Device");
    static UA_NodeId DeviceFolder;
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
    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","device name");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","DeviceName");
    attr.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_Variant_setScalarCopy(&attr.value, DeviceName, &UA_TYPES[UA_TYPES_STRING]);
    UA_Server_addVariableNode(server,                                       // UA_Server *server
                              UA_NODEID_NUMERIC(1, 0),                      // UA_NodeId requestedNewNodeId
                              DeviceFolder,                                 // UA_NodeId parentNodeId
                              UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),     // UA_NodeId referenceTypeId
                              UA_QUALIFIEDNAME(1, "DeviceName"),            // UA_QualifiedName browseName
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),  // UA_NodeId typeDefinition
                              attr,                                         // UA_VariableAttributes attr
                              NULL,                                         // const UA_DataSource dataSource
                              NULL);                                        // UA_NodeId *outNewNodeId

    // create the DeviceStatus variable
    // read-only
    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","power supply internal status");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","DeviceStatus");
    attr.dataType = UA_TYPES[UA_TYPES_UINT32].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_DataSource DeviceStatusDataSource = (UA_DataSource)
        {
            .read = readDeviceStatus,
            .write = NULL
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "DeviceStatus"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            DeviceStatusDataSource,
            NULL, NULL);

    // writing OutputOn as true switches on the device power output
    // reading returns the value obtained from the status word
    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","on/off state of the device output");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","OutputOn");
    attr.dataType = UA_TYPES[UA_TYPES_BOOLEAN].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource OutputOnDataSource = (UA_DataSource)
        {
            .read = readDeviceOutputOn,
            .write = writeDeviceOutputOn
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "OutputOn"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            OutputOnDataSource,
            NULL, NULL);

    // create the Reset variable
    // boolean value - writing true performs the reset
    // read will always return false
    bool DeviceMResetValue = false;
    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","reset the module status register");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","MReset");
    attr.dataType = UA_TYPES[UA_TYPES_BOOLEAN].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource DeviceMResetDataSource = (UA_DataSource)
        {
            .read = readBoolean,
            .write = writeMReset
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "MReset"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            DeviceMResetDataSource,
            &DeviceMResetValue, NULL);

    // writing SFP-upmode as true switches to setpoint input from the SFP port
    // setting it to false switches back to normal mode of operation
    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","on/off state of the SFP setpoint input");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","SFP-upmode");
    attr.dataType = UA_TYPES[UA_TYPES_BOOLEAN].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource SFPmodeDataSource = (UA_DataSource)
        {
            .read = readDeviceModeSFP,
            .write = writeDeviceModeSFP
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            DeviceFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "SFP-upmode"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            SFPmodeDataSource,
            NULL, NULL);
    
    /**************************
    SetPoint
    |   Voltage
    |   Current
    |   VoltageSetpoint
    |   CurrentSetpoint
    **************************/

    // create the folder
    object_attr = UA_ObjectAttributes_default;
    object_attr.description = UA_LOCALIZEDTEXT("en_US","output settings");
    object_attr.displayName = UA_LOCALIZEDTEXT("en_US","SetPoint");
    static UA_NodeId SetPointFolder;
    UA_Server_addObjectNode(server,                                        // UA_Server *server
                            UA_NODEID_NUMERIC(1, 0),                       // UA_NodeId requestedNewNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),  // UA_NodeId parentNodeId
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),      // UA_NodeId referenceTypeId
                            UA_QUALIFIEDNAME(1, "SetPoint"),               // UA_QualifiedName browseName
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),     // UA_NodeId typeDefinition
                            object_attr,                                   // UA_ObjectAttributes attr
                            NULL,                                          // UA_InstantiationCallback *instantiationCallback
                            &SetPointFolder);                              // UA_NodeId *outNewNodeId

    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","current readback [A]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","Current");
    attr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_DataSource CurrentDataSource = (UA_DataSource)
        {
            .read = readCurrent,
            .write = 0
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "Current"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            CurrentDataSource,
            NULL, NULL);

    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","voltage readback [V]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","Voltage");
    attr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_DataSource VoltageDataSource = (UA_DataSource)
        {
            .read = readVoltage,
            .write = 0
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "Voltage"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            VoltageDataSource,
            NULL, NULL);

    // when the setpoint is written, the voltage setting in the device is updated
    // (the special writeVoltageSetpoint() callback is used for that)
    // reading the setpoint returns the active setpoint value reported by the device
    // (the special readVoltageSetpoint() callback is used for that)
    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","voltage setpoint [V]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","VoltageSetpoint");
    attr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource VoltageSetpointDataSource = (UA_DataSource)
        {
            .read = readVoltageSetpoint,
            .write = writeVoltageSetpoint
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "VoltageSetpoint"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            VoltageSetpointDataSource,
            NULL, NULL);

    // when the setpoint is written, the current setting in the device is updated
    // (the special writeCurrentSetpoint() callback is used for that)
    // reading the setpoint returns the active setpoint value  reported by the device
    // (the special readCurrentSetpoint() callback is used for that)
    attr = UA_VariableAttributes_default;
    attr.description = UA_LOCALIZEDTEXT("en_US","current setpoint [A]");
    attr.displayName = UA_LOCALIZEDTEXT("en_US","CurrentSetpoint");
    attr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
	attr.valueRank = UA_VALUERANK_SCALAR;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_DataSource CurrentSetpointDataSource = (UA_DataSource)
        {
            .read = readCurrentSetpoint,
            .write = writeCurrentSetpoint
        };
    UA_Server_addDataSourceVariableNode(
            server,
            UA_NODEID_NUMERIC(1, 0),
            SetPointFolder,
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
            UA_QUALIFIEDNAME(1, "CurrentSetpoint"),
            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
            attr,
            CurrentSetpointDataSource,
            NULL, NULL);
    
    /**************************
    Parameters
    |   define OPCUA variables for configuration registers
    |   all parameters are listed in opcua.xml
    **************************/

    // create the folder
    object_attr = UA_ObjectAttributes_default;
    object_attr.description = UA_LOCALIZEDTEXT("en_US","parameter settings");
    object_attr.displayName = UA_LOCALIZEDTEXT("en_US","Registers");
    static UA_NodeId RegistersFolder;
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
    // the register node context will point to this storage
    static unsigned short RegNr[maxreg];

    // unclear why - the TCP/IP communication crashs when attempting a register write
	// if writeRegister is replace with writeCurrentSetpoint this works
	static UA_DataSource RegDataSource;
	RegDataSource.read = readRegister;
	RegDataSource.write = writeRegister;

	// attempt to use individual datasources for each register
	// no - doesn't work any better
	// UA_DataSource RegDS[maxreg];

    int loopindex=0;
    for (xmlNode *currNode = parametersNode->children; currNode; currNode = currNode->next)
        if (currNode->type == XML_ELEMENT_NODE)
            if (! strcmp(currNode->name, "register"))
            {
                // set the variable attributes as they are read from the config file
                // first the register number
                unsigned short regNumber;
                xmlChar *numberProp = xmlGetProp(currNode,"number");
                buflen = xmlStrPrintf(buf, 80, "%s", numberProp);
                if (buflen == 0)
                    Die("OpcUaServer : Failed to read XML <register> number property\n");
                buf[buflen] = '\0';         // string termination
                if (sscanf(buf,"%u",&regNumber)<1)
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
                // third the node description
                char description[200];
                xmlChar *descProp = xmlGetProp(currNode,"description");
                buflen = xmlStrPrintf(buf, 80, "%s", descProp);
                if (buflen == 0)
                    Die("OpcUaServer : Failed to read XML <register> description property\n");
                buf[buflen] = '\0';         // string termination
                strcpy(description,buf);
                
                printf("OpcUaServer : Register=%d %s - %s\n", regNumber, nodeName, description);
                
                attr = UA_VariableAttributes_default;
                attr.displayName = UA_LOCALIZEDTEXT("en_US",nodeName);
                attr.description = UA_LOCALIZEDTEXT("en_US",description);
                attr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
    			attr.valueRank = UA_VALUERANK_SCALAR;
                attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
                
                // add a node to the data model
                UA_Server_addDataSourceVariableNode(
                        server,
                        UA_NODEID_NUMERIC(1, 0),
                        RegistersFolder,
                        UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                        UA_QUALIFIEDNAME(1, nodeName),
                        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                        attr,
                        RegDataSource,  // *ds,
                        (void *)(RegNr+loopindex),
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
    if(retval != UA_STATUSCODE_GOOD)
        printf("UA_Server_run() error %8x\n", retval);
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "server stopped running.");
    UA_Server_delete(server);
    // nl.deleteMembers(&nl);

    close(sock);

    printf("OpcUaServer : graceful exit\n");
    return (int)retval;

}
