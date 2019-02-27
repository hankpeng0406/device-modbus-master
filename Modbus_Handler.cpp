/****************************************************************************/
/* Copyright(C) : Advantech Technologies, Inc.														 */
/* Create Date  : 2015 by Zach Chih															     */
/* Modified Date: 2016/3/9 by Zach Chih															 */
/* Abstract     : Modbus Handler                                   													*/
/* Reference    : None																									 */
/****************************************************************************/
#include "Modbus_Handler.h"
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <errno.h>
#include <float.h>
#ifdef LUA
#include <lua.hpp>
#endif
#include "wrapper.h"
#include "pthread.h"
#include "util_path.h"
#include "unistd.h"
#include "susiaccess_handler_api.h"
#include "IoTMessageGenerate.h"
#include "Modbus_HandlerLog.h"
#include "Modbus_Parser.h"
#include "ReadINI.h"
#include "version.h"
#include "description.h"

//-----------------------------------------------------------------------------
//#############################################################################
//symbol correspondence
//Code	:	Modbus Spec	in INI File
//DI	:	Discrete Inputs		-->IB
//DO	:	Coils				-->B
//AI	:	Input Registers		-->IR
//AO	:	Holding Registers	-->R
//#############################################################################
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------------------

//#define DEF_HANDLER_NAME          "Modbus_Handler"
#define DEF_MODBUS_TCP              "Modbus_TCP"
#define DEF_MODBUS_RTU              "Modbus_RTU"
#define DEF_SIMULATOR_NAME          "Simulator"
										
//Threshold
#define MODBUS_SET_THR_REP          "setThrRep"
#define MODBUS_DEL_ALL_THR_REP      "delAllThrRep"
#define MODBUS_THR_CHECK_STATUS     "thrCheckStatus"
//#define MODBUS_THR_CHECK_MSG        "thrCheckMsg"
#define MODBUS_THR_CHECK_MSG        "msg"
//#define INI_PATH                    "\Modbus.ini"
#define cagent_request_custom       2002
#define cagent_custom_action        30002
#define SET_STR_LENGTH              200
#define MAX_BLOCK_LENGTH            600

#define INFOSTRLEN	                64

/*
//----------------------------------------------Customize
#define BARCODE_NUM 15
#define BARCODE_REG_NUM 7
//----------------------------------------------Customize_end
*/

//-----------------------------------------------------------------------------
//----------------------sensor info item list function define------------------
//-----------------------------------------------------------------------------
static sensor_info_list CreateSensorInfoList();
static void DestroySensorInfoList(sensor_info_list sensorInfoList);
static int InsertSensorInfoNode(sensor_info_list sensorInfoList, sensor_info_t * pSensorInfo);
static sensor_info_node_t * FindSensorInfoNodeWithID(sensor_info_list sensorInfoList, int id);
static int DeleteSensorInfoNodeWithID(sensor_info_list sensorInfoList, int id);
static int DeleteAllSensorInfoNode(sensor_info_list sensorInfoList);
static bool IsSensorInfoListEmpty(sensor_info_list sensorInfoList);
//-----------------------------------------------------------------------------
static double LuaConversion(double inputVal, char* strLua);
bool IoTSetDeviceInfo(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_ATTRIBUTE_T* attr, int indexWDev);
bool IoTSetBits(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName,
	MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Data **wdData, uint8_t** arrBits, int numBits, bool bUDI_DO);
bool IoTSetBitsBlock(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName, 
	MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Block **wbData, uint8_t** arrBits, int numBlocks, bool bUDI_DO);
bool IoTSetRegisters(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName, 
	MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Data **wdData, uint16_t** arrRegs, int numRegs, bool bUAI_AO);
bool IoTSetRegistersBlock(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName,
	MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Block **wbData, uint16_t** arrBlocks, int numBlocks, bool bUAI_AO);
//bool MSGSetValues(MSG_ATTRIBUTE_T* attr, WISE_Data* wdData, bool bUAI_AO, int numberOfRegs, WISE_Sensor* sensors, int count, MSG_CLASSIFY_T *pSensor);
bool read_INI_Platform(char *modulePath, char *iniPath);
bool read_INI_DeviceDetail();
bool read_INI_Bits(char* title, char* key, char* filename, WISE_Data** wdData, uint8_t** arrBits, int* numBits);
bool read_INI_Registers(char* title, char* key, char* filename, WISE_Data** wdData, uint16_t** arrRegs, int *numRegs);
bool read_INI_Blocks(char* title, char* key, char* filename, WISE_Block** wbData, void** arrRegs, int* numBlocks);

//-----------------------------------------------------------------------------
// Internal Prototypes:
//-----------------------------------------------------------------------------
//
typedef struct {
	pthread_t threadHandler;
	bool isThreadRunning;
} handler_context_t;

typedef struct report_data_params_t {
	unsigned int intervalTimeMs;
	unsigned int continueTimeMs;
	char repFilter[4096];
} report_data_params_t;

static report_data_params_t AutoUploadParams;
static report_data_params_t AutoReportParams;

/*
//----------------------------------------------Customize
char ascii_array[43] = {'0','1','2','3','4','5','6','7','8','9',':',';','<','=','>','?','@','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
int barflag=0; // 0-> no barcode; 1-> AI barcode; 2->AO barcode;
//----------------------------------------------Customize_end
*/
//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------
static void* g_loghandle = NULL;

static Handler_info  g_PluginInfo;
static handler_context_t g_RetrieveContex;
static handler_context_t g_AutoReportContex;
static handler_context_t g_AutoUploadContex;
static handler_context_t g_ThresholdSetContex;
static handler_context_t g_ThresholdCheckContex;
static handler_context_t g_ThresholdDeleteContex;
//clock_t AutoUpload_start;
//clock_t AutoUpload_continue;
static HANDLER_THREAD_STATUS g_status = handler_status_no_init;
static bool g_bRetrieve = false;
static bool g_bAutoReport = false;
static bool g_bAutoUpload = false;
static time_t g_monitortime;
static HandlerSendCbf  g_sendcbf = NULL;						// Client Sends information (in JSON format) to Cloud Server	
static HandlerSendCustCbf  g_sendcustcbf = NULL;			    // Client Sends information (in JSON format) to Cloud Server with custom topic	
static HandlerSubscribeCustCbf g_subscribecustcbf = NULL;
static HandlerAutoReportCbf g_sendreportcbf = NULL;				// Client Sends report (in JSON format) to Cloud Server with AutoReport topic
static HandlerSendCapabilityCbf g_sendcapabilitycbf = NULL;
static HandlerSendEventCbf g_sendeventcbf = NULL;

static char * CurIotDataJsonStr = NULL;
//-----------------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------------
//const char strPluginName[MAX_TOPIC_LEN] = {"Modbus_Handler"};
char *strPluginName = NULL;										//be used to set the customized handler name by module_config.xml
char *strPluginVersion = NULL;
char *strPluginDescription = NULL;
const int iRequestID = cagent_request_custom;
const int iActionID = cagent_custom_action;
MSG_CLASSIFY_T *g_Capability = NULL;
bool bAllDataAlloc = false;
int Rev_Fail_Num = 0;
bool g_bRev_Fail = false;

bool bConnectionFlag = false;
bool bBlanking = false;
pthread_mutex_t pModbusMux;
pthread_mutex_t pModbusThresholdMux;
bool bIsSimtor = false;
bool bFind = false; //Find INI file
int  iTCP_RTU = 0;
char Modbus_Protocol[20] = "";
char Device_Name[20] = "";
bool Modbus_Log = false; //Log
int  Modbus_Interval = 1; //Interval
int Modbus_Delay = 100;
char *MRLog_path;
char *MSLog_path;
FILE *pMRLog = NULL;
FILE *pMSLog = NULL;
//--------Modbus_TCP
char Modbus_Clent_IP[16] = "";
int Modbus_Client_Port = 502;
//int Modbus_UnitID = 1;
//--------Modbus_RTU
char Modbus_Slave_Port[256] = "";
int Modbus_Baud = 19200;
char Modbus_Parity[5];
int Modbus_DataBits = 8;
int Modbus_StopBits = 1;
//int Modbus_SlaveID = 1;
//--------
modbus_t *ctx = NULL;

//--------Floating real number of registers
uint16_t *AI_Regs_temp_Ary;
uint16_t *AO_Regs_temp_Ary;
uint16_t *AO_Set_Regs_temp_Ary;

int numberOfAI_Regs = 0;
int numberOfAO_Regs = 0;

WISE_Device *WDev;
int numberOfWDev = 0;
char *Modbus_Devices;

time_t Start_Upload;
time_t Continue_Upload;

//-----------------------------------------------------------------------------
// AutoReport:
//-----------------------------------------------------------------------------
cJSON *Report_root = NULL;
cJSON *Stop_Report_root = NULL;
int Report_array_size = 0, Report_interval = 0;
bool Report_Reply_All = false;
char **Report_Data_paths = NULL;

cJSON *Report_item, *Report_it, *Report_js_name, *Report_first, *Report_second_interval, *Report_second, *Report_third, *Report_js_list;
//-----------------------------------------------------------------------------
// Threshold:
//-----------------------------------------------------------------------------
char *Threshold_Data = NULL;
static modbus_threshold_list MODBUSthresholdList = NULL;

//-----------------------------------------------------------------------------
// Function:
//-----------------------------------------------------------------------------
char* itobs(uint32_t n, char *ps)
{
	int size = 8 * sizeof(n);
	int i = size - 1;

	while (i + 1)
	{
		ps[i--] = (1 & n) + '0';
		n >>= 1;
	}
	ps[size] = '\0';
	return ps;
}

//fix modbus_get_float (CDAB) to custom_get_float(ABCD) work! -->sw_mode = 1
static float custom_get_float(uint16_t *src)
{
	uint16_t fix_bytes_order[] = { src[1], src[0] };
	return modbus_get_float(fix_bytes_order);
}

//fix modbus_set_float (CDAB) to custom_set_float(ABCD)	work! -->sw_mode = 1
static void custom_set_float(float fv, uint16_t *src)
{
	uint16_t temp;

	modbus_set_float(fv, src);
	temp = src[1];
	src[1] = src[0];
	src[0] = temp;
}

//fix modbus_get_float_dcba to custom_get_float_dcba(DCBA) work! -->sw_mode = 2
static float custom_get_float_dcba(uint16_t *src)
{
	uint16_t fix_bytes_order[] = { src[1], src[0] };
	//printf("%d %d\n",src[1],src[0]);
	return modbus_get_float_dcba(fix_bytes_order);
}

//fix modbus_set_float_dcba to custom_set_float_dcba(DCBA) work! -->sw_mode = 2
static void custom_set_float_dcba(float fv, uint16_t *src)
{
	uint16_t temp;

	modbus_set_float_dcba(fv, src);
	temp = src[1];
	src[1] = src[0];
	src[0] = temp;
}

//custom_get_float_badc(BADC) work!	-->sw_mode = 3
static float custom_get_float_badc(uint16_t *src)
{
	float f;
	uint32_t i;
	//char s[8 * sizeof(i) + 1];

	i = bswap_32((((uint32_t)src[1]) << 16) + src[0]);
	memcpy(&f, &i, sizeof(float));
	//printf("%d = %s\n", i, itobs(i,s));

	return f;
}

//custom_set_float_badc(BADC) work! -->sw_mode = 3
static void custom_set_float_badc(float fv, uint16_t *src)
{
	uint32_t i;

	memcpy(&i, &fv, sizeof(uint32_t));
	i = bswap_32(i);
	src[0] = (uint16_t)i;
	src[1] = (uint16_t)(i >> 16);
	//printf("%d\n",src[0]);
	//printf("%d\n",src[1]);
}

//custom_get_float_cdab(CDAB) work! -->sw_mode = 4
static float custom_get_float_cdab(uint16_t *src)
{
	float f;
	uint32_t i;
	char s[8 * sizeof(i) + 1];

	i = (((uint32_t)src[1]) << 16) + src[0];
	memcpy(&f, &i, sizeof(float));
	//printf("%d = %s\n", i, itobs(i,s));
	return f;
}

//custom_set_float_cdab(CDAB) work! -->sw_mode = 4
static void custom_set_float_cdab(float fv, uint16_t *src)
{
	uint32_t i;

	memcpy(&i, &fv, sizeof(uint32_t));
	src[0] = (uint16_t)i;
	src[1] = (uint16_t)(i >> 16);
	//printf("%d\n",src[0]);
	//printf("%d\n",src[1]);
}


//custom_get_unsigned_int(ABCD) work! -->sw_mode = 5
static uint32_t custom_get_unsigned_int(uint16_t *src)
{
	uint16_t fix_bytes_order[] = { src[1], src[0] };
	uint32_t i;
	i = (((uint32_t)src[0]) << 16) + src[1];

	return i;
}

//custom_set_unsigned_int(ABCD) work! -->sw_mode = 5
static void custom_set_unsigned_int(uint32_t uiv, uint16_t *src)
{
	uint32_t i;

	memcpy(&i, &uiv, sizeof(uint32_t));
	src[1] = (uint16_t)i;
	src[0] = (uint16_t)(i >> 16);
}

//custom_get_unsigned_int_cdab(CDAB) work! -->sw_mode = 6
static uint32_t custom_get_unsigned_int_cdab(uint16_t *src)
{
	uint16_t fix_bytes_order[] = { src[1], src[0] };
	uint32_t i;
	i = (((uint32_t)src[1]) << 16) + src[0];

	return i;
}

//custom_set_unsigned_int_cdab(CDAB) work! -->sw_mode = 6
static void custom_set_unsigned_int_cdab(uint32_t uiv, uint16_t *src)
{
	uint32_t i;

	memcpy(&i, &uiv, sizeof(uint32_t));
	src[0] = (uint16_t)i;
	src[1] = (uint16_t)(i >> 16);
}

//custom_get_int(ABCD) work! -->sw_mode = 7
static int custom_get_int(uint16_t *src)
{
	uint16_t fix_bytes_order[] = { src[1], src[0] };
	int i;
	i = (((uint32_t)src[0]) << 16) + src[1];

	return i;
}

//custom_set_int(ABCD) work! -->sw_mode = 7
static void custom_set_int(int iv, uint16_t *src)
{
	uint32_t i;

	memcpy(&i, &iv, sizeof(uint32_t));
	src[1] = (uint16_t)i;
	src[0] = (uint16_t)(i >> 16);
}

//custom_get_int_cdab(CDAB) work! -->sw_mode = 8
static int custom_get_int_cdab(uint16_t *src)
{
	uint16_t fix_bytes_order[] = { src[1], src[0] };
	int i;
	i = (((uint32_t)src[1]) << 16) + src[0];

	return i;
}

//custom_set_int_cdab(CDAB) work! -->sw_mode = 8
static void custom_set_int_cdab(int iv, uint16_t *src)
{
	uint32_t i;

	memcpy(&i, &iv, sizeof(uint32_t));
	src[0] = (uint16_t)i;
	src[1] = (uint16_t)(i >> 16);
}

void Handler_Uninitialize();
bool Modbus_Rev();
#ifdef _MSC_VER
BOOL WINAPI DllMain(HINSTANCE module_handle, DWORD reason_for_call, LPVOID reserved)
{
	if (reason_for_call == DLL_PROCESS_ATTACH) // Self-explanatory
	{
		printf("DllInitializer\r\n");
		DisableThreadLibraryCalls(module_handle); // Disable DllMain calls for DLL_THREAD_*
		if (reserved == NULL) // Dynamic load
		{
			// Initialize your stuff or whatever
			// Return FALSE if you don't want your module to be dynamically loaded
		}
		else // Static load
		{
			// Return FALSE if you don't want your module to be statically loaded
			return FALSE;
		}
	}

	if (reason_for_call == DLL_PROCESS_DETACH) // Self-explanatory
	{
		printf("DllFinalizer\r\n");
		if (reserved == NULL) // Either loading the DLL has failed or FreeLibrary was called
		{
			// Cleanup
			Handler_Uninitialize();
		}
		else // Process is terminating
		{
			// Cleanup
			Handler_Uninitialize();
		}
	}
	return TRUE;
}
#else
__attribute__((constructor))
/**
 * initializer of the shared lib.
 */
	static void Initializer(int argc, char** argv, char** envp)
{
	printf("DllInitializer\r\n");
}

__attribute__((destructor))
/**
 * It is called when shared lib is being unloaded.
 *
 */
	static void Finalizer()
{
	printf("DllFinalizer\r\n");
	Handler_Uninitialize();
}
#endif

static sensor_info_list CreateSensorInfoList()
{
	sensor_info_node_t * head = NULL;
	head = (sensor_info_node_t *)malloc(sizeof(sensor_info_node_t));
	if (head)
	{
		memset(head, 0, sizeof(sensor_info_node_t));
		head->next = NULL;
	}
	return head;
}

static void DestroySensorInfoList(sensor_info_list sensorInfoList)
{
	if (NULL == sensorInfoList) return;
	DeleteAllSensorInfoNode(sensorInfoList);
	free(sensorInfoList);
	sensorInfoList = NULL;
}

static int DeleteAllSensorInfoNode(sensor_info_list sensorInfoList)
{
	int iRet = -1;
	sensor_info_node_t * delNode = NULL, *head = NULL;
	int i = 0;
	if (sensorInfoList == NULL) return iRet;
	head = sensorInfoList;
	delNode = head->next;

	while (delNode)
	{
		head->next = delNode->next;
		if (delNode->sensorInfo.jsonStr != NULL)
		{
			free(delNode->sensorInfo.jsonStr);
			delNode->sensorInfo.jsonStr = NULL;
		}
		if (delNode->sensorInfo.pathStr != NULL)
		{
			free(delNode->sensorInfo.pathStr);
			delNode->sensorInfo.pathStr = NULL;
		}
		free(delNode);
		delNode = head->next;
	}
	iRet = 0;
	return iRet;
}

//read [Devices] sections in INI file
bool read_INI_Devices()
{
	char modulePath[200] = { 0 };
	char iniPath[200] = { 0 };
	char str[100] = { 0 };
	FILE *fPtr;
	char *temp_INI_name = NULL;
	bool bRetStatus = false;
	char *extini = NULL;

	temp_INI_name = (char *)calloc(strlen(strPluginName) + 1 + 4, sizeof(char));	//+4 for ".ini"
	strcpy(temp_INI_name, strPluginName);
	strcat(temp_INI_name, ".ini");

	// Load ini file
	util_module_path_get(modulePath);
	util_path_combine(iniPath, modulePath, temp_INI_name);

	fPtr = fopen(iniPath, "r");
	if (fPtr)
	{
		printf("INI Opened Successful...\n");
		numberOfWDev = GetIniKeyInt("Devices", "numberOfDevices", iniPath);

		if (numberOfWDev != 0)
		{
			WDev = (WISE_Device *)calloc(numberOfWDev, sizeof(WISE_Device));
		}

		for (int i = 0; i < numberOfWDev; i++)
		{
			char strDevice[10];
			double tmp;

			sprintf(strDevice, "Device%d", i);
			strcpy(str, GetIniKeyString("Devices", strDevice, iniPath));

			extini = strstr(str, ".ini");
			if(extini != NULL)
			{
				// With file extension .ini
				strncpy(WDev[i].devicename, str, extini-str);
				strcpy(WDev[i].filename, str);
			}
			else
			{
				// Without file extension .ini
				strcpy(WDev[i].devicename, str);
				strcpy(WDev[i].filename, str);
				strcat(WDev[i].filename, ".ini");
			}

			printf("Device%d: %s, INI: %s\n", i, WDev[i].devicename, WDev[i].filename);
		}
		fclose(fPtr);
		bRetStatus = true;
	}
	else
	{
		printf("INI Opened Failed...\n");
		bRetStatus = false;
	}
	free(temp_INI_name);
	temp_INI_name = NULL;
	return bRetStatus;
}

//==========================================================
//read platform section in INI file
bool read_INI_Platform(char *modulePath, char *iniPath)
{
	FILE *fPtr;
	char *temp_INI_name = NULL;
	bool bRetStatus = false;

	temp_INI_name = (char *)calloc(strlen(strPluginName) + 1 + 4, sizeof(char));	//+4 for ".ini"
	strcpy(temp_INI_name, strPluginName);
	strcat(temp_INI_name, ".ini");
	// Load ini file
	util_module_path_get(modulePath);
	util_path_combine(iniPath, modulePath, temp_INI_name);

	fPtr = fopen(iniPath, "r");
	if (fPtr)
	{
		printf("INI Opened Successful...\n");

		strcpy(Modbus_Protocol, GetIniKeyString("Platform", "Protocol", iniPath));
		strcpy(Device_Name, GetIniKeyString("Platform", "Name", iniPath));

		if (strcmp(Modbus_Protocol, DEF_MODBUS_TCP) == 0)
		{
			printf("Protocol : Modbus_TCP\n");
			iTCP_RTU = 0;

			strcpy(Modbus_Clent_IP, GetIniKeyString("Platform", "ClientIP", iniPath));
			Modbus_Client_Port = GetIniKeyInt("Platform", "ClientPort", iniPath);
			//Modbus_UnitID = GetIniKeyInt("Platform", "UnitID", iniPath);
			Modbus_Interval = GetIniKeyInt("Platform", "Interval", iniPath);

			if (Modbus_Interval <= 0)
			{
				Modbus_Interval = 1;
			}

			Modbus_Delay = GetIniKeyInt("Platform", "Delay", iniPath);

			if (Modbus_Delay <= 0)
			{
				Modbus_Delay = 1;
			}

			if (GetIniKeyInt("Platform", "Log", iniPath))
			{
				Modbus_Log = true;
			}
			else
			{
				Modbus_Log = false;
			}
		}
		else if (strcmp(Modbus_Protocol, DEF_MODBUS_RTU) == 0)
		{
			printf("Protocol : Modbus_RTU\n");
			iTCP_RTU = 1;

			strcpy(Modbus_Slave_Port, GetIniKeyString("Platform", "SlavePort", iniPath));
			Modbus_Baud = GetIniKeyInt("Platform", "Baud", iniPath);
			strcpy(Modbus_Parity, GetIniKeyString("Platform", "Parity", iniPath));
			Modbus_DataBits = GetIniKeyInt("Platform", "DataBits", iniPath);
			Modbus_StopBits = GetIniKeyInt("Platform", "StopBits", iniPath);
			//Modbus_SlaveID = GetIniKeyInt("Platform", "SlaveID", iniPath);
			
			Modbus_Interval = GetIniKeyInt("Platform", "Interval", iniPath);
			if (Modbus_Interval <= 0)
			{
				Modbus_Interval = 1;
			}

			Modbus_Delay = GetIniKeyInt("Platform", "Delay", iniPath);
			if (Modbus_Delay <= 0)
			{
				Modbus_Delay = 100;
			}

			if (GetIniKeyInt("Platform", "Log", iniPath))
			{
				Modbus_Log = true;
			}
			else
			{
				Modbus_Log = false;
			}
		}
		else
		{
			printf("Protocol error!!\n");
			iTCP_RTU = -1;
		}

		if (strcmp(Device_Name, DEF_SIMULATOR_NAME) == 0)
		{
			printf("bIsSimtor=true;\n");
			bIsSimtor = true;
		}
		else
		{
			printf("bIsSimtor=false;\n");
			bIsSimtor = false;
		}

		fclose(fPtr);
		bRetStatus = true;
	}
	else
	{
		printf("INI Opened Failed...\n");
		bRetStatus = false;
	}
	free(temp_INI_name);
	temp_INI_name = NULL;
	return bRetStatus;
}

bool read_INI_DeviceDetail()
{
	char *pDevPath = NULL;
	char *pModulePath = NULL;
	bool bRetStatus = true;
	FILE *fDevPtr;
	char *temp_DevINI_name = NULL;

	if (numberOfWDev != 0)
	{
		//*arrRegs = (uint16_t *)calloc(numberOfDevices, sizeof(uint16_t));
		//*arrRegs = (uint16_t *)calloc(numberOfDevices, numberOfRegs * sizeof(uint16_t));
	}

	for (int i = 0; i<numberOfWDev; i++)
	{
		temp_DevINI_name = (char *)calloc(strlen(WDev[i].filename) + 1, sizeof(char));
		pModulePath = (char *)calloc(200, sizeof(char));
		pDevPath = (char *)calloc(200, sizeof(char));
		util_module_path_get(pModulePath);
		strcpy(temp_DevINI_name, WDev[i].filename);
		// Load ini file
		util_path_combine(pDevPath, pModulePath, temp_DevINI_name);

		fDevPtr = fopen(pDevPath, "r");
		if (fDevPtr)
		{
			if (iTCP_RTU == 0)
			{
				WDev[i].deviceID = GetIniKeyInt("DeviceInfo", "UnitID", pDevPath);
			}
			else
			{
				WDev[i].deviceID = GetIniKeyInt("DeviceInfo", "SlaveID", pDevPath);
			}

			//--------------------DI
			read_INI_Bits("Discrete Inputs", "numberOfIB", pDevPath, &(WDev[i].DI), &(WDev[i].DI_Bits), &(WDev[i].numberOfDI));
			read_INI_Blocks("Discrete Inputs Block", "numberOfIBB", pDevPath, &(WDev[i].DIB), (void**)&(WDev[i].DI_Blocks), &(WDev[i].numberOfDIB));
			//--------------------DO
			read_INI_Bits("Coils", "numberOfB", pDevPath, &(WDev[i].DO), &(WDev[i].DO_Bits), &(WDev[i].numberOfDO));
			read_INI_Blocks("Coils Block", "numberOfBB", pDevPath, &(WDev[i].DOB), (void**)&(WDev[i].DO_Blocks), &(WDev[i].numberOfDOB));
			//--------------------AI
			read_INI_Registers("Input Registers", "numberOfIR", pDevPath, &(WDev[i].AI), &(WDev[i].AI_Regs), &(WDev[i].numberOfAI));
			read_INI_Blocks("Input Registers Block", "numberOfIRB", pDevPath, &(WDev[i].AIB), (void**)&(WDev[i].AI_Blocks), &(WDev[i].numberOfAIB));
			//--------------------AO
			read_INI_Registers("Holding Registers", "numberOfR", pDevPath, &(WDev[i].AO), &(WDev[i].AO_Regs), &(WDev[i].numberOfAO));
			read_INI_Blocks("Holding Registers Block", "numberOfRB", pDevPath, &(WDev[i].AOB), (void**)&(WDev[i].AO_Blocks), &(WDev[i].numberOfAOB));
			fclose(fDevPtr);
		}
		else
		{
			printf("Device INI Opened Failed...\n");
			bRetStatus = false;
		}
	}
	free(pModulePath);
	free(pDevPath);
	free(temp_DevINI_name);
	pModulePath = NULL;
	pDevPath = NULL;
	temp_DevINI_name = NULL;
	return bRetStatus;
}

bool read_INI_Bits(char* title, char* key, char* filename, WISE_Data** wdData, uint8_t** arrBits, int* numBits)
{
	char modulePath[200] = { 0 };
	char iniPath[200] = { 0 };
	char str[100] = { 0 };
	char *pstr = NULL;
	bool retValue = true;
	int numberOfBits = 0;
	FILE *fPtr;
	WISE_Data* pWd = NULL;

	int count = GetIniKeyInt(title, key, filename);
	*numBits = numberOfBits = count;

	if (count != 0)
	{
		*arrBits = (uint8_t *)calloc(count, sizeof(uint8_t));
		*wdData = (WISE_Data *)calloc(count, sizeof(WISE_Data));
		pWd = *wdData;
	}

	for (int i = 0; i < count; i++)
	{
		char strNumberOfBits[10];
		pWd[i].Regs = 0;
		pWd[i].Bits = 0;
		pWd[i].fv = 0;
		pWd[i].pre_fv = 0;
		pWd[i].uiv = 0;
		pWd[i].pre_uiv = 0;
		pWd[i].iv = 0;
		pWd[i].pre_iv = 0;
		pWd[i].bRevFin = false;

		if (strcmp(title, "Discrete Inputs") == 0)
		{
			sprintf(strNumberOfBits, "IB%d", i);
		}
		else
		{
			sprintf(strNumberOfBits, "B%d", i);
		}

		strcpy(str, GetIniKeyString(title, strNumberOfBits, filename));

		pstr = strtok(str, ",");
		if (pstr != NULL)
			pWd[i].address = atoi(pstr);//get the address
		else
			pWd[i].address = 0;

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			strcpy(pWd[i].name, pstr); // get the name
		else
			strcpy(pWd[i].name, "");

		printf("%d. Bit address: %d name: %s\n", i, pWd[i].address, pWd[i].name);
	}
	return retValue;
}

bool read_INI_Registers(char* title, char* key, char* filename, WISE_Data** wdData, uint16_t** arrRegs, int* numRegs)
{
	char modulePath[200] = { 0 };
	char iniPath[200] = { 0 };
	char str[100] = { 0 };
	char *pstr = NULL;
	bool retValue = true;
	int numberOfRegs = 0;
	FILE *fPtr;
	WISE_Data* pWd = NULL;

	int count = GetIniKeyInt(title, key, filename);
	*numRegs = numberOfRegs = count;

	if (count != 0)
	{
		*wdData = (WISE_Data *)calloc(count, sizeof(WISE_Data));
		pWd = *wdData;
	}

	for (int i = 0; i < count; i++)
	{
		char strNumberOfRegs[10];
		double tmp;
		pWd[i].Regs = 0;
		pWd[i].Bits = 0;
		pWd[i].fv = 0;
		pWd[i].pre_fv = 0;
		pWd[i].uiv = 0;
		pWd[i].pre_uiv = 0;
		pWd[i].iv = 0;
		pWd[i].pre_iv = 0;
		pWd[i].bRevFin = false;

		if(strcmp(title, "Input Registers") == 0)
		{
			sprintf(strNumberOfRegs, "IR%d", i);
		}
		else
		{
			sprintf(strNumberOfRegs, "R%d", i);
		}

		strcpy(str, GetIniKeyString(title, strNumberOfRegs, filename));

		pstr = strtok(str, ",");
		if (pstr != NULL)
			pWd[i].address = atoi(pstr);//get the address
		else
			pWd[i].address = 0;

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			strcpy(pWd[i].name, pstr); // get the name
		else
			strcpy(pWd[i].name, "");

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			pWd[i].min = atof(pstr); // get the min
		else
			pWd[i].min = 0;

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			pWd[i].max = atof(pstr); // get the max
		else
			pWd[i].max = 0;

		if (pWd[i].min > pWd[i].max)
		{
			tmp = pWd[i].min;
			pWd[i].min = pWd[i].max;
			pWd[i].max = tmp;
		}

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			pWd[i].precision = atof(pstr); // get the precision
		else
			pWd[i].precision = 1;

		if (pWd[i].precision == 0)
			pWd[i].precision = 1;

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			strcpy(pWd[i].unit, pstr); // get the unit
		else
			strcpy(pWd[i].unit, "");

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			pWd[i].sw_mode = atoi(pstr); // get the sw_mode
		else
			pWd[i].sw_mode = 0;

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			strcpy(pWd[i].conversion, pstr); // get the conversion
		else
			strcpy(pWd[i].conversion, "");

		if (pWd[i].sw_mode == 1 || pWd[i].sw_mode == 2 || pWd[i].sw_mode == 3 || pWd[i].sw_mode == 4 ||
			pWd[i].sw_mode == 5 || pWd[i].sw_mode == 6 || pWd[i].sw_mode == 7 || pWd[i].sw_mode == 8)
		{
			numberOfRegs++;
		}
		printf("%d. Register address: %d name: %s min: %lf max: %lf pre: %lf unit: %s sw_mode: %d\n", 
			i, pWd[i].address, pWd[i].name, pWd[i].min, pWd[i].max, pWd[i].precision, pWd[i].unit, pWd[i].sw_mode);
	}

	if (numberOfRegs != 0)
	{
		*arrRegs = (uint16_t *)calloc(numberOfRegs, sizeof(uint16_t));
	}
	return retValue;
}

bool read_INI_Blocks(char* title, char* key, char* filename, WISE_Block** wbData, void** arrRegs, int* numBlocks)
{
	char modulePath[200] = { 0 };
	char iniPath[200] = { 0 };
	char str[100] = { 0 };
	char *pstr = NULL;
	bool retValue = true;
	int numberOfBlocks = 0;
	int numberOfRegs = 0;
	FILE *fPtr;
	WISE_Block* pWb = NULL;
	bool isBitBlock = true;

	int count = GetIniKeyInt(title, key, filename);

	*numBlocks = numberOfBlocks = count;
	if (count != 0)
	{
		*wbData = (WISE_Block *)calloc(count, sizeof(WISE_Block));
		pWb = *wbData;
	}

	for (int i = 0; i < count; i++)
	{
		char strNumberOfBlocks[10];
		double tmp;

		/*
		for (int j = 0; j < MAX_BIT_BLOCK_LENGTH; j++)
		{
			pWb[i].Bits[j] = 0;
		}
		*/

		/*
		for (int j=0; j<MAX_REGISTER_BLOCK_LENGTH; j++)
		{
			pWb[i].Regs[j] = 0;
		}
		*/

		if (strcmp(title, "Discrete Inputs Block") == 0)
		{
			sprintf(strNumberOfBlocks, "IBB%d", i);
			isBitBlock = true;
		}
		else if (strcmp(title, "Coils Block") == 0)
		{
			sprintf(strNumberOfBlocks, "BB%d", i);
			isBitBlock = true;
		}
		else if (strcmp(title, "Input Registers Block") == 0)
		{
			sprintf(strNumberOfBlocks, "IRB%d", i);
			isBitBlock = false;
		}
		else if(strcmp(title, "Holding Registers Block") == 0)
		{
			sprintf(strNumberOfBlocks, "RB%d", i);
			isBitBlock = false;
		}

		strcpy(str, GetIniKeyString(title, strNumberOfBlocks, filename));

		pstr = strtok(str, ",");
		if (pstr != NULL)
			pWb[i].address = atoi(pstr);//get the address
		else
			pWb[i].address = 0;

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			pWb[i].length = atof(pstr); // get the length
		else
			pWb[i].length = 0;

		pstr = strtok(NULL, ",");
		if (pstr != NULL)
			strcpy(pWb[i].name, pstr); // get the name
		else
			strcpy(pWb[i].name, "");

		numberOfBlocks++;
		numberOfRegs = numberOfRegs + pWb[i].length;
		printf("%d. Block address: %d length: %d name: %s\n", i, pWb[i].address, pWb[i].length, pWb[i].name);
	}

	if (numberOfBlocks != 0)
	{
		if (isBitBlock == true)
		{
			*arrRegs = (uint8_t *)calloc(numberOfBlocks, numberOfRegs * sizeof(uint8_t));
		}
		else
		{
			*arrRegs = (uint16_t *)calloc(numberOfBlocks, numberOfRegs * sizeof(uint16_t));
		}
	}
	return retValue;
}

double LuaConversion(double inputVal, char* strLua)
{
	double retVal = inputVal;
	char strLuaFunc[300];
#ifdef LUA
	lua_State *L = NULL;

	//if (luaL_dostring(L, "function Convert(modbus_val) return modbus_val+1 end"))
	//if (luaL_dostring(L, "function Convert(modbus_val) return math.pow(modbus_val,2) end"))
	strcpy(strLuaFunc, "function Convert(modbus_val) return ");

	char* pStrTemp = strchr(strLua, '\"');
	if (pStrTemp != NULL)
	{
		if (strlen(strLua) <= 2)
			return retVal;
		pStrTemp++;

		char* pStrLuaScript = strtok(pStrTemp, "\"");
		if (pStrLuaScript != NULL)
		{
			strcpy(strLua, strtok(pStrTemp, "\""));
		}
		else
		{
			return -1;
		}
	}

	strcat(strLuaFunc, strLua);
	strcat(strLuaFunc, " end");

	L = luaL_newstate();
	luaL_openlibs(L);


	if (luaL_dostring(L, strLuaFunc))
	{
		lua_close(L);
		return retVal;
	}

	lua_getglobal(L, "Convert");
	lua_pushnumber(L, inputVal);
	lua_call(L, 1, 1);

	int iLuaType = lua_type(L, -1);
	switch (iLuaType)
	{
	case LUA_TNIL:
		retVal = lua_tointeger(L, -1);
		printf("Result: %i\n", retVal);
		break;

	case LUA_TBOOLEAN:
		break;

	case LUA_TLIGHTUSERDATA:
		break;

	case LUA_TNUMBER:
		retVal = lua_tonumber(L, -1);
		printf("Result: %f\n", retVal);
		break;

	case LUA_TSTRING:
		break;
	case LUA_TTABLE:
		break;
	case LUA_TFUNCTION:
		break;
	case LUA_TUSERDATA:
		break;
	case LUA_TTHREAD:
		break;
	default:
		break;
	}
	lua_close(L); //close Lua state
#endif
	return retVal;
}

//Prepare to Upload Data
void UpDataPrepare(WISE_Data *Data, bool bUAI_AO, int *AIORcur, bool ret, uint16_t** arrRegs)	//DI,DO already keep the latest data,so dont have to take care here.
{
	int i = 0;

	if (bUAI_AO)
	{
		if(*arrRegs != NULL)
		{
			AI_Regs_temp_Ary[0] = **arrRegs;
		}

		if((*arrRegs+1) != NULL)
		{
			AI_Regs_temp_Ary[1] = *(*arrRegs + 1);
		}

		switch (Data->sw_mode)
		{
		case 1:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float(AI_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 2:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float_dcba(AI_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 3:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float_badc(AI_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 4:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float_cdab(AI_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 5:
			if (Data->bRevFin)
			{
				Data->uiv = custom_get_unsigned_int(AI_Regs_temp_Ary);
				Data->pre_uiv = Data->uiv;
			}
			else
			{
				Data->uiv = Data->pre_uiv;
			}
			(*AIORcur) += 2;
			break;

		case 6:
			if (Data->bRevFin)
			{
				Data->uiv = custom_get_unsigned_int_cdab(AI_Regs_temp_Ary);
				Data->pre_uiv = Data->uiv;
			}
			else
			{
				Data->uiv = Data->pre_uiv;
			}
			(*AIORcur) += 2;
			break;

		case 7:
			if (Data->bRevFin)
			{
				Data->iv = custom_get_int(AI_Regs_temp_Ary);
				Data->pre_iv = Data->iv;
			}
			else
			{
				Data->iv = Data->pre_iv;
			}
			(*AIORcur) += 2;
			break;

		case 8:
			if (Data->bRevFin)
			{
				Data->iv = custom_get_int_cdab(AI_Regs_temp_Ary);
				Data->pre_iv = Data->iv;
			}
			else
			{
				Data->iv = Data->pre_iv;
			}
			(*AIORcur) += 2;
			break;

		default:
			Data->Regs = **arrRegs;
			(*AIORcur)++;
		}
	}
	else
	{
		if(*arrRegs != NULL)
		{
			AO_Regs_temp_Ary[0] = **arrRegs;
		}

		if((*arrRegs+1) != NULL)
		{
			AO_Regs_temp_Ary[1] = *(*arrRegs + 1);
		}

		switch (Data->sw_mode)
		{
		case 1:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float(AO_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 2:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float_dcba(AO_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 3:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float_badc(AO_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 4:
			if (Data->bRevFin)
			{
				Data->fv = custom_get_float_cdab(AO_Regs_temp_Ary);
				Data->pre_fv = Data->fv;
			}
			else
			{
				Data->fv = Data->pre_fv;
			}
			(*AIORcur) += 2;
			break;

		case 5:
			if (Data->bRevFin)
			{
				Data->uiv = custom_get_unsigned_int(AO_Regs_temp_Ary);
				Data->pre_uiv = Data->uiv;
			}
			else
			{
				Data->uiv = Data->pre_uiv;
			}
			(*AIORcur) += 2;
			break;

		case 6:
			if (Data->bRevFin)
			{
				Data->uiv = custom_get_unsigned_int_cdab(AO_Regs_temp_Ary);
				Data->pre_uiv = Data->uiv;
			}
			else
			{
				Data->uiv = Data->pre_uiv;
			}
			(*AIORcur) += 2;
			break;

		case 7:
			if (Data->bRevFin)
			{
				Data->iv = custom_get_int(AO_Regs_temp_Ary);
				Data->pre_iv = Data->iv;
			}
			else
			{
				Data->iv = Data->pre_iv;
			}
			(*AIORcur) += 2;
			break;

		case 8:
			if (Data->bRevFin)
			{
				Data->iv = custom_get_int_cdab(AO_Regs_temp_Ary);
				Data->pre_iv = Data->iv;
			}
			else
			{
				Data->iv = Data->pre_iv;
			}
			(*AIORcur) += 2;
			break;

		default:
			Data->Regs = **arrRegs;
			(*AIORcur)++;
		}
	}
}

//Prepare to Download Data
int DownDataPrepare(WISE_Data *Data, int devid, float fv, uint32_t uiv, int iv, bool ret)
{
	int rc;
	if (!ret)
	{
		Data->fv = 0;
		Data->Regs = 0;
		return -1;
	}
	else
	{
		pthread_mutex_lock(&pModbusMux);
		modbus_set_slave(ctx, devid);

		switch (Data->sw_mode)
		{
		case 1:
			custom_set_float(fv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);
			break;
		case 2:
			custom_set_float_dcba(fv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);
			break;
		case 3:
			custom_set_float_badc(fv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);
			break;
		case 4:
			custom_set_float_cdab(fv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);
			break;
		case 5:
			custom_set_unsigned_int(uiv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);
			break;
		case 6:
			custom_set_unsigned_int_cdab(uiv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);;
			break;
		case 7:
			custom_set_int(iv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);
			break;
		case 8:
			custom_set_int_cdab(iv, AO_Set_Regs_temp_Ary);
			rc = modbus_write_register(ctx, Data->address, AO_Set_Regs_temp_Ary[0]);
			rc = modbus_write_register(ctx, Data->address + 1, AO_Set_Regs_temp_Ary[1]);
			break;
		default:
			rc = modbus_write_register(ctx, Data->address, fv / Data->precision);
		}

		if (Modbus_Delay > 0)
		{
			usleep(Modbus_Delay * 1000);
		}

		pthread_mutex_unlock(&pModbusMux);
		return rc;
	}
}

//Prepare to Download Bit Block Data
int DownBitBlockPrepare(WISE_Block *Data, int devid, char* sv, bool ret)
{
	int rc;
	char tmpbuff[1024];
	uint8_t Bits[MAX_BIT_BLOCK_LENGTH];

	memset(tmpbuff, 0, sizeof(tmpbuff));
	memset(Bits, 0, sizeof(Bits));

	for (int i = 0; i<strlen(sv); i++)
	{
		if (sv[i] != ' ')
		{
			strcpy(tmpbuff, sv + i);
			break;
		}
		else
		{
			continue;
		}
	}

	if (!ret)
	{
		return -1;
	}
	else
	{
		for (int i = 0; i<Data->length; i++)
		{	
			if (tmpbuff[i] == '0')
			{
				Bits[i] = 0;
			}
			else
			{
				Bits[i] = 1;
			}
		}

		pthread_mutex_lock(&pModbusMux);
		modbus_set_slave(ctx, devid);
		rc = modbus_write_bits(ctx, Data->address, Data->length, Bits);

		if (Modbus_Delay > 0)
			usleep(Modbus_Delay * 1000);

		pthread_mutex_unlock(&pModbusMux);
		return rc;
	}
}

//Prepare to Download Register Block Data
int DownRegisterBlockPrepare(WISE_Block *Data, int devid, char* sv, bool ret)
{
	int rc;
	int value = 0;
	char tmpvalue[1024];
	char tmpbuff[1024];
	uint16_t Regs[MAX_REGISTER_BLOCK_LENGTH];

	memset(tmpbuff, 0, sizeof(tmpbuff));
	memset(Regs, 0, sizeof(Regs));

	for (int i=0; i<strlen(sv); i++)
	{
		if (sv[i] != ' ')
		{
			strcpy(tmpbuff, sv+i);
			break;
		}
		else
		{
			continue;
		}
	}

	if (!ret)
	{
		return -1;
	}
	else
	{
		for (int i=0; i<Data->length; i++)
		{
			char* separter = strchr(tmpbuff, ' ');
			if (separter)
			{
				memset(tmpvalue, 0, sizeof(tmpvalue));
				strncpy(tmpvalue, tmpbuff, separter - tmpbuff);
				//value = atoi(tmpvalue);
				value = (int)strtol(tmpvalue, NULL, 16);
				strcpy(tmpbuff, separter + 1);
			}
			else
			{
				strcpy(tmpvalue, tmpbuff);
				//value = atoi(tmpvalue);
				value = (int)strtol(tmpvalue, NULL, 16);
			}
			Regs[i] = value;
		}

		pthread_mutex_lock(&pModbusMux);
		modbus_set_slave(ctx, devid);
		rc = modbus_write_registers(ctx, Data->address, Data->length, Regs);

		if (Modbus_Delay > 0)
			usleep(Modbus_Delay * 1000);

		pthread_mutex_unlock(&pModbusMux);
		return rc;
	}
}

//Assemble Data
void AssembleData()
{
	int AIRcur = 0;
	int AORcur = 0;
	uint16_t* pRegs;

	for (int count=0; count<numberOfWDev; count++)
	{
		pRegs = WDev[count].AI_Regs;

		for (int i = 0; i < WDev[count].numberOfAI; i++)
		{
			UpDataPrepare(&(WDev[count].AI[i]), true, &AORcur, g_bRetrieve, &pRegs);

			if(WDev[count].AI[i].sw_mode < 1)
			{
				pRegs++;
			}
			else
			{
				pRegs+=2;
			}
		}

		pRegs = WDev[count].AO_Regs;

		for (int i = 0; i < WDev[count].numberOfAO; i++)
		{
			UpDataPrepare(&(WDev[count].AO[i]), false, &AORcur, g_bRetrieve, &pRegs);

			if(WDev[count].AO[i].sw_mode < 1)
			{
				pRegs++;				
			}
			else
			{
				pRegs+=2;
			}
		}
	}
}

MSG_CLASSIFY_T * CreateCapability()
{
	MSG_CLASSIFY_T *rootCap = IoT_CreateRoot((char*)strPluginName);
	MSG_CLASSIFY_T *devCap;
	MSG_CLASSIFY_T *devInfoCap;
	MSG_CLASSIFY_T *modbusCap = NULL;
	MSG_ATTRIBUTE_T* attr;
	IoT_READWRITE_MODE mode = IoT_READONLY;

	char devName[80];
	char Client_Port[6];
	char Client_UnitID[6];
	char SlaveID[6];
	bool bUAI_AO;

	if (bFind)
	{
		devCap = IoT_AddGroup(rootCap, "Platform");
		if (devCap)
		{
			mode = IoT_READONLY;
			attr = IoT_AddSensorNode(devCap, "Version");
			if (attr)
				IoT_SetStringValue(attr, strPluginVersion, mode);

			attr = IoT_AddSensorNode(devCap, "Description");
			if (attr)
				IoT_SetStringValue(attr, strPluginDescription, mode);

			attr = IoT_AddSensorNode(devCap, "Protocol");
			if (attr)
				IoT_SetStringValue(attr, Modbus_Protocol, mode);

			attr = IoT_AddSensorNode(devCap, "Name");
			if (attr)
				IoT_SetStringValue(attr, Device_Name, mode);

			if (iTCP_RTU == 0)
			{
				sprintf(Client_Port, "%d", Modbus_Client_Port);
				//sprintf(Client_UnitID, "%d", Modbus_UnitID);

				attr = IoT_AddSensorNode(devCap, "ClientIP");
				if (attr)
					IoT_SetStringValue(attr, Modbus_Clent_IP, mode);

				attr = IoT_AddSensorNode(devCap, "ClientPort");
				if (attr)
					IoT_SetStringValue(attr, Client_Port, mode);

				/*
				attr = IoT_AddSensorNode(devCap, "UnitID");
				if (attr)
					IoT_SetStringValue(attr, Client_UnitID, mode);
				*/
			}
			else if (iTCP_RTU == 1)
			{
				//sprintf(SlaveID, "%d", Modbus_SlaveID);

				attr = IoT_AddSensorNode(devCap, "SlavePort");
				if (attr)
					IoT_SetStringValue(attr, Modbus_Slave_Port, mode);

				attr = IoT_AddSensorNode(devCap, "Baud");
				if (attr)
					IoT_SetDoubleValue(attr, Modbus_Baud, mode, "bps");

				attr = IoT_AddSensorNode(devCap, "Parity");
				if (attr)
					IoT_SetStringValue(attr, Modbus_Parity, mode);

				attr = IoT_AddSensorNode(devCap, "DataBits");
				if (attr)
					IoT_SetDoubleValue(attr, Modbus_DataBits, mode, "bits");

				attr = IoT_AddSensorNode(devCap, "StopBits");
				if (attr)
					IoT_SetDoubleValue(attr, Modbus_StopBits, mode, "bits");

				/*
				attr = IoT_AddSensorNode(devCap, "SlaveID");
				if (attr)
					IoT_SetStringValue(attr, SlaveID, mode);
				*/
			}
			else
			{
				;//Not TCP or RTU
			}

			attr = IoT_AddSensorNode(devCap, "Connection");
			if (attr)
			{
				if (bIsSimtor)
				{
					IoT_SetBoolValue(attr, true, mode);
				}
				else
				{
					IoT_SetBoolValue(attr, bConnectionFlag, mode);
				}
			}
		}

		for (int i=0; i<numberOfWDev; i++)
		{
			strcpy(devName, WDev[i].devicename);
			IoTSetDeviceInfo(rootCap, devCap, devName, attr, i);

			if (WDev[i].numberOfDI != 0)
			{
				IoTSetBits(rootCap, devCap, devName, modbusCap, "Discrete Inputs", 
					attr, NULL, 0, &(WDev[i].DI), &(WDev[i].DI_Bits), WDev[i].numberOfDI, true);
			}

			if (WDev[i].numberOfDO != 0)
			{
				IoTSetBits(rootCap, devCap, devName, modbusCap, "Coils", 
					attr, NULL, 0, &(WDev[i].DO), &(WDev[i].DO_Bits), WDev[i].numberOfDO, false);
			}

			if (WDev[i].numberOfAI != 0)
			{
				IoTSetRegisters(rootCap, devCap, devName, modbusCap, "Input Registers", 
					attr, NULL, 0, &(WDev[i].AI), &(WDev[i].AI_Regs), WDev[i].numberOfAI, true);
			}

			if (WDev[i].numberOfAO != 0)
			{
				IoTSetRegisters(rootCap, devCap, devName, modbusCap, "Holding Registers", 
					attr, NULL, 0, &(WDev[i].AO), &(WDev[i].AO_Regs), WDev[i].numberOfAO, false);
			}

			if (WDev[i].numberOfDIB != 0)
			{
				IoTSetBitsBlock(rootCap, devCap, devName, modbusCap, "Discrete Inputs Block", 
					attr, NULL, 0, &(WDev[i].DIB), &(WDev[i].DI_Blocks), WDev[i].numberOfDIB, true);
			}

			if (WDev[i].numberOfDOB != 0)
			{
				IoTSetBitsBlock(rootCap, devCap, devName, modbusCap, "Coils Block", 
					attr, NULL, 0, &(WDev[i].DOB), &(WDev[i].DO_Blocks), WDev[i].numberOfDOB, false);
			}

			if (WDev[i].numberOfAIB != 0)
			{
				IoTSetRegistersBlock(rootCap, devCap, devName, modbusCap, "Input Registers Block", 
					attr, NULL, 0, &(WDev[i].AIB), &(WDev[i].AI_Blocks), WDev[i].numberOfAIB, true);
			}

			if (WDev[i].numberOfAOB != 0)
			{
				IoTSetRegistersBlock(rootCap, devCap, devName, modbusCap, "Holding Registers Block", 
					attr, NULL, 0, &(WDev[i].AOB), &(WDev[i].AO_Blocks), WDev[i].numberOfAOB, false);
			}
		}
	}
	return rootCap;
}

//--------------------------------------------------------------------------------------------------------------
//------------------------------------------------Modbus--------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//Establish a Modbus Connection
bool Modbus_Connect()
{
	if (modbus_connect(ctx) == -1)
	{
		ModbusLog(g_loghandle, Error, "Modbus Connection failed!! - Connect ERROR : %s", modbus_strerror(errno));

		bConnectionFlag = false;
		return false;
	}
	else
	{
		ModbusLog(g_loghandle, Normal, "Modbus Connection established!!");

		pthread_mutex_init(&pModbusMux, NULL);
		pthread_mutex_init(&pModbusThresholdMux, NULL);

		if (iTCP_RTU == 0)
		{
			//modbus_set_slave(ctx, Modbus_UnitID);               // To do by Hank
			bConnectionFlag = true;
		}
		else if (iTCP_RTU == 1)
		{
			int ret = 0;
			//modbus_set_slave(ctx, 2);               // To do by Hank
			ret = modbus_rtu_set_serial_mode(ctx, 1);
			if (ret != 0)
			{
				printf("set serial mode failed\n");
			}
		}
		else
			;	//Not TCP or RTU
		return true;
	}
}

//Disconnect a Modbus Connection
bool Modbus_Disconnect()
{
	pthread_mutex_destroy(&pModbusMux);
	bConnectionFlag = false;
	modbus_close(ctx);
	ModbusLog(g_loghandle, Warning, "Modbus Disconnection!!");
	return false;
}

//Receive Data From Modbus
bool Modbus_Rev()
{
	int ret = -1;
	int i = 0;
	bool bFCon = false;
	char str[655350] = "";

	strcat(str, "{");

	if (g_bRev_Fail == true)
		bFCon = true;

	if (bFCon && g_bRev_Fail)
		Rev_Fail_Num++;
	else
		Rev_Fail_Num = 0;

	//------------------floatint test case   10.565(abcd) 0.033731(dcba) refer to float_sample.txt
	/*uint16_t a=16681;
	uint16_t b=2621;
	uint16_t *t=(uint16_t *)calloc(2,sizeof(uint16_t));
	t[0]=a;
	t[1]=b;
	printf("%f\n",custom_get_float(t));
	printf("%f\n",custom_get_float_dcba(t));*/
	//------------------floatint test case end

	//printf("Rev_Fail_Num : %d - %d\n", Rev_Fail_Num, time(NULL));
	if (Rev_Fail_Num > 5) //reconnect --> continual 6 times lost connection  
	{
		Modbus_Disconnect();
		sleep(5);			//sleep 5 secs for avoiding lots of sockets created and not released yet

		if (!Modbus_Connect())
			return false;
		else
			Rev_Fail_Num = 0;
	}

	for (int count=0; count<numberOfWDev; count++)
	{
		if (WDev[count].numberOfDI != 0 ||
			WDev[count].numberOfDO != 0 ||
			WDev[count].numberOfAI != 0 ||
			WDev[count].numberOfAO != 0 ||
			WDev[count].numberOfDIB != 0 ||
			WDev[count].numberOfDOB != 0 ||
			WDev[count].numberOfAIB != 0 ||
			WDev[count].numberOfAOB != 0)
		{
			g_bRev_Fail = true;
		}
	}

	for (int count = 0; count < numberOfWDev; count++)
	{
		if (WDev[count].numberOfDI != 0)
		{
			for (i = 0; i < WDev[count].numberOfDI; i++)
			{
				char tmp[50];
				WDev[count].DI[i].bRevFin = false;	//not in use actually

				if (ctx != NULL)
				{
					pthread_mutex_lock(&pModbusMux);
					modbus_set_slave(ctx, WDev[count].deviceID);
					ret = modbus_read_input_bits(ctx, WDev[count].DI[i].address, 1, (WDev[count].DI_Bits + i));
					usleep(Modbus_Delay * 1000);
					pthread_mutex_unlock(&pModbusMux);
				}
				else
				{
					ret = -1;
				}

				if (ret == -1)
				{
					ModbusLog(g_loghandle, Error, "modbus_read_input_bits failed - %d - IB[%d] Rev ERROR : %s", 
						Rev_Fail_Num, i, modbus_strerror(errno));

					sprintf(tmp, "\"IB[%d]\":\"%s\"", i, "FAIL");
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfDI - 1)) ||
						WDev[count].numberOfDO != 0 ||
						WDev[count].numberOfAI != 0 ||
						WDev[count].numberOfAO != 0)
					{
						strcat(str, ",");
					}
				}
				else
				{
					sprintf(tmp, "\"IB[%d]\":%d", i, *(WDev[count].DI_Bits + i));
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfDI - 1)) ||
						WDev[count].numberOfDO != 0 ||
						WDev[count].numberOfAI != 0 ||
						WDev[count].numberOfAO != 0)
					{
						strcat(str, ",");
					}

					if (!bConnectionFlag)
						bConnectionFlag = true;

					WDev[count].DI[i].Bits = *(WDev[count].DI_Bits + i);
					WDev[count].DI[i].bRevFin = true;	//not in use actually

					//printf("DI_Bits[%d] : %d\n", i, DI_Bits[i]);
					g_bRev_Fail = false;
				}
				//sleep(2);
			}
		}

		if (WDev[count].numberOfDO != 0)
		{
			for (i = 0; i < WDev[count].numberOfDO; i++)
			{
				char tmp[50];
				WDev[count].DO[i].bRevFin = false;	//not in use actually
				if (ctx != NULL)
				{
					pthread_mutex_lock(&pModbusMux);
					modbus_set_slave(ctx, WDev[count].deviceID);
					ret = modbus_read_bits(ctx, WDev[count].DO[i].address, 1, (WDev[count].DO_Bits + i));
					if (Modbus_Delay > 0)
					{
						usleep(Modbus_Delay * 1000);
					}
					pthread_mutex_unlock(&pModbusMux);
				}
				else
					ret = -1;

				if (ret == -1)
				{
					ModbusLog(g_loghandle, Error, "modbus_read_bits failed - %d - B[%d] Rev ERROR : %s", 
						Rev_Fail_Num, i, modbus_strerror(errno));

					sprintf(tmp, "\"B[%d]\":\"%s\"", i, "FAIL");
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfDI - 1)) ||
						WDev[count].numberOfDO != 0 ||
						WDev[count].numberOfAI != 0 ||
						WDev[count].numberOfAO != 0)
					{
						strcat(str, ",");
					}
				}
				else
				{
					sprintf(tmp, "\"B[%d]\":%d", i, *(WDev[count].DO_Bits + i));
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfDO - 1)) || WDev[count].numberOfAI != 0 || WDev[count].numberOfAO != 0)
					{
						strcat(str, ",");
					}

					if (!bConnectionFlag)
					{
						bConnectionFlag = true;
					}

					WDev[count].DO[i].Bits = *(WDev[count].DO_Bits + i);
					WDev[count].DO[i].bRevFin = true;	//not in use actually

					//printf("DO_Bits[%d] : %d\n", i, DO_Bits[i]);
					g_bRev_Fail = false;
				}
				//sleep(2);
			}
		}

		if (WDev[count].numberOfAI != 0)
		{
			int AIRRevcur = 0;
			for (i = 0; i < WDev[count].numberOfAI; i++)
			{
				char tmp[50];
				WDev[count].AI[i].bRevFin = false;
				if (WDev[count].AI[i].sw_mode == 1 || WDev[count].AI[i].sw_mode == 2 || 
					WDev[count].AI[i].sw_mode == 3 || WDev[count].AI[i].sw_mode == 4 || 
					WDev[count].AI[i].sw_mode == 5 || WDev[count].AI[i].sw_mode == 6 || 
					WDev[count].AI[i].sw_mode == 7 || WDev[count].AI[i].sw_mode == 8)
				{
					if (ctx != NULL)
					{
						pthread_mutex_lock(&pModbusMux);

						modbus_set_slave(ctx, WDev[count].deviceID);
						ret = modbus_flush(ctx);
						if(ret > 0)
						{
							//printf("Flush %d byte(s) non-transmitted input register data!\r\n");
						}

						ret = modbus_read_input_registers(ctx, WDev[count].AI[i].address, 2, (WDev[count].AI_Regs + AIRRevcur));
						if (Modbus_Delay > 0)
						{
							usleep(Modbus_Delay * 1000);
						}

						pthread_mutex_unlock(&pModbusMux);
					}
					else
					{
						ret = -1;
					}

					if (ret == -1)
					{
						ModbusLog(g_loghandle, Error, "modbus_read_input_registers failed - %d - IR[%d] Rev ERROR : %s",
							Rev_Fail_Num, i, modbus_strerror(errno));
						sprintf(tmp, "\"IR[%d]\":\"%s\"", i, "FAIL");
						strcat(str, tmp);
						if ((i != (WDev[count].numberOfAI - 1)) || WDev[count].numberOfAO != 0)
						{
							strcat(str, ",");
						}
					}
					else
					{
						sprintf(tmp, "\"IR[%d]\":[%d,%d]", i, *(WDev[count].AI_Regs + AIRRevcur), *(WDev[count].AI_Regs + AIRRevcur + 1));
						strcat(str, tmp);

						if ((i != (WDev[count].numberOfAI - 1)) || WDev[count].numberOfAO != 0)
						{
							strcat(str, ",");
						}

						if (!bConnectionFlag)
						{
							bConnectionFlag = true;
						}

						g_bRev_Fail = false;
					}
	
					AIRRevcur += 2;
					WDev[count].AI[i].bRevFin = true;
				}
				else
				{
					if (ctx != NULL)
					{
						pthread_mutex_lock(&pModbusMux);

						modbus_set_slave(ctx, WDev[count].deviceID);
						ret = modbus_flush(ctx);
						if(ret > 0)
						{
							//printf("Flush %d byte(s) non-transmitted input register data!\r\n");
						}
						ret = modbus_read_input_registers(ctx, WDev[count].AI[i].address, 1, (WDev[count].AI_Regs + AIRRevcur));

						if (Modbus_Delay > 0)
						{
							usleep(Modbus_Delay * 1000);
						}

						pthread_mutex_unlock(&pModbusMux);
					}
					else
					{
						ret = -1;
					}

					if (ret == -1)
					{
						ModbusLog(g_loghandle, Error, "modbus_read_input_registers failed - %d - IR[%d] Rev ERROR : %s", 
							Rev_Fail_Num, i, modbus_strerror(errno));

						sprintf(tmp, "\"IR[%d]\":\"%s\"", i, "FAIL");
						strcat(str, tmp);

						if ((i != (WDev[count].numberOfAI - 1)) || WDev[count].numberOfAO != 0)
						{
							strcat(str, ",");
						}
					}
					else
					{
						sprintf(tmp, "\"IR[%d]\":%d", i, *(WDev[count].AI_Regs + AIRRevcur));
						strcat(str, tmp);

						if ((i != (WDev[count].numberOfAI - 1)) || WDev[count].numberOfAO != 0)
						{
							strcat(str, ",");
						}

						if (!bConnectionFlag)
						{
							bConnectionFlag = true;
						}

						WDev[count].AI[i].Regs = *(WDev[count].AI_Regs + AIRRevcur);
						g_bRev_Fail = false;
					}

					//printf("AI_Regs[%d] : %d\n",i,*(AI_Regs+AIRRevcur));
					AIRRevcur++;
					WDev[count].AI[i].bRevFin = true;
				}
				//sleep(2);
			}
		}

		if (WDev[count].numberOfAO != 0)
		{
			int AORRevcur = 0;
			for (i = 0; i < WDev[count].numberOfAO; i++)
			{
				char tmp[50];
				WDev[count].AO[i].bRevFin = false;
				if (WDev[count].AO[i].sw_mode == 1 || WDev[count].AO[i].sw_mode == 2 ||
					WDev[count].AO[i].sw_mode == 3 || WDev[count].AO[i].sw_mode == 4 ||
					WDev[count].AO[i].sw_mode == 5 || WDev[count].AO[i].sw_mode == 6 ||
					WDev[count].AO[i].sw_mode == 7 || WDev[count].AO[i].sw_mode == 8)
				{
					if (ctx != NULL)
					{
						pthread_mutex_lock(&pModbusMux);

						modbus_set_slave(ctx, WDev[count].deviceID);
						ret = modbus_flush(ctx);
						if(ret > 0)
						{
							//printf("Flush %d byte(s) non-transmitted holding register data!\r\n");
						}
						ret = modbus_read_registers(ctx, WDev[count].AO[i].address, 2, (WDev[count].AO_Regs + AORRevcur));

						if (Modbus_Delay > 0)
						{
							usleep(Modbus_Delay * 1000);
						}
						pthread_mutex_unlock(&pModbusMux);
					}
					else
					{
						ret = -1;
					}

					if (ret == -1)
					{
						ModbusLog(g_loghandle, Error, "modbus_read_registers failed - %d - R[%d] Rev ERROR : %s", 
							Rev_Fail_Num, i, modbus_strerror(errno));

						sprintf(tmp, "\"R[%d]\":\"%s\"", i, "FAIL");
						strcat(str, tmp);

						if ((i != (WDev[count].numberOfAO - 1)))
						{
							strcat(str, ",");
						}
					}
					else
					{
						sprintf(tmp, "\"R[%d]\":[%d,%d]", i, *(WDev[count].AO_Regs + AORRevcur), *(WDev[count].AO_Regs + AORRevcur + 1));
						strcat(str, tmp);

						if ((i != (WDev[count].numberOfAO - 1)))
						{
							strcat(str, ",");
						}

						if (!bConnectionFlag)
						{
							bConnectionFlag = true;
						}

						g_bRev_Fail = false;
					}
					AORRevcur += 2;
					WDev[count].AO[i].bRevFin = true;
				}
				else
				{
					if (ctx != NULL)
					{
						pthread_mutex_lock(&pModbusMux);
						ret = modbus_set_slave(ctx, WDev[count].deviceID);
						if(ret > 0)
						{
							//printf("Flush %d byte(s) non-transmitted holding register data!\r\n");
						}
						ret = modbus_read_registers(ctx, WDev[count].AO[i].address, 1, (WDev[count].AO_Regs + AORRevcur));
						if (Modbus_Delay > 0)
						{
							usleep(Modbus_Delay * 1000);
						}
						pthread_mutex_unlock(&pModbusMux);
					}
					else
					{
						ret = -1;
					}

					if (ret == -1)
					{
						ModbusLog(g_loghandle, Error, "modbus_read_registers failed - %d - R[%d] Rev ERROR : %s",
							Rev_Fail_Num, i, modbus_strerror(errno));

						sprintf(tmp, "\"R[%d]\":\"%s\"", i, "FAIL");
						strcat(str, tmp);

						if ((i != (WDev[count].numberOfAO - 1)))
						{
							strcat(str, ",");
						}
					}
					else
					{
						sprintf(tmp, "\"R[%d]\":%d", i, *(WDev[count].AO_Regs + AORRevcur));
						strcat(str, tmp);

						if ((i != (WDev[count].numberOfAO - 1)))
						{
							strcat(str, ",");
						}

						if (!bConnectionFlag)
						{
							bConnectionFlag = true;
						}

						WDev[count].AO[i].Regs = *(WDev[count].AO_Regs + AORRevcur);
						g_bRev_Fail = false;
					}

					//printf("AO_Regs[%d] : %d\n",i,*(AO_Regs+AORRevcur));
					AORRevcur++;
					WDev[count].AO[i].bRevFin = true;
				}
				//sleep(2);
			}
		}

		if (WDev[count].numberOfDIB != 0)
		{
			uint8_t* pBlockStart = WDev[count].DI_Blocks;

			for (i = 0; i < WDev[count].numberOfDIB; i++)
			{
				char tmp[200];
				WDev[count].DIB[i].bRevFin = false;	//not in use actually
				if (ctx != NULL)
				{
					pthread_mutex_lock(&pModbusMux);
					modbus_set_slave(ctx, WDev[count].deviceID);
					ret = modbus_read_input_bits(ctx, WDev[count].DIB[i].address, WDev[count].DIB[i].length, pBlockStart);
					usleep(Modbus_Delay * 1000);
					pthread_mutex_unlock(&pModbusMux);
				}
				else
				{
					ret = -1;
				}

				if (ret == -1)
				{
					ModbusLog(g_loghandle, Error, "modbus_read_input_bits failed - %d - IBB[%d] Rev ERROR : %s", 
						Rev_Fail_Num, i, modbus_strerror(errno));

					sprintf(tmp, "\"IBB[%d]\":\"%s\"", i, "FAIL");
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfDIB - 1)) ||
						WDev[count].numberOfDOB != 0 ||
						WDev[count].numberOfAIB != 0 ||
						WDev[count].numberOfAOB != 0)
					{
						strcat(str, ",");
					}

				}
				else
				{
					sprintf(tmp, "\"IBB[%d]\":%d", i, *pBlockStart);
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfDIB - 1)) ||
						WDev[count].numberOfDOB != 0 ||
						WDev[count].numberOfAIB != 0 ||
						WDev[count].numberOfAOB != 0)
					{
						strcat(str, ",");
					}

					if (!bConnectionFlag)
					{
						bConnectionFlag = true;
					}

					g_bRev_Fail = false;

					for (int j = 0; j < WDev[count].DIB[i].length; j++)
					{
						WDev[count].DIB[i].Bits[j] = *(pBlockStart + j);
					}
					WDev[count].DIB[i].bRevFin = true;	//not in use actually
					//printf("DIB_Bits[%d] : %d\n",i,DIB_Bits[i]);
				}
				pBlockStart = pBlockStart + WDev[count].DIB[i].length;
				//sleep(2);
			}
		}

		if (WDev[count].numberOfDOB != 0)
		{
			uint8_t* pBlockStart = WDev[count].DO_Blocks;

			for (i = 0; i < WDev[count].numberOfDOB; i++)
			{
				char tmp[200];
				WDev[count].DOB[i].bRevFin = false;	//not in use actually
				if (ctx != NULL)
				{
					pthread_mutex_lock(&pModbusMux);
					modbus_set_slave(ctx, WDev[count].deviceID);
					ret = modbus_read_bits(ctx, WDev[count].DOB[i].address, WDev[count].DOB[i].length, pBlockStart);
					if (Modbus_Delay > 0)
					{
						usleep(Modbus_Delay * 1000);
					}
					pthread_mutex_unlock(&pModbusMux);
				}
				else
				{
					ret = -1;
				}

				if (ret == -1)
				{
					ModbusLog(g_loghandle, Error, "modbus_read_bits failed - %d - BB[%d] Rev ERROR : %s", 
						Rev_Fail_Num, i, modbus_strerror(errno));
					sprintf(tmp, "\"BB[%d]\":\"%s\"", i, "FAIL");
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfDOB - 1)) || WDev[count].numberOfAIB != 0 || WDev[count].numberOfAOB != 0)
					{
						strcat(str, ",");
					}
				}
				else
				{
					sprintf(tmp, "\"BB[%d]\":%d", i, *pBlockStart);
					strcat(str, tmp);
					if ((i != (WDev[count].numberOfDOB - 1)) || WDev[count].numberOfAIB != 0 || WDev[count].numberOfAOB != 0)
					{
						strcat(str, ",");
					}

					if (!bConnectionFlag)
					{
						bConnectionFlag = true;
					}

					g_bRev_Fail = false;

					for (int j = 0; j < WDev[count].DOB[i].length; j++)
					{
						WDev[count].DOB[i].Bits[j] = *(pBlockStart + j);
					}

					WDev[count].DOB[i].bRevFin = true;	//not in use actually
					//printf("DO_Blocks[%d] : %d\n", i, DO_Blocks[i]);
				}
				pBlockStart = pBlockStart + WDev[count].DOB[i].length;
				//sleep(2);
			}
		}

		if (WDev[count].numberOfAIB != 0)
		{
			//int AIRRevcur = 0;
			uint16_t* pBlockStart = WDev[count].AI_Blocks;

			for (i = 0; i < WDev[count].numberOfAIB; i++)
			{
				char tmp[200];
				WDev[count].AIB[i].bRevFin = false;

				if (ctx != NULL)
				{
					pthread_mutex_lock(&pModbusMux);

					modbus_set_slave(ctx, WDev[count].deviceID);
					//modbus_flush(ctx);
					ret = modbus_read_input_registers(ctx, WDev[count].AIB[i].address, WDev[count].AIB[i].length, pBlockStart);
					if (Modbus_Delay > 0)
					{
						usleep(Modbus_Delay * 1000);
					}

					pthread_mutex_unlock(&pModbusMux);
				}
				else
				{
					ret = -1;
				}

				if (ret == -1)
				{
					ModbusLog(g_loghandle, Error, "modbus_read_registers failed - %d - IRB[%d] Rev ERROR : %s", 
						Rev_Fail_Num, i, modbus_strerror(errno));

					sprintf(tmp, "\"IRB[%d]\":\"%s\"", i, "FAIL");
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfAIB - 1)))
					{
						strcat(str, ",");
					}
				}
				else
				{
					sprintf(tmp, "\"IRB[%d]\":%d", i, *pBlockStart);
					strcat(str, tmp);

					if ((i != (WDev[count].numberOfAIB - 1)))
					{
						strcat(str, ",");
					}

					if (!bConnectionFlag)
					{
						bConnectionFlag = true;
					}

					g_bRev_Fail = false;

					for (int j = 0; j < WDev[count].AIB[i].length; j++)
					{
						WDev[count].AIB[i].Regs[j] = *(pBlockStart + j);
					}

					WDev[count].AIB[i].bRevFin = true;
				}
				pBlockStart = pBlockStart + WDev[count].AIB[i].length;
			}
		}

		if (WDev[count].numberOfAOB != 0)
		{
			//int AORRevcur = 0;
			uint16_t* pBlockStart = WDev[count].AO_Blocks;

			for (i = 0; i < WDev[count].numberOfAOB; i++)
			{
				char tmp[200];
				WDev[count].AOB[i].bRevFin = false;

				if (ctx != NULL)
				{
					pthread_mutex_lock(&pModbusMux);

					modbus_set_slave(ctx, WDev[count].deviceID);
					//modbus_flush(ctx);
					ret = modbus_read_registers(ctx, WDev[count].AOB[i].address, WDev[count].AOB[i].length, pBlockStart);
					if (Modbus_Delay > 0)
					{
						usleep(Modbus_Delay * 1000);
					}

					pthread_mutex_unlock(&pModbusMux);
				}
				else
				{
					ret = -1;
				}

				if (ret == -1)
				{
					ModbusLog(g_loghandle, Error, "modbus_read_registers failed - %d - RB[%d] Rev ERROR : %s", 
						Rev_Fail_Num, i, modbus_strerror(errno));
					sprintf(tmp, "\"RB[%d]\":\"%s\"", i, "FAIL");
					strcat(str, tmp);
					if ((i != (WDev[count].numberOfAOB - 1)))
					{
						strcat(str, ",");
					}
				}
				else
				{
					sprintf(tmp, "\"RB[%d]\":%d", i, *pBlockStart);
					strcat(str, tmp);
					if ((i != (WDev[count].numberOfAOB - 1)))
					{
						strcat(str, ",");
					}

					if (!bConnectionFlag)
					{
						bConnectionFlag = true;
					}

					g_bRev_Fail = false;

					for (int j = 0; j < WDev[count].AOB[i].length; j++)
					{
						WDev[count].AOB[i].Regs[j] = *(pBlockStart + j);
					}

					WDev[count].AOB[i].bRevFin = true;
				}
				pBlockStart = pBlockStart + WDev[count].AOB[i].length;
			}
		}
	}

	strcat(str, "}");
	if (Modbus_Log)
	{
		pMRLog = fopen(MRLog_path, "a+");
		if (pMRLog == NULL)
		{
			ModbusLog(g_loghandle, Error, "Fail to Open MR.txt!!");
		}
		else
		{
			time_t current_time = time(NULL);
			char* c_time_string;

			if (current_time == ((time_t)-1))
			{
				fprintf(stderr, "Failure to obtain the current time.\n");
			}

			c_time_string = ctime(&current_time);

			if (c_time_string == NULL)
			{
				fprintf(stderr, "Failure to convert the current time.\n");
			}

			fprintf(pMRLog, "%d, %lld, %s, %s\n", Rev_Fail_Num, current_time, c_time_string, str);
			fclose(pMRLog);
		}
	}
	return true;
}

//--------------------------------------------------------------------------------------------------------------
//------------------------------------------------SensorInfo----------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

static bool IsSensorInfoListEmpty(sensor_info_list sensorInfoList)
{
	bool bRet = TRUE;
	sensor_info_node_t * curNode = NULL, *head = NULL;

	if (sensorInfoList == NULL)
		return bRet;

	head = sensorInfoList;
	curNode = head->next;

	if (curNode != NULL)
		bRet = FALSE;

	return bRet;
}

//---------------------------------------------------------------------------------------------------------------------
//------------------------------------------------CAPABILITY_GET_SET_UPLOAD-------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//Get Capability
static void GetCapability()
{
	char* result = NULL;
	MSG_CLASSIFY_T *t_Capability = NULL;
	int len = 0;

	t_Capability = CreateCapability();

	if (t_Capability)
	{
		result = IoT_PrintCapability(t_Capability);
		len = strlen(result);
		if (len != 0)
			g_sendcbf(&g_PluginInfo, modbus_get_capability_rep, result, strlen(result) + 1, NULL, NULL);

		IoT_ReleaseAll(t_Capability);
		t_Capability = NULL;
	}
	else
	{
		char * errorRepJsonStr = NULL;
		char errorStr[128];
		sprintf(errorStr, "Command(%d), Get capability error!", modbus_get_capability_req);
		int jsonStrlen = Parser_PackModbusError(errorStr, &errorRepJsonStr);
		if (jsonStrlen > 0 && errorRepJsonStr != NULL)
		{
			g_sendcbf(&g_PluginInfo, modbus_error_rep, errorRepJsonStr, strlen(errorRepJsonStr) + 1, NULL, NULL);
		}
		if (errorRepJsonStr)free(errorRepJsonStr);
	}

	if (result)
		free(result);
}


bool AddStringAttribute(MSG_ATTRIBUTE_T	*pAttr, MSG_CLASSIFY_T *pClass, char *pString, char* rwMode, double dwStatus)
{
	pAttr = MSG_AddJSONAttribute(pClass, "sv");
	if (pAttr)
	{
		if (MSG_SetStringValue(pAttr, pString, rwMode))
		{
			pAttr = MSG_AddJSONAttribute(pClass, "StatusCode");
			if (pAttr)
			{
				MSG_SetDoubleValue(pAttr, dwStatus, rwMode, NULL);
			}
		}
	}
	return true;
}

bool AddDoubleAttribute(MSG_ATTRIBUTE_T *pAttr, MSG_CLASSIFY_T *pClass, double dwValue, char* rwMode, char* pUnit, double dwStatus)
{
	pAttr = MSG_AddJSONAttribute(pClass, "v");
	if (pAttr)
	{
		if (MSG_SetDoubleValue(pAttr, dwValue, rwMode, pUnit))
		{
			pAttr = MSG_AddJSONAttribute(pClass, "StatusCode");
			if (pAttr)
				MSG_SetDoubleValue(pAttr, dwStatus, rwMode, NULL);
		}
	}
	return true;
}

bool AddBoolAttribute(MSG_ATTRIBUTE_T *pAttr, MSG_CLASSIFY_T *pClass, bool bValue, char* rwMode, double dwStatus)
{
	pAttr = MSG_AddJSONAttribute(pClass, "bv");
	if (pAttr)
	{
		if (MSG_SetBoolValue(pAttr, bValue, rwMode))
		{
			pAttr = MSG_AddJSONAttribute(pClass, "StatusCode");
			if (pAttr)
				MSG_SetDoubleValue(pAttr, dwStatus, rwMode, NULL);
		}
	}
	return true;
}


//Get Sensors' Data
static void GetSensorsDataEx(sensor_info_list sensorInfoList, char * pSessionID)
{
	int Num_Sensor = 1000; // to avoid exceeding 
	WISE_Sensor *sensors = (WISE_Sensor *)calloc(Num_Sensor, sizeof(WISE_Sensor));
	MSG_CLASSIFY_T  *pSensorInofList = NULL, *pEArray = NULL, *pSensor = NULL;
	MSG_CLASSIFY_T  *pRoot = NULL;
	MSG_ATTRIBUTE_T	*attr;

	char * repJsonStr = NULL;
	int count = 0;
	//char* result = NULL;
	bool bUAI_AO;

#pragma region IsSensorInfoListEmpty
	if (!IsSensorInfoListEmpty(sensorInfoList))
	{
		sensor_info_node_t *curNode = NULL;
		curNode = sensorInfoList->next;

#pragma region pRoot
		pRoot = MSG_CreateRoot();
		if (pRoot)
		{
			attr = MSG_AddJSONAttribute(pRoot, "sessionID");
			if (attr)
				MSG_SetStringValue(attr, pSessionID, NULL);

			pSensorInofList = MSG_AddJSONClassify(pRoot, "sensorInfoList", NULL, false);
			if (pSensorInofList)
				pEArray = MSG_AddJSONClassify(pSensorInofList, "e", NULL, true);

			while (curNode)
			{

#pragma region pSensorInofList
				if (pSensorInofList)
				{
#pragma region pEArray		
					if (pEArray)
					{
#pragma region pSensor
						pSensor = MSG_AddJSONClassify(pEArray, "sensor", NULL, false);
						if (pSensor)
						{
							attr = MSG_AddJSONAttribute(pSensor, "n");
							if (attr)
								MSG_SetStringValue(attr, curNode->sensorInfo.pathStr, NULL);
							if (count < Num_Sensor)
								Modbus_General_Node_Parser(curNode, sensors, count);

#pragma region MSG_Find_Sensor
							if (IoT_IsSensorExist(g_Capability, curNode->sensorInfo.pathStr))
							{
								if (attr)
								{
#pragma region Simulator
									if (bIsSimtor)
									{
#pragma region Platform-DI-DO-AI-AO
										int i = 0;
										if (strcmp(sensors[count].type, "Platform") == 0)
										{

											if (strcmp(sensors[count].name, "Protocol") == 0)
											{
												AddStringAttribute(attr, pSensor, Modbus_Protocol, "r", IOT_SGRC_SUCCESS);
											}
											if (strcmp(sensors[count].name, "Name") == 0)
											{
												AddStringAttribute(attr, pSensor, Device_Name, "r", IOT_SGRC_SUCCESS);
											}
											if (iTCP_RTU == 0)
											{
												if (strcmp(sensors[count].name, "ClientIP") == 0)
												{
													AddStringAttribute(attr, pSensor, Modbus_Clent_IP, "r", IOT_SGRC_SUCCESS);
												}
												if (strcmp(sensors[count].name, "ClientPort") == 0)
												{
													char temp[6];
													sprintf(temp, "%d", Modbus_Client_Port);
													AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
												}
												/*
												if (strcmp(sensors[count].name, "UnitID") == 0)
												{
													char temp[6];
													sprintf(temp, "%d", Modbus_UnitID);
													AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
												}
												*/
											}
											else if (iTCP_RTU == 1)
											{
												if (strcmp(sensors[count].name, "SlavePort") == 0)
												{
													AddStringAttribute(attr, pSensor, Modbus_Slave_Port, "r", IOT_SGRC_SUCCESS);
												}
												if (strcmp(sensors[count].name, "Baud") == 0)
												{
													AddDoubleAttribute(attr, pSensor, Modbus_Baud, "r", "bps", IOT_SGRC_SUCCESS);
												}
												if (strcmp(sensors[count].name, "Parity") == 0)
												{
													AddStringAttribute(attr, pSensor, Modbus_Parity, "r", IOT_SGRC_SUCCESS);
												}
												if (strcmp(sensors[count].name, "DataBits") == 0)
												{
													AddDoubleAttribute(attr, pSensor, Modbus_DataBits, "r", "bits", IOT_SGRC_SUCCESS);
												}
												if (strcmp(sensors[count].name, "StopBits") == 0)
												{
													AddDoubleAttribute(attr, pSensor, Modbus_StopBits, "r", "bits", IOT_SGRC_SUCCESS);
												}
												/*
												if (strcmp(sensors[count].name, "SlaveID") == 0)
												{
													char temp[6];
													sprintf(temp, "%d", Modbus_SlaveID);
													AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
												}
												*/
											}

											if (strcmp(sensors[count].name, "Connection") == 0)
											{
												AddBoolAttribute(attr, pSensor, true, "r", IOT_SGRC_SUCCESS);
											}
										}

										for (int devCount = 0; devCount < numberOfWDev; devCount++)
										{
											if (strcmp(sensors[count].type, "Discrete Inputs") == 0)
											{
												for (int i = 0; i < WDev[devCount].numberOfDI; i++)
												{
													if (strcmp(WDev[devCount].DI[i].name, sensors[count].name) == 0)
													{
														WDev[devCount].DI[i].Bits = rand() % 2;
														AddBoolAttribute(attr, pSensor, WDev[devCount].DI[i].Bits, "r", IOT_SGRC_SUCCESS);
													}
													if (i == WDev[devCount].numberOfDI)
													{
														AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
													}
												}
											}

											if (strcmp(sensors[count].type, "Coils") == 0)
											{
												// To do by Hank
												for (int i = 0; i < WDev[devCount].numberOfDO; i++)
												{
													if (strcmp(WDev[devCount].DO[i].name, sensors[count].name) == 0)
													{
														WDev[devCount].DO[i].Bits = rand() % 2;
														AddBoolAttribute(attr, pSensor, WDev[devCount].DO[i].Bits, "rw", IOT_SGRC_SUCCESS);
													}
													if (i == WDev[devCount].numberOfDO)
													{
														AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
													}
												}
											}
											if (strcmp(sensors[count].type, "Input Registers") == 0)
											{
												for (i = 0; i < WDev[devCount].numberOfAI; i++)
												{
													if (strcmp(WDev[devCount].AI[i].name, sensors[count].name) == 0)
													{
														while (true)
														{
															if (WDev[devCount].AI[i].max == 0)
															{
																WDev[devCount].AI_Regs[i] = 0;
																break;
															}
															WDev[devCount].AI[i].Regs = rand() % (int)WDev[devCount].AI[i].max;
															if (WDev[devCount].AI[i].Regs >= WDev[devCount].AI[i].min)
																break;
															else
																continue;
														}
														AddDoubleAttribute(attr, pSensor, WDev[devCount].AI[i].Regs, "r", WDev[devCount].AI[i].unit, IOT_SGRC_SUCCESS);
													}
													if (i == WDev[devCount].numberOfAI)
													{
														AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
													}
												}
											}
											if (strcmp(sensors[count].type, "Holding Registers") == 0)
											{
												for (i = 0; i < WDev[devCount].numberOfAO; i++)
												{
													if (strcmp(WDev[devCount].AO[i].name, sensors[count].name) == 0)
													{
														while (true)
														{
															if (WDev[devCount].AO[i].max == 0)
															{
																WDev[devCount].AO_Regs[i] = 0;
																break;
															}
															WDev[devCount].AO[i].Regs = rand() % (int)WDev[devCount].AO[i].max;
															if (WDev[devCount].AO[i].Regs >= WDev[devCount].AO[i].min)
																break;
															else
																continue;
														}
														AddDoubleAttribute(attr, pSensor, WDev[devCount].AO[i].Regs, "rw", WDev[devCount].AO[i].unit, IOT_SGRC_SUCCESS);
													}
													if (i == WDev[devCount].numberOfAO)
													{
														AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
													}
												}

											}
										}
#pragma endregion Platform-DI-DO-AI-AO
									}
#pragma endregion Simulator
									else
									{
#pragma region g_bRetrieve
										if (g_bRetrieve)
										{
#pragma region Platform-DI-DO-AI-AO
											int i = 0;
											if (strcmp(sensors[count].type, "Platform") == 0)
											{
												if (strcmp(sensors[count].name, "Protocol") == 0)
												{
													AddStringAttribute(attr, pSensor, Modbus_Protocol, "r", IOT_SGRC_SUCCESS);
												}

												if (strcmp(sensors[count].name, "Name") == 0)
												{
													AddStringAttribute(attr, pSensor, Device_Name, "r", IOT_SGRC_SUCCESS);
												}

												if (strcmp(sensors[count].name, "Description") == 0)
												{
													AddStringAttribute(attr, pSensor, strPluginDescription, "r", IOT_SGRC_SUCCESS);
												}

												if (strcmp(sensors[count].name, "Version") == 0)
												{
													AddStringAttribute(attr, pSensor, strPluginVersion, "r", IOT_SGRC_SUCCESS);
												}



												if (iTCP_RTU == 0)
												{
													if (strcmp(sensors[count].name, "ClientIP") == 0)
													{
														AddStringAttribute(attr, pSensor, Modbus_Clent_IP, "r", IOT_SGRC_SUCCESS);
													}
													if (strcmp(sensors[count].name, "ClientPort") == 0)
													{
														char temp[6];
														sprintf(temp, "%d", Modbus_Client_Port);
														AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
													}

													/*
													if (strcmp(sensors[count].name, "UnitID") == 0)
													{
														char temp[6];
														sprintf(temp, "%d", Modbus_UnitID);
														AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
													}
													*/
												}
												else if (iTCP_RTU == 1)
												{
													if (strcmp(sensors[count].name, "SlavePort") == 0)
													{
														AddStringAttribute(attr, pSensor, Modbus_Slave_Port, "r", IOT_SGRC_SUCCESS);
													}
													if (strcmp(sensors[count].name, "Baud") == 0)
													{
														AddDoubleAttribute(attr, pSensor, Modbus_Baud, "r", "bps", IOT_SGRC_SUCCESS);
													}
													if (strcmp(sensors[count].name, "Parity") == 0)
													{
														AddStringAttribute(attr, pSensor, Modbus_Parity, "r", IOT_SGRC_SUCCESS);
													}
													if (strcmp(sensors[count].name, "DataBits") == 0)
													{
														AddDoubleAttribute(attr, pSensor, Modbus_DataBits, "r", "bits", IOT_SGRC_SUCCESS);
													}
													if (strcmp(sensors[count].name, "StopBits") == 0)
													{
														AddDoubleAttribute(attr, pSensor, Modbus_StopBits, "r", "bits", IOT_SGRC_SUCCESS);
													}

													/*
													if (strcmp(sensors[count].name, "SlaveID") == 0)
													{
														char temp[6];
														sprintf(temp, "%d", Modbus_SlaveID);
														AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
													}
													*/
												}
												else
												{
													;//Not TCP or RTU
												}

												if (strcmp(sensors[count].name, "Connection") == 0)
												{
													AddBoolAttribute(attr, pSensor, bConnectionFlag, "r", IOT_SGRC_SUCCESS);
												}
											}

											for (int devCount = 0; devCount < numberOfWDev; devCount++)
											{

												if(strcmp(sensors[count].devicename, WDev[devCount].devicename) != 0)
												{
													continue;
												}

												if (iTCP_RTU == 0) // TCP
												{
													if (strcmp(sensors[count].type, "UnitID") == 0)
													{
														char temp[6];
														sprintf(temp, "%d", WDev[devCount].deviceID);
														AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
													}
												}
												else if(iTCP_RTU == 1) //RTU
												{
													if (strcmp(sensors[count].type, "SlaveID") == 0)
													{
														char temp[6];
														sprintf(temp, "%d", WDev[devCount].deviceID);
														AddStringAttribute(attr, pSensor, temp, "r", IOT_SGRC_SUCCESS);
													}
												}
												else
												{
													;//Not TCP or RTU
												}


												if (strcmp(sensors[count].type, "Discrete Inputs") == 0)
												{
													for (i = 0; i < WDev[devCount].numberOfDI; i++)
													{
														if (strcmp(WDev[devCount].DI[i].name, sensors[count].name) == 0)
														{
															WDev[devCount].DI[i].Bits = WDev[devCount].DI_Bits[i];
															AddBoolAttribute(attr, pSensor, WDev[devCount].DI[i].Bits, "r", IOT_SGRC_SUCCESS);
														}
														if (i == WDev[devCount].numberOfDI)
														{
															AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
														}
													}
												}
												if (strcmp(sensors[count].type, "Coils") == 0)
												{
													for (i = 0; i < WDev[devCount].numberOfDO; i++)
													{
														if (strcmp(WDev[devCount].DO[i].name, sensors[count].name) == 0)
														{
															WDev[devCount].DO[i].Bits = WDev[devCount].DO_Bits[i];
															AddBoolAttribute(attr, pSensor, WDev[devCount].DO[i].Bits, "rw", IOT_SGRC_SUCCESS);
														}
														if (i == WDev[devCount].numberOfDO)
														{
															AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
														}
													}
												}
												if (strcmp(sensors[count].type, "Input Registers") == 0)
												{
													int AIRcur = 0;
													bUAI_AO = true;
													uint16_t* pRegs;

													pRegs = WDev[devCount].AI_Regs;

													for (i = 0; i < WDev[devCount].numberOfAI; i++)
													{
														UpDataPrepare(&(WDev[devCount].AI[i]), bUAI_AO, &AIRcur, g_bRetrieve, &pRegs);

														if (strcmp(WDev[devCount].AI[i].name, sensors[count].name) == 0)
														{
															//AI[i].Regs=AI_Regs[i];
															/*
															attr = MSG_AddJSONAttribute(pSensor, "v");
															*/
															if (attr)
															{
																if (WDev[devCount].AI[i].sw_mode == 1 || WDev[devCount].AI[i].sw_mode == 2 || WDev[devCount].AI[i].sw_mode == 3 || WDev[devCount].AI[i].sw_mode == 4)
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AI[i].fv, "r", WDev[devCount].AI[i].unit, IOT_SGRC_SUCCESS);
																}
																else if (WDev[devCount].AI[i].sw_mode == 5 || WDev[devCount].AI[i].sw_mode == 6)
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AI[i].uiv, "r", WDev[devCount].AI[i].unit, IOT_SGRC_SUCCESS);
																}
																else if (WDev[devCount].AI[i].sw_mode == 7 || WDev[devCount].AI[i].sw_mode == 8)
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AI[i].iv, "r", WDev[devCount].AI[i].unit, IOT_SGRC_SUCCESS);
																}
																else
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AI[i].Regs*WDev[devCount].AI[i].precision, "r", WDev[devCount].AI[i].unit, IOT_SGRC_SUCCESS);
																}
															}
														}

														if (i == WDev[devCount].numberOfAI)
														{
															AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
														}


														if(WDev[devCount].AI[i].sw_mode < 1)
														{
															pRegs++;				
														}
														else
														{
															pRegs+=2;
														}
													}														
												}

												if (strcmp(sensors[count].type, "Holding Registers") == 0)
												{
													int AORcur = 0;
													bUAI_AO = false;
													uint16_t* pRegs;

													pRegs = WDev[devCount].AO_Regs;

													for (int i = 0; i < WDev[devCount].numberOfAO; i++)
													{
														UpDataPrepare(&(WDev[devCount].AO[i]), bUAI_AO, &AORcur, g_bRetrieve, &pRegs);

														if (strcmp(WDev[devCount].AO[i].name, sensors[count].name) == 0)
														{
															//AO[i].Regs=AO_Regs[i];
															/*
															attr = MSG_AddJSONAttribute(pSensor, "v");
															*/
															if (attr)
															{
																if (WDev[devCount].AO[i].sw_mode == 1 || WDev[devCount].AO[i].sw_mode == 2 || WDev[devCount].AO[i].sw_mode == 3 || WDev[devCount].AO[i].sw_mode == 4)
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AO[i].fv, "rw", WDev[devCount].AO[i].unit, IOT_SGRC_SUCCESS);
																}
																else if (WDev[devCount].AO[i].sw_mode == 5 || WDev[devCount].AO[i].sw_mode == 6)
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AO[i].uiv, "rw", WDev[devCount].AO[i].unit, IOT_SGRC_SUCCESS);
																}
																else if (WDev[devCount].AO[i].sw_mode == 7 || WDev[devCount].AO[i].sw_mode == 8)
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AO[i].iv, "rw", WDev[devCount].AO[i].unit, IOT_SGRC_SUCCESS);
																}
																else
																{
																	AddDoubleAttribute(attr, pSensor, WDev[devCount].AO[i].Regs*WDev[devCount].AO[i].precision, "rw", WDev[devCount].AO[i].unit, IOT_SGRC_SUCCESS);
																}
															}
														}
														if (i == WDev[devCount].numberOfAO)
														{
															AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
														}

														if(WDev[devCount].AO[i].sw_mode < 1)
														{
															pRegs++;				
														}
														else
														{
															pRegs+=2;
														}
													}
												}

												if (strcmp(sensors[count].type, "Discrete Inputs Block") == 0)
												{
													char bufferBlock[MAX_BLOCK_LENGTH] = {};
													char bufferValue[30];

													if (attr)
													{
														for (i = 0; i < WDev[devCount].numberOfDIB; i++)
														{
															if (strcmp(WDev[devCount].DIB[i].name, sensors[count].name) == 0)
															{
																for (int j = 0; j<WDev[devCount].DIB[i].length; j++)
																{
																	sprintf(bufferValue, "%d", WDev[devCount].DIB[i].Bits[j]);
																	strcat(bufferBlock, bufferValue);
																}
																strcat(bufferBlock, "\0");
																AddStringAttribute(attr, pSensor, bufferBlock, "r", IOT_SGRC_SUCCESS);
															}
															if (i == WDev[devCount].numberOfDIB)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
															}
														}
													}
												}

												if (strcmp(sensors[count].type, "Coils Block") == 0)
												{
													char bufferBlock[MAX_BLOCK_LENGTH] = {};
													char bufferValue[30];

													if (attr)
													{
														for (i = 0; i < WDev[devCount].numberOfDOB; i++)
														{
															if (strcmp(WDev[devCount].DOB[i].name, sensors[count].name) == 0)
															{
																for (int j = 0; j<WDev[devCount].DOB[i].length; j++)
																{
																	sprintf(bufferValue, "%d", WDev[devCount].DOB[i].Bits[j]);
																	strcat(bufferBlock, bufferValue);
																}
																strcat(bufferBlock, "\0");
																AddStringAttribute(attr, pSensor, bufferBlock, "r", IOT_SGRC_SUCCESS);
															}
															if (i == WDev[devCount].numberOfDOB)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
															}
														}
													}
												}

												if (strcmp(sensors[count].type, "Input Registers Block") == 0)
												{
													char bufferBlock[MAX_BLOCK_LENGTH] = {};
													char bufferValue[30];

													if (attr)
													{
														for (i = 0; i < WDev[devCount].numberOfAIB; i++)
														{
															if (strcmp(WDev[devCount].AIB[i].name, sensors[count].name) == 0)
															{
																for (int j = 0; j<WDev[devCount].AIB[i].length; j++)
																{
																	/*
																	sprintf(bufferValue, "%d", pWb[i].Regs[j]);
																	strcat(bufferBlock, bufferValue);
																	strcat(bufferBlock, " ");
																	*/					
																	sprintf(bufferValue, "%04X", WDev[devCount].AIB[i].Regs[j]);
																	strcat(bufferBlock, bufferValue);
																	strcat(bufferBlock, " ");
																}
																strcat(bufferBlock, "\0");
																AddStringAttribute(attr, pSensor, bufferBlock, "r", IOT_SGRC_SUCCESS);
															}
														}
													}
												}

												if (strcmp(sensors[count].type, "Holding Registers Block") == 0)
												{
													char bufferBlock[MAX_BLOCK_LENGTH] = {};
													char bufferValue[30];

													if (attr)
													{
														for (i = 0; i < WDev[devCount].numberOfAOB; i++)
														{
															if (strcmp(WDev[devCount].AOB[i].name, sensors[count].name) == 0)
															{
																for (int j = 0; j<WDev[devCount].AOB[i].length; j++)
																{
																	/*
																	sprintf(bufferValue, "%d", pWb[i].Regs[j]);
																	strcat(bufferBlock, bufferValue);
																	strcat(bufferBlock, " ");
																	*/					
																	sprintf(bufferValue, "%04X", WDev[devCount].AOB[i].Regs[j]);
																	strcat(bufferBlock, bufferValue);
																	strcat(bufferBlock, " ");
																}
																strcat(bufferBlock, "\0");
																AddStringAttribute(attr, pSensor, bufferBlock, "r", IOT_SGRC_SUCCESS);
															}
														}
													}
												}
											}
#pragma endregion Platform-DI-DO-AI-AO
										}
										else
										{
#pragma region FAIL	
											if (strcmp(sensors[count].type, "Platform") == 0 ||
												strcmp(sensors[count].type, "Discrete Inputs") == 0 ||			
												strcmp(sensors[count].type, "Input Registers") == 0
												)
											{
												AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "r", IOT_SGRC_FAIL);
											}
											else if(strcmp(sensors[count].type, "Coils") == 0 || strcmp(sensors[count].type, "Holding Registers") == 0)
											{
												AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "rw", IOT_SGRC_FAIL);
											}
											else
											{
											}
#pragma endregion FAIL
										}
#pragma endregion g_bRetrieve
									}
#pragma endregion Simulator
								}
							}
							else
							{
#pragma region NOT FOUND	
								if (strcmp(sensors[count].type, "Platform") == 0 ||
									strcmp(sensors[count].type, "Discrete Inputs") == 0 ||		
									strcmp(sensors[count].type, "Input Registers") == 0)
								{
									AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
								}
								else if (strcmp(sensors[count].type, "Coils") == 0 || strcmp(sensors[count].type, "Holding Registers") == 0)
								{
									AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
								}
								else
								{
									AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, NULL, IOT_SGRC_NOT_FOUND);
								}
#pragma endregion NOT FOUND
							}
#pragma endregion MSG_Find_Sensor

						}
#pragma endregion pSensor
					}
#pragma endregion pEArray
				}
#pragma endregion pSensorInofList
				curNode = curNode->next;
				count++;
			}
			{
				repJsonStr = IoT_PrintData(pRoot);
				printf("\nGET Handler_Data = %s\n", repJsonStr);
				printf("---------------------\n");

				if (repJsonStr != NULL)
				{
					g_sendcbf(&g_PluginInfo, modbus_get_sensors_data_rep, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
				}

				if (repJsonStr)
					free(repJsonStr);
			}
		}
#pragma endregion pRoot
	}
#pragma endregion IsSensorInfoListEmpty

	if (pRoot)
		MSG_ReleaseRoot(pRoot);

	if (sensors)
		free(sensors);
}

//Set Sensors' Data
static void SetSensorsDataEx(sensor_info_list sensorInfoList, char * pSessionID)
{
	int Num_Sensor = 1000; // to avoid exceeding 
	WISE_Sensor *sensors = (WISE_Sensor *)calloc(Num_Sensor, sizeof(WISE_Sensor));
	MSG_CLASSIFY_T  *pSensorInofList = NULL, *pEArray = NULL, *pSensor = NULL;
	MSG_CLASSIFY_T  *pRoot = NULL;
	MSG_ATTRIBUTE_T	*attr;

	char * repJsonStr = NULL;
	int sensorCount = 0;
	bool bVal = false;
	double dVal = 0;
	char *sVal = (char *)calloc(SET_STR_LENGTH, sizeof(char));
	bool bFormat = false;

#pragma region IsSensorInfoListEmpty
	if (!IsSensorInfoListEmpty(sensorInfoList))
	{
		sensor_info_node_t *curNode = NULL;
		curNode = sensorInfoList->next;
#pragma region pRoot
		pRoot = MSG_CreateRoot();
		if (pRoot)
		{
			attr = MSG_AddJSONAttribute(pRoot, "sessionID");
			if (attr)
				MSG_SetStringValue(attr, pSessionID, NULL);

			pSensorInofList = MSG_AddJSONClassify(pRoot, "sensorInfoList", NULL, false);
			if (pSensorInofList)
				pEArray = MSG_AddJSONClassify(pSensorInofList, "e", NULL, true);

			while (curNode)
			{
				bFormat = Modbus_Parser_Set_FormatCheck(curNode, &bVal, &dVal, sVal);
				if (bFormat)
					;
				else
					printf("\nFormat Error!!\n");

#pragma region pSensorInofList
				if (pSensorInofList)
				{
#pragma region pEArray		
					if (pEArray)
					{
#pragma region pSensor
						pSensor = MSG_AddJSONClassify(pEArray, "sensor", NULL, false);
						if (pSensor)
						{
							attr = MSG_AddJSONAttribute(pSensor, "n");
							if (attr)
								MSG_SetStringValue(attr, curNode->sensorInfo.pathStr, NULL);
							if (sensorCount < Num_Sensor)
								Modbus_General_Node_Parser(curNode, sensors, sensorCount);
#pragma region bFormat
							if (!bFormat)
							{
								if (strcmp(sensors[sensorCount].type, "Platform") == 0 || 
									strcmp(sensors[sensorCount].type, "Discrete Inputs") == 0 ||
									strcmp(sensors[sensorCount].type, "Input Registers") == 0 ||
									strcmp(sensors[sensorCount].type, "Discrete Inputs Block") == 0 ||
									strcmp(sensors[sensorCount].type, "Input Registers Block") == 0)
								{
									AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FORMAT_ERROR, "r", IOT_SGRC_FORMAT_ERROR);
								}

								if (strcmp(sensors[sensorCount].type, "Coils") == 0 ||
									strcmp(sensors[sensorCount].type, "Holding Registers") == 0 ||
									strcmp(sensors[sensorCount].type, "Coils Block") == 0 ||
									strcmp(sensors[sensorCount].type, "Holding Registers Block") == 0)
								{
									AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FORMAT_ERROR, "rw", IOT_SGRC_FORMAT_ERROR);
								}
							}
							else
							{
#pragma region MSG_Find_Sensor
								if (IoT_IsSensorExist(g_Capability, curNode->sensorInfo.pathStr))
								{
									if (attr)
									{
#pragma region bIsSimtor
										if (bIsSimtor)
										{
#pragma region Platform-DI-DO-AI-AO
											int i = 0;

											for (int devCount = 0; devCount < numberOfWDev; devCount++)
											{
												if (strcmp(WDev[devCount].devicename, sensors[sensorCount].devicename) != 0)
												{
													continue;
												}

												if (strcmp(sensors[sensorCount].type, "Platform") == 0)
												{
													AddStringAttribute(attr, pSensor, IOT_SGRC_STR_READ_ONLY, "r", IOT_SGRC_READ_ONLY);
												}

												for (int devCount = 0; devCount < numberOfWDev; devCount++)
												{
													if (strcmp(sensors[sensorCount].type, "Discrete Inputs") == 0 ||
														strcmp(sensors[sensorCount].type, "Input Registers") == 0 ||
														strcmp(sensors[sensorCount].type, "Discrete Inputs Block") == 0 ||
														strcmp(sensors[sensorCount].type, "Input Registers Block") == 0)
													{
														AddStringAttribute(attr, pSensor, IOT_SGRC_STR_READ_ONLY, "r", IOT_SGRC_READ_ONLY);
													}

													if (strcmp(sensors[sensorCount].type, "Coils") == 0)
													{
														for (i = 0; i < WDev[devCount].numberOfDO; i++)
														{
															if (strcmp(WDev[devCount].DO[i].name, sensors[sensorCount].name) == 0)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_SUCCESS, "rw", IOT_SGRC_SUCCESS);
															}
															if (i == WDev[devCount].numberOfDO)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
															}
														}
													}

													if (strcmp(sensors[sensorCount].type, "Holding Registers") == 0)
													{
														for (i = 0; i < WDev[devCount].numberOfAO; i++)
														{
															if (strcmp(WDev[devCount].AO[i].name, sensors[sensorCount].name) == 0)
															{
																if (dVal > WDev[devCount].AO[i].max || dVal < WDev[devCount].AO[i].min)
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_OUT_RANGE, "rw", IOT_SGRC_OUT_RANGE);
																}
																else
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_SUCCESS, "rw", IOT_SGRC_SUCCESS);
																}
															}
															if (i == WDev[devCount].numberOfAO)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
															}
														}
													}

													if (strcmp(sensors[sensorCount].type, "Coils Block") == 0)
													{
														// Not support
														AddStringAttribute(attr, pSensor, "Not Support", "r", IOT_SGRC_FAIL);
													}

													if (strcmp(sensors[sensorCount].type, "Holding Registers Block") == 0)
													{
														// Not support
														AddStringAttribute(attr, pSensor, "Not Support", "r", IOT_SGRC_FAIL);
													}
												}
											}
#pragma endregion Platform-DI-DO-AI-AO
										}
#pragma endregion bIsSimtor
										else
										{
#pragma region g_bRetrieve
											if (g_bRetrieve)
											{
#pragma region Platform-DI-DO-AI-AO
												int i = 0;

												for (int devCount = 0; devCount < numberOfWDev; devCount++)
												{
													if (strcmp(WDev[devCount].devicename, sensors[sensorCount].devicename) != 0)
													{
														continue;
													}

													if (strcmp(sensors[sensorCount].type, "Platform") == 0)
													{
														AddStringAttribute(attr, pSensor, IOT_SGRC_STR_READ_ONLY, "r", IOT_SGRC_READ_ONLY);
													}

													if (strcmp(sensors[sensorCount].type, "Discrete Inputs") == 0 ||
														strcmp(sensors[sensorCount].type, "Input Registers") == 0)
													{
														AddStringAttribute(attr, pSensor, IOT_SGRC_STR_READ_ONLY, "r", IOT_SGRC_READ_ONLY);
													}

													if (strcmp(sensors[sensorCount].type, "Coils") == 0)
													{
														for (i = 0; i < WDev[devCount].numberOfDO; i++)
														{
															if (strcmp(WDev[devCount].DO[i].name, sensors[sensorCount].name) == 0)
															{
																int ret = -1;
																pthread_mutex_lock(&pModbusMux);
																modbus_set_slave(ctx, WDev[devCount].deviceID);
																ret = modbus_write_bit(ctx, WDev[devCount].DO[i].address, bVal);
																if (Modbus_Delay > 0)
																	usleep(Modbus_Delay * 1000);
																pthread_mutex_unlock(&pModbusMux);
																if (ret != -1)
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_SUCCESS, "r", IOT_SGRC_SUCCESS);
																}
																else
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "rw", IOT_SGRC_FAIL);
																}
															}
															if (i == WDev[devCount].numberOfDO)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
															}
														}
													}

													if (strcmp(sensors[sensorCount].type, "Holding Registers") == 0)
													{
														for (i = 0; i < WDev[devCount].numberOfAO; i++)
														{
															if (strcmp(WDev[devCount].AO[i].name, sensors[sensorCount].name) == 0)
															{
																if (dVal > WDev[devCount].AO[i].max || dVal < WDev[devCount].AO[i].min)
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_OUT_RANGE, "rw", IOT_SGRC_OUT_RANGE);
																}
																else
																{
																	int ret = DownDataPrepare(&WDev[devCount].AO[i], WDev[devCount].deviceID, (float)dVal, (uint32_t)dVal, (int)dVal, g_bRetrieve);
																	if (ret != -1)
																	{
																		AddStringAttribute(attr, pSensor, IOT_SGRC_STR_SUCCESS, "rw", IOT_SGRC_SUCCESS);
																	}
																	else
																	{
																		AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "rw", IOT_SGRC_FAIL);
																	}
																}
															}
															if (i == WDev[devCount].numberOfAO)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
															}
														}
													}

													if (strcmp(sensors[sensorCount].type, "Coils Block") == 0)
													{
														for (i = 0; i < WDev[devCount].numberOfDOB; i++)
														{
															if (strcmp(WDev[devCount].DOB[i].name, sensors[sensorCount].name) == 0)
															{
																int ret = DownBitBlockPrepare(&WDev[devCount].DOB[i], WDev[devCount].deviceID, sVal, g_bRetrieve);
																if (ret != -1)
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_SUCCESS, "r", IOT_SGRC_SUCCESS);
																}
																else
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "rw", IOT_SGRC_FAIL);
																}
															}
															if (i == WDev[devCount].numberOfDOB)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
															}
														}
													}

													if (strcmp(sensors[sensorCount].type, "Holding Registers Block") == 0)
													{
														for (i = 0; i < WDev[devCount].numberOfAOB; i++)
														{
															if (strcmp(WDev[devCount].AOB[i].name, sensors[sensorCount].name) == 0)
															{
																int ret = DownRegisterBlockPrepare(&WDev[devCount].AOB[i], WDev[devCount].deviceID, sVal, g_bRetrieve);
																if (ret != -1)
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_SUCCESS, "rw", IOT_SGRC_SUCCESS);
																}
																else
																{
																	AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "rw", IOT_SGRC_FAIL);
																}
															}
															if (i == WDev[devCount].numberOfAOB)
															{
																AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
															}
														}
													}
												}
#pragma endregion Platform-DI-DO-AI-AO
											}
											else
											{
#pragma region FAIL	
												if (strcmp(sensors[sensorCount].type, "Platform") == 0 ||
													strcmp(sensors[sensorCount].type, "Discrete Inputs") == 0 ||
													strcmp(sensors[sensorCount].type, "Input Registers") == 0 ||
													strcmp(sensors[sensorCount].type, "Discrete Inputs Block") == 0 ||
													strcmp(sensors[sensorCount].type, "Input Registers Block") == 0)
												{
													AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "r", IOT_SGRC_FAIL);
												}

												if (strcmp(sensors[sensorCount].type, "Coils") == 0 ||
													strcmp(sensors[sensorCount].type, "Holding Registers") == 0 ||
													strcmp(sensors[sensorCount].type, "Coils Block") == 0 ||
													strcmp(sensors[sensorCount].type, "Holding Registers Block") == 0)
												{
													AddStringAttribute(attr, pSensor, IOT_SGRC_STR_FAIL, "rw", IOT_SGRC_FAIL);
												}
#pragma endregion FAIL
											}
#pragma endregion g_bRetrieve
										}
									}
								}
								else
								{
#pragma region NOT FOUND		
									if (strcmp(sensors[sensorCount].type, "Platform") == 0 ||
										strcmp(sensors[sensorCount].type, "Discrete Inputs") ==0 ||
										strcmp(sensors[sensorCount].type, "Input Registers") == 0 ||
										strcmp(sensors[sensorCount].type, "Discrete Inputs Block") == 0 ||
										strcmp(sensors[sensorCount].type, "Input Registers Block") == 0)
									{
										AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "r", IOT_SGRC_NOT_FOUND);
									}
									else if (strcmp(sensors[sensorCount].type, "Coils") == 0 ||
										strcmp(sensors[sensorCount].type, "Holding Registers") == 0 ||
										strcmp(sensors[sensorCount].type, "Coils Block") == 0 ||
										strcmp(sensors[sensorCount].type, "Holding Registers Block") == 0)
									{
										AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, "rw", IOT_SGRC_NOT_FOUND);
									}
									else
									{
										AddStringAttribute(attr, pSensor, IOT_SGRC_STR_NOT_FOUND, NULL, IOT_SGRC_NOT_FOUND);
									}
#pragma endregion NOT FOUND
								}
#pragma endregion MSG_Find_Sensor
							}
#pragma endregion bFormat
						}
#pragma endregion pSensor
					}
#pragma endregion pEArray
				}
#pragma endregion pSensorInofList								
				curNode = curNode->next;
				sensorCount++;
			}
			{
				repJsonStr = IoT_PrintData(pRoot);
				printf("\nSET Handler_Data = %s\n", repJsonStr);
				printf("---------------------\n");
				if (repJsonStr != NULL)
				{
					g_sendcbf(&g_PluginInfo, modbus_set_sensors_data_rep, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
				}
				if (repJsonStr)
					free(repJsonStr);
			}
		}
#pragma endregion pRoot
	}
#pragma endregion IsSensorInfoListEmpty

	if (pRoot)
		MSG_ReleaseRoot(pRoot);
	if (sensors)
		free(sensors);
	if (sVal)
		free(sVal);
}

//Upload All Sensors' Data
static void UploadAllSensorsData(bool bReport_Upload, void *args)
{
	//handler_context_t *pHandlerContex = (handler_context_t *)args;
	char *repJsonStr = NULL;
	MSG_CLASSIFY_T  *devCap = NULL;
	MSG_CLASSIFY_T  *modbusCap = NULL;
	MSG_ATTRIBUTE_T *attr = NULL;
	IoT_READWRITE_MODE mode;
	int i;
	bool bUAI_AO;

#pragma region UploadAllSensorsData
	printf("\nUploadAllSensorsData..........\n");

	if (!g_Capability)
		g_Capability = CreateCapability();

	if (g_Capability)
	{
		devCap = IoT_FindGroup(g_Capability, "Platform");
		if (devCap)
			attr = IoT_FindSensorNode(devCap, "Connection");
		if (attr)
		{
			mode = IoT_READONLY;

			if (bIsSimtor)
			{
				IoT_SetBoolValue(attr, true, mode);
			}
			else
			{
				if (attr)
					IoT_SetBoolValue(attr, bConnectionFlag, mode);
			}
		}

		for (int i = 0; i < numberOfWDev; i++)
		{
			if (WDev[i].numberOfDI != 0)
			{
				IoTSetBits(g_Capability, devCap, WDev[i].devicename, modbusCap, "Discrete Inputs", 
					attr, NULL, 0, &(WDev[i].DI), &(WDev[i].DI_Bits), WDev[i].numberOfDI, true);
				modbusCap = NULL;
			}

			if (WDev[i].numberOfDO != 0)
			{
				IoTSetBits(g_Capability, devCap, WDev[i].devicename, modbusCap, "Coils", 
					attr, NULL, 0, &(WDev[i].DO), &(WDev[i].DO_Bits), WDev[i].numberOfDO, false);
				modbusCap = NULL;
			}

			if (WDev[i].numberOfAI != 0)
			{
				IoTSetRegisters(g_Capability, devCap, WDev[i].devicename, modbusCap, "Input Registers", 
					attr, NULL, 0, &(WDev[i].AI), &(WDev[i].AI_Regs), WDev[i].numberOfAI, true);
				modbusCap = NULL;
			}

			if (WDev[i].numberOfAO != 0)
			{
				IoTSetRegisters(g_Capability, devCap, WDev[i].devicename, modbusCap, "Holding Registers", 
					attr, NULL, 0, &(WDev[i].AO), &(WDev[i].AO_Regs), WDev[i].numberOfAO, false);
				modbusCap = NULL;
			}

			if (WDev[i].numberOfDIB != 0)
			{
				IoTSetBitsBlock(g_Capability, devCap, WDev[i].devicename, modbusCap, "Discrete Inputs Block", 
					attr, NULL, 0, &(WDev[i].DIB), &(WDev[i].DI_Blocks), WDev[i].numberOfDIB, true);
				modbusCap = NULL;
			}

			if (WDev[i].numberOfDOB != 0)
			{
				IoTSetBitsBlock(g_Capability, devCap, WDev[i].devicename, modbusCap, "Coils Block", 
					attr, NULL, 0, &(WDev[i].DOB), &(WDev[i].DO_Blocks), WDev[i].numberOfDOB, false);
				modbusCap = NULL;
			}

			if (WDev[i].numberOfAIB != 0)
			{
				IoTSetRegistersBlock(g_Capability, devCap, WDev[i].devicename, modbusCap, "Input Registers Block", 
					attr, NULL, 0, &(WDev[i].AIB), &(WDev[i].AI_Blocks), WDev[i].numberOfAIB, true);
				modbusCap = NULL;
			}

			if (WDev[i].numberOfAOB != 0)
			{
				IoTSetRegistersBlock(g_Capability, devCap, WDev[i].devicename, modbusCap, "Holding Registers Block", 
					attr, NULL, 0, &(WDev[i].AOB), &(WDev[i].AO_Blocks), WDev[i].numberOfAOB, false);
				modbusCap = NULL;
			}
		}

		repJsonStr = IoT_PrintData(g_Capability);
		printf("\nALL Handler_Data = %s\n", repJsonStr);
		printf("---------------------\n");

		if (bReport_Upload)
		{
			if (g_sendreportcbf)
			{
				if (Modbus_Log)
				{
					pMSLog = fopen(MSLog_path, "a+");
					if (pMSLog == NULL)
						ModbusLog(g_loghandle, Error, "Fail to Open MS.txt!!");
					else
					{
						time_t current_time = time(NULL);
						char* c_time_string;

						if (current_time == ((time_t)-1))
							fprintf(stderr, "Failure to obtain the current time.\n");

						c_time_string = ctime(&current_time);

						if (c_time_string == NULL)
							fprintf(stderr, "Failure to convert the current time.\n");

						fprintf(pMSLog, "%lld, %s, %s\n", current_time, c_time_string, repJsonStr);
						fclose(pMSLog);
						free(c_time_string);
					}
				}
				g_sendreportcbf(&g_PluginInfo, repJsonStr, strlen(repJsonStr), NULL, NULL);
			}
		}
		else
		{
			if (g_sendcbf)
			{
				g_sendcbf(&g_PluginInfo, modbus_auto_upload_rep, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
			}
		}
		free(repJsonStr);
	}
#pragma endregion UploadAllSensorsData
}

//Distinguish Uploading All Data or Partial Data
static void UploadSensorsDataEx(char **Data_paths, int Data_num, bool Reply_All, bool bReport_Upload, void *args)
{
	int Num_Sensor = 1000; // to avoid exceeding 
	WISE_Sensor *sensors = (WISE_Sensor *)calloc(Num_Sensor, sizeof(WISE_Sensor));
	MSG_CLASSIFY_T *rootCap = IoT_CreateRoot((char*)strPluginName);
	MSG_CLASSIFY_T *devCap;
	MSG_CLASSIFY_T *modbusCap;
	MSG_CLASSIFY_T  *classify_Find = NULL;
	MSG_ATTRIBUTE_T *attr_Find = NULL;
	MSG_ATTRIBUTE_T* attr = NULL;
	IoT_READWRITE_MODE mode = IoT_READONLY;

	char * repJsonStr = NULL;
	int Check_Level = 0;
	bool Have_name = false;
	int count = 0;
	bool bUAI_AO;

	/*
	//-------------------------------------Customize
	char barcode[BARCODE_NUM]={""};
	//-------------------------------------Customize_end
	*/

	for (int i = 0; i < Num_Sensor; i++)
	{
		strcpy(sensors[i].handler, "");
		strcpy(sensors[i].type, "");
		strcpy(sensors[i].name, "");
	}
	if (Reply_All)
	{
		UploadAllSensorsData(bReport_Upload, args);
	}
	else
	{
		printf("UploadSensorsDataEx.........\n");
		for (count = 0; count < Data_num; count++)
#pragma region Data_paths
			if (Data_paths[count])
			{
				Modbus_General_Paths_Parser(Data_paths[count], sensors, count);
				if (strcmp(sensors[count].name, "") == 0)
					Have_name = false;
				else
					Have_name = true;

#pragma region g_Capability
				if (!g_Capability)
					g_Capability = CreateCapability();

				if (g_Capability)
				{
#pragma region DEF_HANDLER_NAME
					Check_Level = 0;
					if (strcmp(sensors[count].handler, strPluginName) == 0)
					{
						Check_Level++;
						classify_Find = IoT_FindGroup(g_Capability, sensors[count].type);
#pragma region classify_Find
						if (classify_Find)
						{
							Check_Level++;
							attr_Find = IoT_FindSensorNode(classify_Find, sensors[count].name);
							if (attr_Find)
								Check_Level++;

						}
#pragma endregion classify_Find

#pragma region Check_Level
						switch (Check_Level)
						{
						case 1:
#pragma region Check_Level=1
							break;
#pragma endregion Check_Level=1
						case 2:
#pragma region Check_Level=2
#pragma region Have_name
							if (!Have_name)
							{
								if (strcmp(sensors[count].type, "Platform") == 0)
								{
									char Client_Port[6];
									//char Client_UnitID[6];
									//char SlaveID[6];

									devCap = IoT_AddGroup(rootCap, "Platform");
									if (devCap)
									{
										mode = IoT_READONLY;
										attr = IoT_AddSensorNode(devCap, "Protocol");
										if (attr)
											IoT_SetStringValue(attr, Modbus_Protocol, mode);

										attr = IoT_AddSensorNode(devCap, "Name");
										if (attr)
											IoT_SetStringValue(attr, Device_Name, mode);

										if (iTCP_RTU == 0)
										{
											sprintf(Client_Port, "%d", Modbus_Client_Port);
											//sprintf(Client_UnitID, "%d", Modbus_UnitID);

											attr = IoT_AddSensorNode(devCap, "ClientIP");
											if (attr)
												IoT_SetStringValue(attr, Modbus_Clent_IP, mode);

											attr = IoT_AddSensorNode(devCap, "ClientPort");
											if (attr)
												IoT_SetStringValue(attr, Client_Port, mode);

											/*
											attr = IoT_AddSensorNode(devCap, "UnitID");
											if (attr)
												IoT_SetStringValue(attr, Client_UnitID, mode);
											*/
										}
										else if (iTCP_RTU == 1)
										{
											//sprintf(SlaveID, "%d", Modbus_SlaveID);

											attr = IoT_AddSensorNode(devCap, "SlavePort");
											if (attr)
												IoT_SetStringValue(attr, Modbus_Slave_Port, mode);

											attr = IoT_AddSensorNode(devCap, "Baud");
											if (attr)
												IoT_SetDoubleValue(attr, Modbus_Baud, mode, "bps");

											attr = IoT_AddSensorNode(devCap, "Parity");
											if (attr)
												IoT_SetStringValue(attr, Modbus_Parity, mode);

											attr = IoT_AddSensorNode(devCap, "DataBits");
											if (attr)
												IoT_SetDoubleValue(attr, Modbus_DataBits, mode, "bits");

											attr = IoT_AddSensorNode(devCap, "StopBits");
											if (attr)
												IoT_SetDoubleValue(attr, Modbus_StopBits, mode, "bits");

											/*
											attr = IoT_AddSensorNode(devCap, "SlaveID");
											if (attr)
												IoT_SetStringValue(attr, SlaveID, mode);
											*/
										}
										else
										{
											;//Not TCP or RTU
										}

										attr = IoT_AddSensorNode(devCap, "Connection");

										if (bIsSimtor)
										{
											IoT_SetBoolValue(attr, true, mode);
										}
										else
										{
											if (attr)
												IoT_SetBoolValue(attr, bConnectionFlag, mode);
										}
									}
								}
								else
								{
									for (int i = 0; i < numberOfWDev; i++)
									{
										if (strcmp(sensors[count].type, "Discrete Inputs") == 0)
										{
											IoTSetBits(rootCap, devCap, WDev[i].devicename, modbusCap, "Discrete Inputs", 
												attr, &sensors, count, &(WDev[i].DI), &(WDev[i].DI_Bits), WDev[i].numberOfDI, true);
										}
										else if (strcmp(sensors[count].type, "Coils") == 0)
										{
											IoTSetBits(rootCap, devCap, WDev[i].devicename, modbusCap, "Coils", 
												attr, &sensors, count, &(WDev[i].DO), &(WDev[i].DO_Bits), WDev[i].numberOfDO, false);
										}
										else if (strcmp(sensors[count].type, "Input Registers") == 0)
										{
											IoTSetRegisters(rootCap, devCap, WDev[i].devicename, modbusCap, "Input Registers", 
												attr, &sensors, count, &(WDev[i].AI), &(WDev[i].AI_Regs), WDev[i].numberOfAI, true);
										}
										else if (strcmp(sensors[count].type, "Holding Registers") == 0)
										{
											IoTSetRegisters(rootCap, devCap, WDev[i].devicename, modbusCap, "Holding Registers", 
												attr, &sensors, count, &(WDev[i].AO), &(WDev[i].AO_Regs), WDev[i].numberOfAO, false);
										}
										else if (strcmp(sensors[count].type, "Discrete Inputs Block") == 0)
										{
											IoTSetBitsBlock(rootCap, devCap, WDev[i].devicename, modbusCap, "Discrete Inputs Block", 
												attr, &sensors, count, &(WDev[i].DIB), &(WDev[i].DI_Blocks), WDev[i].numberOfDIB, true);
										}
										else if (strcmp(sensors[count].type, "Coils Block") == 0)
										{
											IoTSetBitsBlock(rootCap, devCap, WDev[i].devicename, modbusCap, "Coils Block", 
												attr, &sensors, count, &(WDev[i].DOB), &(WDev[i].DO_Blocks), WDev[i].numberOfDOB, false);
										}
										else if (strcmp(sensors[count].type, "Input Registers Block") == 0)
										{
											IoTSetRegistersBlock(rootCap, devCap, WDev[i].devicename, modbusCap, "Input Registers Block", 
												attr, &sensors, count, &(WDev[i].AIB), &(WDev[i].AI_Blocks), WDev[i].numberOfAIB, true);
										}
										else if (strcmp(sensors[count].type, "Holding Registers Block") == 0)
										{
											IoTSetRegistersBlock(rootCap, devCap, WDev[i].devicename, modbusCap, "Holding Registers Block", 
												attr, &sensors, count, &(WDev[i].AOB), &(WDev[i].AO_Blocks), WDev[i].numberOfAOB, false);
										}
									}
								}
							}
#pragma endregion Have_name
							break;
#pragma endregion Check_Level=2
						case 3:
#pragma region Check_Level=3
							if (strcmp(sensors[count].type, "Platform") == 0)
							{
								char Client_Port[6];
								//char Client_UnitID[6];
								char SlaveID[6];

								devCap = IoT_AddGroup(rootCap, "Platform");
								if (devCap)
								{
									mode = IoT_READONLY;
									if (strcmp(sensors[count].name, "Protocol") == 0)
									{
										attr = IoT_AddSensorNode(devCap, "Protocol");
										if (attr)
											IoT_SetStringValue(attr, Modbus_Protocol, mode);
									}
									else if (strcmp(sensors[count].name, "Name") == 0)
									{
										attr = IoT_AddSensorNode(devCap, "Name");
										if (attr)
											IoT_SetStringValue(attr, Device_Name, mode);
									}
									if (iTCP_RTU == 0)
									{
										sprintf(Client_Port, "%d", Modbus_Client_Port);
										//sprintf(Client_UnitID, "%d", Modbus_UnitID);
										if (strcmp(sensors[count].name, "ClientIP") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "ClientIP");
											if (attr)
												IoT_SetStringValue(attr, Modbus_Clent_IP, mode);
										}
										else if (strcmp(sensors[count].name, "ClientPort") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "ClientPort");
											if (attr)
												IoT_SetStringValue(attr, Client_Port, mode);
										}
										/*
										else if (strcmp(sensors[count].name, "UnitID") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "UnitID");
											if (attr)
												IoT_SetStringValue(attr, Client_UnitID, mode);
										}
										*/
									}
									else if (iTCP_RTU == 1)
									{
										//sprintf(SlaveID, "%d", Modbus_SlaveID);
										if (strcmp(sensors[count].name, "SlavePort") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "SlavePort");
											if (attr)
												IoT_SetStringValue(attr, Modbus_Slave_Port, mode);
										}
										else if (strcmp(sensors[count].name, "Baud") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "Baud");
											if (attr)
												IoT_SetDoubleValue(attr, Modbus_Baud, mode, "bps");
										}
										else if (strcmp(sensors[count].name, "Parity") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "Parity");
											if (attr)
												IoT_SetStringValue(attr, Modbus_Parity, mode);
										}
										else if (strcmp(sensors[count].name, "DataBits") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "DataBits");
											if (attr)
												IoT_SetDoubleValue(attr, Modbus_DataBits, mode, "bits");
										}
										else if (strcmp(sensors[count].name, "StopBits") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "StopBits");
											if (attr)
												IoT_SetDoubleValue(attr, Modbus_StopBits, mode, "bits");;
										}
										/*
										else if (strcmp(sensors[count].name, "SlaveID") == 0)
										{
											attr = IoT_AddSensorNode(devCap, "SlaveID");
											if (attr)
												IoT_SetStringValue(attr, SlaveID, mode);
										}
										*/
									}
									else
										;//Not TCP or RTU

									if (strcmp(sensors[count].name, "Connection") == 0)
									{
										attr = IoT_AddSensorNode(devCap, "Connection");

										if (bIsSimtor)
										{
											IoT_SetBoolValue(attr, true, mode);
										}
										else
										{
											if (attr)
												IoT_SetBoolValue(attr, bConnectionFlag, mode);
										}
									}
								}
							}

							for (int devCount = 0; devCount < numberOfWDev; devCount++)
							{

								if (strcmp(sensors[count].type, "Discrete Inputs") == 0)
								{
									IoTSetBits(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Discrete Inputs",
										attr, &sensors, count, &(WDev[devCount].DI), &(WDev[devCount].DI_Bits), WDev[devCount].numberOfDI, true);
								}
								else if (strcmp(sensors[count].type, "Coils") == 0)
								{
									IoTSetBits(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Coils", 
										attr, &sensors, count, &(WDev[devCount].DO), &(WDev[devCount].DO_Bits), WDev[devCount].numberOfDO, false);
								}
								else if (strcmp(sensors[count].type, "Input Registers") == 0)
								{
									IoTSetRegisters(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Input Registers", 
										attr, &sensors, count, &WDev[devCount].AI, &WDev[devCount].AI_Regs, WDev[devCount].numberOfAI, true);
								}
								else if (strcmp(sensors[count].type, "Holding Registers") == 0)
								{
									IoTSetRegisters(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Holding Registers", 
										attr, &sensors, count, &WDev[devCount].AO, &WDev[devCount].AO_Regs, WDev[devCount].numberOfAO, false);
								}
								else if (strcmp(sensors[count].type, "Discrete Inputs Block") == 0)
								{
									IoTSetBitsBlock(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Discrete Inputs Block", 
										attr, &sensors, count, &WDev[devCount].DIB, &WDev[devCount].DI_Blocks, WDev[devCount].numberOfDIB, true);
								}
								else if (strcmp(sensors[count].type, "Coils Block") == 0)
								{
									IoTSetBitsBlock(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Coils Block", 
										attr, &sensors, count, &WDev[devCount].DOB, &WDev[devCount].DO_Blocks, WDev[devCount].numberOfDOB, false);
								}
								else if (strcmp(sensors[count].type, "Input Registers Block") == 0)
								{
									IoTSetRegistersBlock(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Input Registers Block", 
										attr, &sensors, count, &WDev[devCount].AIB, &WDev[devCount].AI_Blocks, WDev[devCount].numberOfAIB, true);
								}
								else if (strcmp(sensors[count].type, "Holding Registers Block") == 0)
								{
									IoTSetRegistersBlock(rootCap, devCap, WDev[devCount].devicename, modbusCap, "Holding Registers Block", 
										attr, &sensors, count, &WDev[devCount].AOB, &WDev[devCount].AO_Blocks, WDev[devCount].numberOfAOB, false);
								}
							}
							break;
#pragma endregion Check_Level=3
						}
#pragma endregion Check_Level
					}
#pragma endregion DEF_HANDLER_NAME
				}
#pragma endregion g_Capability
			}
#pragma endregion Data_paths

		repJsonStr = IoT_PrintData(rootCap);
		printf("\nNOT ALL Handler_Data = %s\n", repJsonStr);
		printf("---------------------\n");

		if (bReport_Upload)
		{
			if (g_sendreportcbf)
			{
				if (Modbus_Log)
				{
					pMSLog = fopen(MSLog_path, "a+");
					if (pMSLog == NULL)
						ModbusLog(g_loghandle, Error, "Fail to Open MS.txt!!");
					else {
						time_t current_time = time(NULL);
						char* c_time_string;

						if (current_time == ((time_t)-1))
							fprintf(stderr, "Failure to obtain the current time.\n");

						c_time_string = ctime(&current_time);

						if (c_time_string == NULL)
							fprintf(stderr, "Failure to convert the current time.\n");

						fprintf(pMSLog, "%lld, %s, %s\n", current_time, c_time_string, repJsonStr);
						fclose(pMSLog);
					}
				}
				g_sendreportcbf(&g_PluginInfo, repJsonStr, strlen(repJsonStr), NULL, NULL);
			}
		}
		else
		{
			if (g_sendcbf)
				g_sendcbf(&g_PluginInfo, modbus_auto_upload_rep, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
		}
		free(repJsonStr);
	}

	if (sensors)
		free(sensors);

	if (rootCap)
		IoT_ReleaseAll(rootCap);
}

bool IoTSetDeviceInfo(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_ATTRIBUTE_T* attr, int indexWDev)
{
	bool retValue = false;
	//int AIORcur = 0;
	//WISE_Sensor* pWs = NULL;
	//WISE_Data* pWd = NULL;
	IoT_READWRITE_MODE mode = IoT_READONLY;
	char devID[6];

	devGroup = IoT_FindGroup(parentGroup, devGroupName);
	if (devGroup == NULL)
	{
		devGroup = IoT_AddGroup(parentGroup, devGroupName);
	}

	if (devGroup)
	{
		//devInfoCap = IoT_AddGroup(devCap, "Device Information");
		if (iTCP_RTU == 0)
		{
			attr = IoT_AddSensorNode(devGroup, "UnitID");
		}
		else
		{
			attr = IoT_AddSensorNode(devGroup, "SlaveID");
		}

		if (attr)
		{
			sprintf(devID, "%d", WDev[indexWDev].deviceID);
			IoT_SetStringValue(attr, devID, mode);
			retValue = true;
		}
	}
	return retValue;
}

bool IoTSetBits(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName, 
	MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Data **wdData, uint8_t** arrBits, int numBits, bool bUDI_DO)
{
	bool retValue = true;
	//int AIORcur = 0;
	WISE_Sensor* pWs = NULL;
	WISE_Data* pWd = NULL;
	IoT_READWRITE_MODE mode = IoT_READONLY;
	uint8_t* pArrBits = *arrBits;

	if (wsData != NULL)
	{
		pWs = *wsData;
	}

	if (wdData != NULL)
	{
		pWd = *wdData;
	}

	devGroup = IoT_FindGroup(parentGroup, devGroupName);
	if (devGroup == NULL)
	{
		devGroup = IoT_AddGroup(parentGroup, devGroupName);
	}

	modbusGroup = IoT_FindGroup(devGroup, modbusGroupName);
	if (modbusGroup == NULL)
	{
		modbusGroup = IoT_AddGroup(devGroup, modbusGroupName);
	}

	if (modbusGroup)
	{
		if (bUDI_DO == true)
		{
			mode = IoT_READONLY;
		}
		else
		{
			mode = IoT_READWRITE;
		}

		if (numBits != 0)
		{
			for (int i = 0; i < numBits; i++)
			{
				attr = IoT_FindSensorNode(modbusGroup, pWd[i].name);
				if (!attr)
				{
					attr = IoT_AddSensorNode(modbusGroup, pWd[i].name);
				}

				if (bIsSimtor)
				{
					pArrBits[i] = rand() % 2;
				}
				else
				{
					//if(!g_bRetrieve)
					//	pArrBits[i]=false;
				}

				if (attr)
				{
					pWd[i].Bits = pArrBits[i];
					IoT_SetBoolValue(attr, (bool)pWd[i].Bits, mode);
				}
			}
			attr = NULL;
		}
	}
	return retValue;
}

bool IoTSetBitsBlock(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName, MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Block **wbData, uint8_t** arrBits, int numBlocks, bool bUDI_DO)
{
	bool retValue = true;
	int DIORcur = 0;
	WISE_Sensor* pWs = NULL;
	WISE_Block* pWb = NULL;
	IoT_READWRITE_MODE mode = IoT_READONLY;
	uint8_t* pArrBits = *arrBits;
	//uint8_t tempValue;
	char bufferBlock[MAX_BLOCK_LENGTH] = {};
	char bufferValue[30];

	if (wsData != NULL)
	{
		pWs = *wsData;
	}

	if (wbData != NULL)
	{
		pWb = *wbData;
	}

	devGroup = IoT_FindGroup(parentGroup, devGroupName);
	if (devGroup == NULL)
	{
		devGroup = IoT_AddGroup(parentGroup, devGroupName);
	}

	modbusGroup = IoT_FindGroup(devGroup, modbusGroupName);
	if (modbusGroup == NULL)
	{
		modbusGroup = IoT_AddGroup(devGroup, modbusGroupName);
	}

	if (modbusGroup)
	{
		if (bUDI_DO == true)
		{
			mode = IoT_READONLY;
		}
		else
		{
			mode = IoT_READWRITE;
		}

		for (int i = 0; i < numBlocks; i++)
		{
			strcpy(bufferBlock, "");

			if (pWs != NULL)
			{
				if (strcmp(pWs[wsCount].name, pWb[i].name) != 0)
				{
					return true;
				}
			}

			if (bIsSimtor)
			{
				pArrBits[i] = rand() % 2;
			}
			else
			{
				//if(!g_bRetrieve)
				//	pArrBits[i]=false;
			}

			if (modbusGroup != NULL)
			{
				attr = IoT_FindSensorNode(modbusGroup, pWb[i].name);
				if (!attr)
				{
					attr = IoT_AddSensorNode(modbusGroup, pWb[i].name);
				}
			}

			if (attr)
			{
				for (int j = 0; j<pWb[i].length; j++)
				{
					sprintf(bufferValue, "%d", pWb[i].Bits[j]);
					strcat(bufferBlock, bufferValue);
					DIORcur++;
				}
				strcat(bufferBlock, "\0");
				IoT_SetStringValue(attr, bufferBlock, mode);
			}
		}
		attr = NULL;
	}
	return retValue;
}

bool IoTSetRegisters(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName, 
	MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Data **wdData, uint16_t** arrRegs, int numRegs, bool bUAI_AO)
{
	bool retValue = true;
	int AIORcur = 0;
	WISE_Sensor* pWs = NULL;
	WISE_Data* pWd = NULL;
	IoT_READWRITE_MODE mode = IoT_READONLY;
	uint16_t* pArrRegs = *arrRegs;

	if (wsData != NULL)
	{
		pWs = *wsData;
	}

	if (wdData != NULL)
	{
		pWd = *wdData;
	}

	devGroup = IoT_FindGroup(parentGroup, devGroupName);
	if (devGroup == NULL)
	{
		devGroup = IoT_AddGroup(parentGroup, devGroupName);
	}

	modbusGroup = IoT_FindGroup(devGroup, modbusGroupName);
	if (modbusGroup == NULL)
	{
		modbusGroup = IoT_AddGroup(devGroup, modbusGroupName);
	}

	if (modbusGroup)
	{
		if (bUAI_AO == true)
		{
			mode = IoT_READONLY;
		}
		else
		{
			mode = IoT_READWRITE;
		}

		for (int i = 0; i < numRegs; i++)
		{
			if (pWs != NULL)
			{
				if (strcmp(pWs[wsCount].name, pWd[i].name) != 0)
				{
					return true;
				}
			}

			if (bIsSimtor)
			{
				while (true)
				{
					if (pWd[i].max == 0)
					{
						pArrRegs[i] = 0;
						break;
					}
					pArrRegs[i] = rand() % (int)pWd[i].max / pWd[i].precision;
					if (pArrRegs[i] >= pWd[i].min)
						break;
					else
						continue;
				}
			}
			else
			{
				//if(!g_bRetrieve)
				//	pArrRegs[i]=0;
			}

			if (modbusGroup != NULL)
			{
				attr = IoT_AddSensorNode(modbusGroup, pWd[i].name);
			}

			if (attr)
			{
				UpDataPrepare(pWd, bUAI_AO, &AIORcur, g_bRetrieve, arrRegs);

				if (strcmp(pWd[i].conversion, "") == 0)
				{
					if (pWd[i].sw_mode == 1 || pWd[i].sw_mode == 2 || pWd[i].sw_mode == 3 || pWd[i].sw_mode == 4)
					{
						IoT_SetDoubleValueWithMaxMin(attr, pWd[i].fv*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
					else if (pWd[i].sw_mode == 5 || pWd[i].sw_mode == 6)
					{
						IoT_SetDoubleValueWithMaxMin(attr, pWd[i].uiv*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
					else if (pWd[i].sw_mode == 7 || pWd[i].sw_mode == 8)
					{
						IoT_SetDoubleValueWithMaxMin(attr, pWd[i].iv*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
					else
					{
						IoT_SetDoubleValueWithMaxMin(attr, pWd[i].Regs*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
				}
				else
				{
					if (pWd[i].sw_mode == 1 || pWd[i].sw_mode == 2 || pWd[i].sw_mode == 3 || pWd[i].sw_mode == 4)
					{
						float valConv = LuaConversion(pWd[i].fv, pWd[i].conversion);
						IoT_SetDoubleValueWithMaxMin(attr, valConv*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
					else if (pWd[i].sw_mode == 5 || pWd[i].sw_mode == 6)
					{
						uint32_t valConv = LuaConversion(pWd[i].uiv, pWd[i].conversion);
						IoT_SetDoubleValueWithMaxMin(attr, valConv*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
					else if (pWd[i].sw_mode == 7 || pWd[i].sw_mode == 8)
					{
						int valConv = LuaConversion(pWd[i].iv, pWd[i].conversion);
						IoT_SetDoubleValueWithMaxMin(attr, valConv*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
					else
					{
						double valConv = LuaConversion(pWd[i].Regs, pWd[i].conversion);
						IoT_SetDoubleValueWithMaxMin(attr, valConv*pWd[i].precision, mode, pWd[i].max, pWd[i].min, pWd[i].unit);
					}
				}
			}
		}
	}
	return retValue;
}

bool IoTSetRegistersBlock(MSG_CLASSIFY_T *parentGroup, MSG_CLASSIFY_T *devGroup, char* devGroupName, MSG_CLASSIFY_T *modbusGroup, char* modbusGroupName, MSG_ATTRIBUTE_T* attr, WISE_Sensor** wsData, int wsCount, WISE_Block **wbData, uint16_t** arrBlocks, int numBlocks, bool bUAI_AO)
{
	bool retValue = true;
	int AIORcur = 0;
	WISE_Sensor* pWs = NULL;
	WISE_Block* pWb = NULL;
	IoT_READWRITE_MODE mode = IoT_READONLY;
	uint16_t* pArrBlocks = *arrBlocks;
	char bufferBlock[MAX_BLOCK_LENGTH] = {};
	char bufferValue[30];
	//uint16_t tempValue;

	if (wsData != NULL)
	{
		pWs = *wsData;
	}

	if (wbData != NULL)
	{
		pWb = *wbData;
	}

	devGroup = IoT_FindGroup(parentGroup, devGroupName);
	if (devGroup == NULL)
	{
		devGroup = IoT_AddGroup(parentGroup, devGroupName);
	}

	modbusGroup = IoT_FindGroup(devGroup, modbusGroupName);
	if (modbusGroup == NULL)
	{
		modbusGroup = IoT_AddGroup(devGroup, modbusGroupName);
	}

	if (modbusGroup)
	{
		if (bUAI_AO == true)
		{
			mode = IoT_READONLY;
		}
		else
		{
			mode = IoT_READWRITE;
		}

		for (int i = 0; i < numBlocks; i++)
		{
			strcpy(bufferBlock, "");

			if (pWb[i].bRevFin != true)
			{
				continue;
			}

			if (pWs != NULL)
			{
				if (strcmp(pWs[wsCount].name, pWb[i].name) != 0)
				{
					return true;
				}
			}

			if (bIsSimtor)
			{
				// TBD
				//while (true)
				//{
				//	if (pWd[i].max == 0)
				//	{
				//		pArrBlocks[i] = 0;
				//		break;
				//	}
				//	pArrBlocks[i] = rand() % (int)pWd[i].max / pWd[i].precision;
				//	if (pArrBlocks[i] >= pWd[i].min)
				//		break;
				//	else
				//		continue;
				//}
			}
			else
			{
				//if(!g_bRetrieve)
				//	pArrRegs[i]=0;
			}

			if (modbusGroup != NULL)
			{
				attr = IoT_AddSensorNode(modbusGroup, pWb[i].name);
			}

			if (attr)
			{
				for (int j = 0; j<pWb[i].length; j++)
				{
					/*
					sprintf(bufferValue, "%d", pWb[i].Regs[j]);
					strcat(bufferBlock, bufferValue);
					strcat(bufferBlock, " ");
					*/					

					sprintf(bufferValue, "%04X", pWb[i].Regs[j]);
					strcat(bufferBlock, bufferValue);
					strcat(bufferBlock, " ");

					AIORcur++;
				}
				strcat(bufferBlock, "\0");
				IoT_SetStringValue(attr, bufferBlock, mode);
			}
		}
	}
	return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------Threshold----------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
static void* ThresholdDeleteThreadStart(void *args);
static void* ThresholdSetThreadStart(void *args);
//-------------------------Deletion Part------------------------
int Parser_PackDelAllThrRep(char * repStr, char ** outputStr)
{
	char * out = NULL;
	int outLen = 0;
	cJSON *pSUSICommDataItem = NULL;
	if (repStr == NULL || outputStr == NULL) return outLen;
	pSUSICommDataItem = cJSON_CreateObject();

	cJSON_AddStringToObject(pSUSICommDataItem, MODBUS_DEL_ALL_THR_REP, repStr);

	out = cJSON_PrintUnformatted(pSUSICommDataItem);
	outLen = strlen(out) + 1;
	*outputStr = (char *)(malloc(outLen));
	memset(*outputStr, 0, outLen);
	strcpy(*outputStr, out);
	cJSON_Delete(pSUSICommDataItem);
	printf("%s\n", out);
	free(out);
	return outLen;
}

static void ModbusWhenDelThrCheckNormal(modbus_threshold_list thrItemList, char ** checkMsg, unsigned int bufLen)
{
	if (NULL == thrItemList || NULL == (char*)(*checkMsg)) return;
	{
		modbus_threshold_list curThrItemList = thrItemList;
		modbus_threshold_node * curThrItemNode = curThrItemList->next;
		char tmpMsg[256] = { 0 };
		while (curThrItemNode)
		{
			memset(tmpMsg, 0, sizeof(tmpMsg));
			if (curThrItemNode->info.isEnable && !curThrItemNode->info.isNormal)
			{
				curThrItemNode->info.isNormal = true;
				sprintf(tmpMsg, "%s %s", curThrItemNode->info.name, DEF_NOR_EVENT_STR);
			}

			if (strlen(tmpMsg))
			{
				if (bufLen < strlen(*checkMsg) + strlen(tmpMsg) + 16)
				{
					int newLen = strlen(*checkMsg) + strlen(tmpMsg) + 2 * 1024;
					*checkMsg = (char*)realloc(*checkMsg, newLen);
				}
				if (strlen(*checkMsg))
					sprintf(*checkMsg, "%s;%s", *checkMsg, tmpMsg);
				else
					sprintf(*checkMsg, "%s", tmpMsg);
			}
			curThrItemNode = curThrItemNode->next;
		}
	}
}

void DeleteThreshold()
{
	g_ThresholdDeleteContex.isThreadRunning = true;

	if (pthread_create(&g_ThresholdDeleteContex.threadHandler, NULL, ThresholdDeleteThreadStart, NULL) != 0)
	{
		g_ThresholdDeleteContex.isThreadRunning = false;
		printf("> start ThresholdDelete thread failed!\r\n");
	}
}

//-------------------------Setting Part------------------------
static int DeleteAllModbusThrItemNode(modbus_threshold_list thrList)
{
	int iRet = -1;
	modbus_threshold_node * delNode = NULL, *head = NULL;
	if (thrList == NULL) return iRet;
	head = thrList;

	delNode = head->next;
	while (delNode)
	{
		head->next = delNode->next;
		if (delNode->info.checkSrcValList.head)
		{
			check_value_node * frontValueNode = delNode->info.checkSrcValList.head;
			check_value_node * delValueNode = frontValueNode->next;
			while (delValueNode)
			{
				frontValueNode->next = delValueNode->next;
				free(delValueNode);
				delValueNode = frontValueNode->next;
			}

			free(delNode->info.checkSrcValList.head);
			delNode->info.checkSrcValList.head = NULL;
		}
		if (delNode->info.name != NULL)
			free(delNode->info.name);
		free(delNode);
		delNode = head->next;
	}

	iRet = 0;
	return iRet;
}

static void DestroyThresholdList(modbus_threshold_list thrList)
{
	if (NULL == thrList) return;
	DeleteAllModbusThrItemNode(thrList);
	free(thrList);
	thrList = NULL;
}

int Ack_SetThreshold(char *repMsg, char **repJsonStr) {

	char * out = NULL;
	int outLen = 0;
	cJSON *pSUSICommDataItem = NULL;
	if (repMsg == NULL || repJsonStr == NULL) return outLen;
	pSUSICommDataItem = cJSON_CreateObject();

	cJSON_AddStringToObject(pSUSICommDataItem, "setThrRep", repMsg);

	out = cJSON_PrintUnformatted(pSUSICommDataItem);
	outLen = strlen(out) + 1;
	*repJsonStr = (char *)(malloc(outLen));
	memset(*repJsonStr, 0, outLen);
	strcpy(*repJsonStr, out);
	cJSON_Delete(pSUSICommDataItem);
	printf("%s\n", out);
	free(out);
	return outLen;
}

modbus_threshold_list CreateThresholdList()
{
	modbus_threshold_node * head = NULL;
	head = (modbus_threshold_node *)malloc(sizeof(modbus_threshold_node));
	if (head)
	{
		memset(head, 0, sizeof(modbus_threshold_node));
		head->next = NULL;
		head->info.name = NULL;
		head->info.isValid = false;
		head->info.isEnable = false;
		head->info.maxThr = DEF_INVALID_VALUE;
		head->info.minThr = DEF_INVALID_VALUE;
		head->info.thrType = DEF_THR_UNKNOW_TYPE;
		head->info.lastingTimeS = DEF_INVALID_TIME;
		head->info.intervalTimeS = DEF_INVALID_TIME;
		head->info.checkRetValue = DEF_INVALID_VALUE;
		head->info.checkSrcValList.head = NULL;
		head->info.checkSrcValList.nodeCnt = 0;
		head->info.checkType = ck_type_unknow;
		head->info.repThrTime = DEF_INVALID_VALUE;
		head->info.isNormal = true;
	}
	return head;
}

bool ParserThresholdInfo(cJSON * jsonObj, threshold_info *pThrItemInfo, bool *nIsValidPnt)
{
	bool bRet = false;
	bool *nIsValid = nIsValidPnt;
	bool allThrFlag = false;

	if (jsonObj == NULL || pThrItemInfo == NULL) return bRet;
	{
		cJSON * pSubItem = NULL;
		pSubItem = jsonObj;
		if (pSubItem)
		{
			pSubItem = cJSON_GetObjectItem(jsonObj, MODBUS_THR_N);
			if (pSubItem)
			{
				pThrItemInfo->name = (char *)calloc(1, strlen(pSubItem->valuestring) + 1);
				strcpy(pThrItemInfo->name, pSubItem->valuestring);
			}
			else
			{
				pThrItemInfo->maxThr = DEF_INVALID_VALUE;
			}

			pThrItemInfo->isEnable = 0;
			pSubItem = cJSON_GetObjectItem(jsonObj, MODBUS_THR_ENABLE);
			if (pSubItem)
			{
				if (!strcasecmp(pSubItem->valuestring, "true"))
				{
					pThrItemInfo->isEnable = 1;
				}
			}

			pSubItem = cJSON_GetObjectItem(jsonObj, MODBUS_THR_MAX);
			if (pSubItem)
			{
				pThrItemInfo->maxThr = (float)pSubItem->valuedouble;
			}
			else
			{
				pThrItemInfo->maxThr = DEF_INVALID_VALUE;
			}

			pSubItem = cJSON_GetObjectItem(jsonObj, MODBUS_THR_MIN);
			if (pSubItem)
			{
				pThrItemInfo->minThr = (float)pSubItem->valuedouble;
			}
			else
			{
				pThrItemInfo->minThr = DEF_INVALID_VALUE;
			}

			pSubItem = cJSON_GetObjectItem(jsonObj, MODBUS_THR_TYPE);
			if (pSubItem)
			{
				pThrItemInfo->thrType = pSubItem->valueint;
			}
			else
			{
				pThrItemInfo->thrType = DEF_THR_UNKNOW_TYPE;
			}

			pSubItem = cJSON_GetObjectItem(jsonObj, MODBUS_THR_LTIME);
			if (pSubItem)
			{
				pThrItemInfo->lastingTimeS = pSubItem->valueint;
			}
			else
			{
				pThrItemInfo->lastingTimeS = DEF_INVALID_TIME;
			}

			pSubItem = cJSON_GetObjectItem(jsonObj, MODBUS_THR_ITIME);
			if (pSubItem)
			{
				pThrItemInfo->intervalTimeS = pSubItem->valueint;
			}
			else
			{
				pThrItemInfo->intervalTimeS = DEF_INVALID_TIME;
			}

			//pThrItemInfo->checkRetValue.vi = DEF_INVALID_VALUE;
			pThrItemInfo->checkRetValue = DEF_INVALID_VALUE;
			pThrItemInfo->checkSrcValList.head = NULL;
			pThrItemInfo->checkSrcValList.nodeCnt = 0;
			pThrItemInfo->checkType = ck_type_avg;
			pThrItemInfo->isValid = true;
			pThrItemInfo->isNormal = 1;
			pThrItemInfo->repThrTime = 0;

			bRet = true;
		}
	}
	*nIsValid = true;//´Ë²ÎÊý¿ÉÉ¾³ý;
	return bRet;
}

bool ParseThreshold(modbus_threshold_list Threshold_List)
{
	bool bRet = false;
	if (Threshold_Data == NULL || Threshold_List == NULL) return bRet;
	{
		cJSON * root = NULL;
		root = cJSON_Parse(Threshold_Data);
		if (root)
		{
			cJSON * commDataItem = cJSON_GetObjectItem(root, MODBUS_JSON_ROOT_NAME);
			if (commDataItem)
			{
				cJSON * info = NULL;
				cJSON * child = commDataItem->child;

				while (child->type != cJSON_Array)
				{
					child = child->next;
				}

				if (child->type == cJSON_Array)
				{
					info = child;
					if (info)
					{
						modbus_threshold_node * head = Threshold_List;
						int nCount = cJSON_GetArraySize(info);
						int i = 0;
						cJSON * subItem = NULL;
						for (i = 0; i < nCount; i++)
						{
							subItem = cJSON_GetArrayItem(info, i);
							if (subItem)
							{
								bool nIsValid = false;
								modbus_threshold_node * pThrItemNode = NULL;
								pThrItemNode = (modbus_threshold_node *)malloc(sizeof(modbus_threshold_node));
								memset(pThrItemNode, 0, sizeof(modbus_threshold_node));
								if (ParserThresholdInfo(subItem, &pThrItemNode->info, &nIsValid))//in in out
								{
									if (nIsValid)
									{
										pThrItemNode->next = head->next;
										head->next = pThrItemNode;
									}
									else
									{
										free(pThrItemNode);
										pThrItemNode = NULL;
									}
								}
								else
								{
									free(pThrItemNode);
									pThrItemNode = NULL;
								}
							}
						}
						bRet = true;
					}
				}
			}
		}
		if (Threshold_Data != NULL)
		{
			free(Threshold_Data);
			Threshold_Data = NULL;
		}
		cJSON_Delete(root);
	}
	return bRet;
}

int Parser_PackThrCheckRep(modbus_thr_rep_info * pThrCheckRep, char ** outputStr)
{
	char * out = NULL;
	int outLen = 0;
	cJSON *pSUSICommDataItem = NULL;
	if (pThrCheckRep == NULL || outputStr == NULL) return outLen;
	pSUSICommDataItem = cJSON_CreateObject();

	if (pThrCheckRep->isTotalNormal)
	{
		cJSON_AddStringToObject(pSUSICommDataItem, "subtype", "THRESHOLD_CHECK_INFO");
		cJSON_AddStringToObject(pSUSICommDataItem, MODBUS_THR_CHECK_STATUS, "True");
	}
	else
	{
		cJSON_AddStringToObject(pSUSICommDataItem, "subtype", "THRESHOLD_CHECK_ERROR");
		cJSON_AddStringToObject(pSUSICommDataItem, MODBUS_THR_CHECK_STATUS, "False");
	}
	if (pThrCheckRep->repInfo)
	{
		cJSON_AddStringToObject(pSUSICommDataItem, MODBUS_THR_CHECK_MSG, pThrCheckRep->repInfo);
	}
	else
	{
		cJSON_AddStringToObject(pSUSICommDataItem, MODBUS_THR_CHECK_MSG, "");
	}

	out = cJSON_PrintUnformatted(pSUSICommDataItem);
	outLen = strlen(out) + 1;
	*outputStr = (char *)(malloc(outLen));
	memset(*outputStr, 0, outLen);
	strcpy(*outputStr, out);
	cJSON_Delete(pSUSICommDataItem);
	printf("%s\n", out);
	free(out);
	return outLen;
}

bool InsertThresholdNode(modbus_threshold_list thrList, modbus_threshold_info * pinfo)
{
	bool bRet = false;

	if (pinfo == NULL || thrList == NULL) return bRet;

	modbus_threshold_node *head = thrList;

	modbus_threshold_node *newNode = (modbus_threshold_node *)malloc(sizeof(modbus_threshold_node));
	memset(newNode, 0, sizeof(modbus_threshold_node));
	newNode->info.name = (char *)calloc(1, strlen(pinfo->name) + 1);

	strcpy(newNode->info.name, pinfo->name);
	newNode->info.isEnable = pinfo->isEnable;
	newNode->info.maxThr = pinfo->maxThr;
	newNode->info.minThr = pinfo->minThr;
	newNode->info.thrType = pinfo->thrType;
	newNode->info.lastingTimeS = pinfo->lastingTimeS;
	newNode->info.intervalTimeS = pinfo->intervalTimeS;
	newNode->info.checkType = pinfo->checkType;
	newNode->info.checkRetValue = DEF_INVALID_VALUE;
	newNode->info.checkSrcValList.head = NULL;
	newNode->info.checkSrcValList.nodeCnt = 0;
	newNode->info.repThrTime = 0;
	//newNode->info.isNormal = true;
	//newNode->info.isValid = true;
	newNode->info.isNormal = pinfo->isNormal;
	newNode->info.isValid = pinfo->isValid;
	newNode->next = head->next;
	head->next = newNode;

	bRet = true;
	return bRet;
}

modbus_threshold_node* FindThresholdNodeWithName(modbus_threshold_list thrList, char *name)
{
	modbus_threshold_node * findNode = NULL, *head = NULL;
	if (thrList == NULL || name == NULL) return findNode;
	head = thrList;
	findNode = head->next;
	while (findNode)
	{
		if (!strcmp(findNode->info.name, name)) break;
		else
		{
			findNode = findNode->next;
		}
	}
	return findNode;
}

static void ModbusIsThrItemListNormal(modbus_threshold_list curThrItemList, bool * isNormal)
{
	if (NULL == isNormal || curThrItemList == NULL) return;
	{
		modbus_threshold_node * curThrItemNode = curThrItemList->next;
		while (curThrItemNode)
		{
			if (curThrItemNode->info.isEnable && !curThrItemNode->info.isNormal)
			{
				*isNormal = false;
				break;
			}
			curThrItemNode = curThrItemNode->next;
		}
	}
}
void UpdateThreshold(modbus_threshold_list curThrList, modbus_threshold_list newThrList)
{
	bool bRet = false;
	if (curThrList == NULL)


		if (NULL == newThrList || NULL == curThrList) return;
	{
		modbus_threshold_node * newinfoNode = NULL, *findinfoNode = NULL;
		modbus_threshold_node * curinfoNode = curThrList->next;
		while (curinfoNode) //first all thr node set invalid
		{
			curinfoNode->info.isValid = 0;
			curinfoNode = curinfoNode->next;
		}
		newinfoNode = newThrList->next;
		while (newinfoNode)  //merge old&new thr list
		{
			//findinfoNode = FindNMinfoNodeWithName(curThrList, newinfoNode->info.adapterName, newinfoNode->info.tagName);
			findinfoNode = FindThresholdNodeWithName(curThrList, newinfoNode->info.name);
			if (findinfoNode) //exist then update thr argc
			{
				findinfoNode->info.isValid = 1;
				findinfoNode->info.intervalTimeS = newinfoNode->info.intervalTimeS;
				findinfoNode->info.lastingTimeS = newinfoNode->info.lastingTimeS;
				findinfoNode->info.isEnable = newinfoNode->info.isEnable;
				findinfoNode->info.maxThr = newinfoNode->info.maxThr;
				findinfoNode->info.minThr = newinfoNode->info.minThr;
				findinfoNode->info.thrType = newinfoNode->info.thrType;
			}
			else  //not exist then insert to old list
			{
				InsertThresholdNode(curThrList, &newinfoNode->info);
			}
			newinfoNode = newinfoNode->next;
		}
		{
			unsigned int defRepMsg = 2 * 1024;
			char *repMsg = (char*)malloc(defRepMsg);
			modbus_threshold_node * preNode = curThrList, *normalRepNode = NULL, *delNode = NULL;
			memset(repMsg, 0, defRepMsg);
			curinfoNode = preNode->next;
			while (curinfoNode) //check need delete&normal report node
			{
				normalRepNode = NULL;
				delNode = NULL;
				if (curinfoNode->info.isValid == 0)
				{
					preNode->next = curinfoNode->next;
					delNode = curinfoNode;
					if (curinfoNode->info.isNormal == false)
					{
						normalRepNode = curinfoNode;
					}
				}
				else
				{
					preNode = curinfoNode;
				}
				if (normalRepNode == NULL && curinfoNode->info.isEnable == false && curinfoNode->info.isNormal == false)
				{
					normalRepNode = curinfoNode;
				}
				if (normalRepNode)
				{
					char *tmpMsg = NULL;
					int len = strlen(curinfoNode->info.name) + strlen(DEF_NOR_EVENT_STR) + 32;//if?ï¿½ï¿½?å¤§ï¿½?info.adapterNameï¼Œfree?ï¿½å‡º??
					tmpMsg = (char*)malloc(len);
					memset(tmpMsg, 0, len);
					sprintf(tmpMsg, "%s %s", curinfoNode->info.name, DEF_NOR_EVENT_STR);
					if (tmpMsg && strlen(tmpMsg))
					{
						if (defRepMsg < strlen(tmpMsg) + strlen(repMsg) + 1)
						{
							int newLen = strlen(tmpMsg) + strlen(repMsg) + 1024;
							repMsg = (char *)realloc(repMsg, newLen);
						}
						if (strlen(repMsg))
						{
							sprintf(repMsg, "%s;%s", repMsg, tmpMsg);
						}
						else
						{
							sprintf(repMsg, "%s", tmpMsg);
						}
					}
					if (tmpMsg)free(tmpMsg);
					tmpMsg = NULL;
					normalRepNode->info.isNormal = true;
				}
				if (delNode)
				{
					if (delNode->info.name != NULL)
					{
						free(delNode->info.name);
						delNode->info.name = NULL;
					}
					if (delNode->info.checkSrcValList.head)
					{
						check_value_node * frontValueNode = delNode->info.checkSrcValList.head;
						check_value_node * delValueNode = frontValueNode->next;
						while (delValueNode)
						{
							frontValueNode->next = delValueNode->next;
							free(delValueNode);
							delValueNode = frontValueNode->next;
						}
						free(delNode->info.checkSrcValList.head);
						delNode->info.checkSrcValList.head = NULL;
					}
					//if(delNode->info.name) free(delNode->info.name);
					//if(delNode->info.desc) free(delNode->info.desc);
					free(delNode);
					delNode = NULL;
				}
				curinfoNode = preNode->next;
			}
			if (strlen(repMsg))
			{
				char * repJsonStr = NULL;
				int jsonStrlen = 0;
				modbus_thr_rep_info thrRepInfo;
				int repInfoLen = strlen(repMsg) + 1;
				unsigned int repMsgLen = 0;

				repMsgLen = strlen(repMsg) + 1;
				thrRepInfo.isTotalNormal = true;
				//add
				ModbusIsThrItemListNormal(curThrList, &thrRepInfo.isTotalNormal);
				thrRepInfo.repInfo = (char*)malloc(repMsgLen);
				memset(thrRepInfo.repInfo, 0, repMsgLen);
				strcpy(thrRepInfo.repInfo, repMsg);
				jsonStrlen = Parser_PackThrCheckRep(&thrRepInfo, &repJsonStr);
				if (jsonStrlen > 0 && repJsonStr != NULL)
				{
					g_sendcbf(&g_PluginInfo, modbus_thr_check_rep, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
				}
				if (repJsonStr) free(repJsonStr);
				if (thrRepInfo.repInfo) free(thrRepInfo.repInfo);
			}
			if (repMsg)
				free(repMsg);
		}

		/*#if true
				{
					netmon_handler_context_t * pNetMonHandlerContext = &NetMonHandlerContext;
					net_info_list curNetInfoList = pNetMonHandlerContext->netInfoList;
					//app_os_mutex_lock(&NMThrInfoMutex);
					if(NMinfoList != NULL && NMinfoList->next != NULL)
					{
						UpdateNMinfoList();//must need!
					}
					//app_os_mutex_unlock(&NMThrInfoMutex);
				}
		#endif*/
	}
}

void SetThreshold()
{
	if (Threshold_Data == NULL || strlen(Threshold_Data) <= 1)
		return;

	g_ThresholdSetContex.isThreadRunning = true;

	if (pthread_create(&g_ThresholdSetContex.threadHandler, NULL, ThresholdSetThreadStart, NULL) != 0)
	{
		g_ThresholdSetContex.isThreadRunning = false;
		printf("> start ThresholdSet thread failed!\r\n");
	}
}

//-------------------------Check Part------------------------
static int FindDataNameWithThresholdName(char * name, int *type)
{
	int i = 0;
	int index = -1;
	char *str = (char *)calloc(1, strlen(name) + 1);
	char *delim = "/";
	char *pch = NULL;
	char *data_type = NULL;
	char *data_name = NULL;
	strcpy(str, name);
	printf("Splitting string \"%s\" into tokens:\n", str);
	pch = strtok(str, delim);
	while (pch != NULL)
	{
		if (i == 1)
		{
			data_type = (char *)calloc(1, strlen(pch) + 1);
			strcpy(data_type, pch);
		}
		else if (i == 2)
		{
			data_name = (char *)calloc(1, strlen(pch) + 1);
			strcpy(data_name, pch);
		}
		//printf ("%s\n",pch);
		pch = strtok(NULL, delim);
		i++;
	}

	for (int devCount = 0; devCount < numberOfWDev; devCount++)
	{
		if (strcmp(data_type, "Discrete Inputs") == 0)
		{
			for (int i = 0; i < WDev[devCount].numberOfDI; i++)
			{
				if (strcmp(WDev[devCount].DI[i].name, data_name) == 0)
				{
					index = i;
					*type = 0;
					break;
				}
			}

		}
		if (strcmp(data_type, "Coils") == 0)
		{
			for (int i = 0; i < WDev[devCount].numberOfDO; i++)
			{
				if (strcmp(WDev[devCount].DO[i].name, data_name) == 0)
				{
					index = i;
					*type = 1;
					break;
				}
			}
		}

		if (strcmp(data_type, "Input Registers") == 0)
		{
			for (int i = 0; i < WDev[devCount].numberOfAI; i++)
			{
				if (strcmp(WDev[devCount].AI[i].name, data_name) == 0)
				{
					index = i;
					*type = 2;
					break;
				}
			}

		}
		if (strcmp(data_type, "Holding Registers") == 0)
		{
			for (int i = 0; i < WDev[devCount].numberOfAO; i++)
			{
				if (strcmp(WDev[devCount].AO[i].name, data_name) == 0)
				{
					index = i;
					*type = 3;
					break;
				}
			}
		}
	}

	if (str != NULL)
		free(str);

	if (data_type != NULL)
		free(data_type);

	if (data_name != NULL)
		free(data_name);

	return index;
}

static bool ModbusCheckSrcVal(threshold_info * pThrItemInfo, float * pCheckValue)
{
	bool bRet = false;
	if (pThrItemInfo == NULL || pCheckValue == NULL) return bRet;
	{
		long long nowTime = time(NULL);
		pThrItemInfo->checkRetValue = DEF_INVALID_VALUE;
		if (pThrItemInfo->checkSrcValList.head == NULL)
		{
			pThrItemInfo->checkSrcValList.head = (check_value_node *)malloc(sizeof(check_value_node));
			pThrItemInfo->checkSrcValList.nodeCnt = 0;
			pThrItemInfo->checkSrcValList.head->checkValTime = DEF_INVALID_TIME;
			pThrItemInfo->checkSrcValList.head->ckV = DEF_INVALID_VALUE;
			pThrItemInfo->checkSrcValList.head->next = NULL;
		}

		if (pThrItemInfo->checkSrcValList.nodeCnt > 0)
		{
			long long minCkvTime = 0;
			check_value_node * curNode = pThrItemInfo->checkSrcValList.head->next;
			minCkvTime = curNode->checkValTime;
			while (curNode)
			{
				if (curNode->checkValTime < minCkvTime)  minCkvTime = curNode->checkValTime;
				curNode = curNode->next;
			}

			if (nowTime - minCkvTime >= pThrItemInfo->lastingTimeS)
			{
				switch (pThrItemInfo->checkType)
				{
				case ck_type_avg:
				{
					check_value_node * curNode = pThrItemInfo->checkSrcValList.head->next;
					float avgTmpF = 0;
					int avgTmpI = 0;
					while (curNode)
					{
						if (curNode->ckV != DEF_INVALID_VALUE)
						{
							avgTmpF += curNode->ckV;
						}
						curNode = curNode->next;
					}
					if (pThrItemInfo->checkSrcValList.nodeCnt > 0)
					{
						avgTmpF = avgTmpF / pThrItemInfo->checkSrcValList.nodeCnt;
						pThrItemInfo->checkRetValue = avgTmpF;
						bRet = true;
					}
					break;
				}
				case ck_type_max:
				{
					check_value_node * curNode = pThrItemInfo->checkSrcValList.head->next;
					float maxTmpF = -FLT_MAX;
					int maxTmpI = -FLT_MAX;
					while (curNode)
					{
						if (curNode->ckV > maxTmpF)
							maxTmpF = curNode->ckV;

						curNode = curNode->next;
					}

					if (maxTmpF > -FLT_MAX)
					{
						pThrItemInfo->checkRetValue = maxTmpF;
						bRet = true;
					}
					break;
				}
				case ck_type_min:
				{
					check_value_node * curNode = pThrItemInfo->checkSrcValList.head->next;
					float minTmpF = FLT_MAX;
					int minTmpI = FLT_MAX;
					while (curNode)
					{
						if (curNode->ckV < minTmpF)
							minTmpF = curNode->ckV;

						curNode = curNode->next;
					}

					if (minTmpF < FLT_MAX)
					{
						pThrItemInfo->checkRetValue = minTmpF;
						bRet = true;
					}
					break;
				}
				default: break;
				}

				{
					check_value_node * frontNode = pThrItemInfo->checkSrcValList.head;
					check_value_node * curNode = frontNode->next;
					check_value_node * delNode = NULL;
					while (curNode)
					{
						if (nowTime - curNode->checkValTime >= pThrItemInfo->lastingTimeS)
						{
							delNode = curNode;
							frontNode->next = curNode->next;
							curNode = frontNode->next;
							free(delNode);
							pThrItemInfo->checkSrcValList.nodeCnt--;
							delNode = NULL;
						}
						else
						{
							frontNode = curNode;
							curNode = frontNode->next;
						}
					}
				}
			}
		} //end if(pThrItemInfo->checkSrcValList.nodeCnt > 0)
		{
			check_value_node * head = pThrItemInfo->checkSrcValList.head;
			check_value_node * newNode = (check_value_node *)malloc(sizeof(check_value_node));
			newNode->checkValTime = nowTime;
			newNode->ckV = (*pCheckValue);
			newNode->next = head->next;
			head->next = newNode;
			pThrItemInfo->checkSrcValList.nodeCnt++;
		}
	}
	return bRet;
}

static bool ModbusCheckThrItem(threshold_info * pThrItemInfo, float * checkVal, char * checkRetMsg)
{
	bool bRet = false;
	bool isTrigger = false;
	bool triggerMax = false;
	bool triggerMin = false;
	char tmpRetMsg[1024] = { 0 };
	char checkTypeStr[32] = { 0 };
	if (pThrItemInfo == NULL || checkVal == NULL || checkRetMsg == NULL) return bRet;
	{
		switch (pThrItemInfo->checkType)
		{
		case ck_type_avg:
		{
			sprintf(checkTypeStr, "average");
			break;
		}
		case ck_type_max:
		{
			sprintf(checkTypeStr, "max");
			break;
		}
		case ck_type_min:
		{
			sprintf(checkTypeStr, "min");
			break;
		}
		default: break;
		}
	}

	if (ModbusCheckSrcVal(pThrItemInfo, checkVal) && pThrItemInfo->checkRetValue != DEF_INVALID_VALUE)
	{
		if (pThrItemInfo->thrType & DEF_THR_MAX_TYPE)
		{
			if (pThrItemInfo->maxThr != DEF_INVALID_VALUE && (pThrItemInfo->checkRetValue > pThrItemInfo->maxThr))
			{
				char pathStr[256] = { 0 };
				char bnStr[256] = { 0 };
				sprintf(pathStr, "%s", pThrItemInfo->name);
				sprintf(tmpRetMsg, "%s #tk#%s#tk# (%f)> #tk#maxThreshold#tk# (%f)", pathStr, checkTypeStr, pThrItemInfo->checkRetValue, pThrItemInfo->maxThr);
				//sprintf(bnStr, "%s%d-%s", NETMON_GROUP_NETINDEX, pThrItemInfo->index, pThrItemInfo->adapterName);
				//sprintf(pathStr, "%s/%s/%s/%s", DEF_HANDLER_NAME, NETMON_INFO_LIST, bnStr, pThrItemInfo->tagName);
				//sprintf(tmpRetMsg, "%s #tk#%s#tk# (%f)> #tk#maxThreshold#tk# (%f)", pathStr, checkTypeStr, pThrItemInfo->checkRetValue, pThrItemInfo->maxThr);
				//sprintf(tmpRetMsg, "%s (#tk#%s#tk#:%f)> #tk#maxThreshold#tk# (%f)", pThrItemInfo->n, checkTypeStr, pThrItemInfo->checkRetValue, pThrItemInfo->maxThr);
				triggerMax = true;
			}
		}
		if (pThrItemInfo->thrType & DEF_THR_MIN_TYPE)
		{
			if (pThrItemInfo->minThr != DEF_INVALID_VALUE && (pThrItemInfo->checkRetValue < pThrItemInfo->minThr))
			{
				char pathStr[256] = { 0 };
				char bnStr[256] = { 0 };
				sprintf(pathStr, "%s", pThrItemInfo->name);
				sprintf(tmpRetMsg, "%s #tk#%s#tk# (%f)< #tk#minThreshold#tk# (%f)", pathStr, checkTypeStr, pThrItemInfo->checkRetValue, pThrItemInfo->minThr);
				//sprintf(bnStr, "%s%d-%s", NETMON_GROUP_NETINDEX, pThrItemInfo->index, pThrItemInfo->adapterName);
				//sprintf(pathStr, "%s/%s/%s/%s", DEF_HANDLER_NAME, NETMON_INFO_LIST, bnStr, pThrItemInfo->tagName);
				//sprintf(tmpRetMsg, "%s #tk#%s#tk# (%f)< #tk#minThreshold#tk# (%f)", pathStr, checkTypeStr, pThrItemInfo->checkRetValue, pThrItemInfo->minThr);
				//sprintf(tmpRetMsg, "%s (#tk#%s#tk#:%.0f)< #tk#minThreshold#tk# (%f)", pThrItemInfo->n, checkTypeStr, pThrItemInfo->checkRetValue, pThrItemInfo->minThr);
				triggerMin = true;
			}
		}
	}

	switch (pThrItemInfo->thrType)
	{
	case DEF_THR_MAX_TYPE:
	{
		isTrigger = triggerMax;
		break;
	}
	case DEF_THR_MIN_TYPE:
	{
		isTrigger = triggerMin;
		break;
	}
	case DEF_THR_MAXMIN_TYPE:
	{
		isTrigger = triggerMin || triggerMax;
		break;
	}
	}

	if (isTrigger)
	{
		long long nowTime = time(NULL);
		if (pThrItemInfo->intervalTimeS == DEF_INVALID_TIME || pThrItemInfo->intervalTimeS == 0 || nowTime - pThrItemInfo->repThrTime >= pThrItemInfo->intervalTimeS)
		{
			pThrItemInfo->repThrTime = nowTime;
			pThrItemInfo->isNormal = false;
			bRet = true;
		}
	}
	else
	{
		if (!pThrItemInfo->isNormal && pThrItemInfo->checkRetValue != DEF_INVALID_VALUE)
		{
			memset(tmpRetMsg, 0, sizeof(tmpRetMsg));
			sprintf(tmpRetMsg, "%s %s", pThrItemInfo->name, DEF_NOR_EVENT_STR);
			//sprintf(tmpRetMsg, "%s (#tk#%s#tk#:%f) %s", pThrItemInfo->n, checkTypeStr, pThrItemInfo->checkRetValue, DEF_NOR_EVENT_STR);
			pThrItemInfo->isNormal = true;
			bRet = true;
		}
	}

	if (!bRet) sprintf(checkRetMsg, "");
	else sprintf(checkRetMsg, "%s", tmpRetMsg);

	return bRet;
}

static bool ModbusCheckThr(modbus_threshold_list curThrItemList, char ** checkRetMsg, unsigned int bufLen)
{
	bool bRet = false;
	if (curThrItemList == NULL || NULL == (char*)(*checkRetMsg))
		return bRet;

	modbus_threshold_node * curThrItemNode = NULL;
	char tmpMsg[1024 * 2] = { 0 };
	float curCheckValue;
	curThrItemNode = curThrItemList->next;

	while (curThrItemNode)
	{
		if (curThrItemNode->info.isEnable && strlen(curThrItemNode->info.name) > 0)
		{
			int index = -1;
			int type = -1;

			index = FindDataNameWithThresholdName(curThrItemNode->info.name, &type);
			if (index == -1)
			{
				if (bufLen < strlen(*checkRetMsg) + strlen(DEF_NOT_SUPT_EVENT_STR) + 16)
				{
					int newLen = strlen(*checkRetMsg) + strlen(DEF_NOT_SUPT_EVENT_STR) + 2 * 1024;
					*checkRetMsg = (char*)realloc(*checkRetMsg, newLen);
				}
				if (strlen(*checkRetMsg))sprintf(*checkRetMsg, "%s;%s not support", *checkRetMsg, curThrItemNode->info.name);
				else sprintf(*checkRetMsg, "%s not support", curThrItemNode->info.name);
				curThrItemNode->info.isEnable = false;
				curThrItemNode = curThrItemNode->next;
				usleep(10 * 1000);
				continue;
			}

			for (int devCount = 0; devCount < numberOfWDev; devCount++)
			{
				memset(tmpMsg, 0, sizeof(tmpMsg));
				if (type == 0)				//Discrete Inputs
				{
					curCheckValue = *(WDev[devCount].DI_Bits + index);
				}
				else if (type == 1)		//Coils
				{
					curCheckValue = *(WDev[devCount].DO_Bits + index);
				}
				else if (type == 2)		//Input Registers
				{
					if (strcmp(WDev[devCount].AI[index].conversion, "") == 0)
					{
						if (WDev[devCount].AI[index].sw_mode == 1 || WDev[devCount].AI[index].sw_mode == 2 || WDev[devCount].AI[index].sw_mode == 3 || WDev[devCount].AI[index].sw_mode == 4)
						{
							curCheckValue = (float)WDev[devCount].AI[index].fv*WDev[devCount].AI[index].precision;
						}
						else if (WDev[devCount].AI[index].sw_mode == 5 || WDev[devCount].AI[index].sw_mode == 6)
						{
							curCheckValue = (float)WDev[devCount].AI[index].uiv*WDev[devCount].AI[index].precision;
						}
						else if (WDev[devCount].AI[index].sw_mode == 7 || WDev[devCount].AI[index].sw_mode == 8)
						{
							curCheckValue = (float)WDev[devCount].AI[index].iv*WDev[devCount].AI[index].precision;
						}
						else
						{
							curCheckValue = (float)WDev[devCount].AI[index].Regs*WDev[devCount].AI[index].precision;
						}
					}
					else
					{
						if (WDev[devCount].AI[index].sw_mode == 1 || WDev[devCount].AI[index].sw_mode == 2 || WDev[devCount].AI[index].sw_mode == 3 || WDev[devCount].AI[index].sw_mode == 4)
						{
							float valConv = LuaConversion(WDev[devCount].AI[index].fv, WDev[devCount].AI[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AI[index].precision;
						}
						else if (WDev[devCount].AI[index].sw_mode == 5 || WDev[devCount].AI[index].sw_mode == 6)
						{
							uint32_t valConv = LuaConversion(WDev[devCount].AI[index].uiv, WDev[devCount].AI[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AI[index].precision;
						}
						else if (WDev[devCount].AI[index].sw_mode == 7 || WDev[devCount].AI[index].sw_mode == 8)
						{
							int valConv = LuaConversion(WDev[devCount].AI[index].iv, WDev[devCount].AI[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AI[index].precision;
						}
						else
						{
							double valConv = LuaConversion(WDev[devCount].AI[index].Regs, WDev[devCount].AI[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AI[index].precision;
						}
					}
				}
				else if (type == 3)		//Holding Registers
				{
					if (strcmp(WDev[devCount].AO[index].conversion, "") == 0)
					{
						if (WDev[devCount].AO[index].sw_mode == 1 || WDev[devCount].AO[index].sw_mode == 2 || WDev[devCount].AO[index].sw_mode == 3 || WDev[devCount].AO[index].sw_mode == 4)
						{
							curCheckValue = (float)WDev[devCount].AO[index].fv*WDev[devCount].AO[index].precision;
						}
						else if (WDev[devCount].AO[index].sw_mode == 5 || WDev[devCount].AO[index].sw_mode == 6)
						{
							curCheckValue = (float)WDev[devCount].AO[index].uiv*WDev[devCount].AO[index].precision;
						}
						else if (WDev[devCount].AO[index].sw_mode == 7 || WDev[devCount].AO[index].sw_mode == 8)
						{
							curCheckValue = (float)WDev[devCount].AO[index].iv*WDev[devCount].AO[index].precision;
						}
						else
						{
							curCheckValue = (float)WDev[devCount].AO[index].Regs*WDev[devCount].AO[index].precision;
						}
					}
					else
					{
						if (WDev[devCount].AO[index].sw_mode == 1 || WDev[devCount].AO[index].sw_mode == 2 || WDev[devCount].AO[index].sw_mode == 3 || WDev[devCount].AO[index].sw_mode == 4)
						{
							float valConv = LuaConversion(WDev[devCount].AO[index].fv, WDev[devCount].AO[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AO[index].precision;
						}
						else if (WDev[devCount].AO[index].sw_mode == 5 || WDev[devCount].AO[index].sw_mode == 6)
						{
							uint32_t valConv = LuaConversion(WDev[devCount].AO[index].uiv, WDev[devCount].AO[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AO[index].precision;
						}
						else if (WDev[devCount].AO[index].sw_mode == 7 || WDev[devCount].AO[index].sw_mode == 8)
						{
							int valConv = LuaConversion(WDev[devCount].AO[index].iv, WDev[devCount].AO[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AO[index].precision;
						}
						else
						{
							double valConv = LuaConversion(WDev[devCount].AO[index].Regs, WDev[devCount].AO[index].conversion);
							curCheckValue = (float)valConv*WDev[devCount].AO[index].precision;
						}
					}
				}
			}

			ModbusCheckThrItem(&curThrItemNode->info, &curCheckValue, tmpMsg);//?ï¿½ç?out

			if (strlen(tmpMsg))
			{
				if (bufLen < strlen(*checkRetMsg) + strlen(tmpMsg) + 16)
				{
					int newLen = strlen(*checkRetMsg) + strlen(tmpMsg) + 2 * 1024;
					*checkRetMsg = (char*)realloc(*checkRetMsg, newLen);
				}
				if (strlen(*checkRetMsg))sprintf(*checkRetMsg, "%s;%s", *checkRetMsg, tmpMsg);
				else sprintf(*checkRetMsg, "%s", tmpMsg);
			}

			curThrItemNode = curThrItemNode->next;
			usleep(10 * 1000);
		}
		else if (!curThrItemNode->info.isEnable)
		{
			curThrItemNode = curThrItemNode->next;
			usleep(10 * 1000);
		}
	}
	return bRet;
}

static bool ModbusIsThrNormal(modbus_threshold_list thrList, bool * isNormal)
{
	bool bRet = false;
	if (isNormal == NULL || thrList == NULL) return bRet;
	{
		modbus_threshold_node * curThrItemNode = NULL;
		curThrItemNode = thrList->next;
		while (curThrItemNode)
		{
			if (curThrItemNode->info.isEnable && !curThrItemNode->info.isNormal && curThrItemNode->info.isValid)
			{
				*isNormal = false;
				break;
			}
			curThrItemNode = curThrItemNode->next;
		}
	}
	bRet = true;
	return bRet;
}
//--------------------------------------------------------------------------------------------------------------
//------------------------------------------------Threads-------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//Modbus Retrieve Thread
static void* RetrieveThreadStart(void *args)
{
	handler_context_t *pHandlerContex = (handler_context_t *)args;

#pragma region Modbus_Retrieve
	while (g_PluginInfo.agentInfo->status == 0)
	{
		if (!pHandlerContex->isThreadRunning && bAllDataAlloc)
			return 0;
		sleep(1);
	}

	while (pHandlerContex->isThreadRunning)
	{
		if (bAllDataAlloc)
			if (!bIsSimtor && ctx != NULL)
			{
				g_bRetrieve = Modbus_Rev();
				AssembleData();
			}
		//usleep(500*1000);	//To fix yocto
		usleep(Modbus_Interval * 1000 * 1000);
	}
#pragma endregion Modbus_Retrieve

	return 0;
}

//Auto Report Thread
static void* AutoReportThreadStart(void *args)
{
	handler_context_t *pHandlerContex = (handler_context_t *)args;
	int i = 0;
	bool bReport_Upload;
	time_t lock_time;
	time_t current_time;

	while (g_PluginInfo.agentInfo->status == 0 && bAllDataAlloc)
	{
		if (!pHandlerContex->isThreadRunning)
			return 0;
		sleep(1);
	}

	while (g_AutoReportContex.isThreadRunning)
	{
#pragma region g_bAutoReport
		if (bAllDataAlloc)
		{
			if (g_bAutoReport)
			{
				bReport_Upload = true;
				printf("\nAutoReport...........\n");

				if (bConnectionFlag == true)
				{
					bBlanking = false;
				}

				if (bBlanking != true)
				{
					bBlanking = !bConnectionFlag;
					UploadSensorsDataEx(Report_Data_paths, Report_array_size, Report_Reply_All, bReport_Upload, args);
				}

				//sleep(Report_interval);
				lock_time = time(NULL);
				while (true)
				{
					current_time = time(NULL);
					if (current_time - lock_time >= Report_interval)
						break;
					usleep(100 * 1000);
				}
			}
			else
				sleep(1);
		}
		else
			sleep(1);
#pragma endregion g_bAutoReport
	}
	return 0;
}

//Auto Upload Thread
static void* AutoUploadThreadStart(void *args)
{
	bool bReport_Upload;
	char tmpRepFilter[4096] = { 0 };
	cJSON *root = NULL;
	unsigned int diff = 0;

	handler_context_t *pHandlerContex = (handler_context_t *)args;
	int i = 0;

	while (g_PluginInfo.agentInfo->status == 0 && bAllDataAlloc)
	{
		if (!pHandlerContex->isThreadRunning)
			return 0;
		sleep(1);
	}

	while (g_AutoUploadContex.isThreadRunning)
	{
		if (bAllDataAlloc)
		{
#pragma region g_bAutoUpload
			if (g_bAutoUpload)
			{
				bool Reply_All = false;
				char **Data_paths = NULL;
				int array_size = 0;

				cJSON *item, *it, *js_name, *js_list;

#pragma region Reply_All 
				if (!Reply_All)
				{
#pragma region root 
					root = cJSON_Parse(AutoUploadParams.repFilter);
					if (!root)
					{
						printf("get root faild !\n");
					}
					else
					{
						js_list = cJSON_GetObjectItem(root, "e");
#pragma region js_list
						if (!js_list)
						{
							printf("no list!\n");
						}
						else
						{
							array_size = cJSON_GetArraySize(js_list);
							Data_paths = (char **)calloc(array_size, sizeof(char *));

							char *p = NULL;

							for (i = 0; i < array_size; i++)
							{

								item = cJSON_GetArrayItem(js_list, i);
								if (Data_paths)
									Data_paths[i] = (char *)calloc(200, sizeof(char));   //set Data_path size as 200*char
								if (!item)
								{
									printf("no item!\n");
								}
								else
								{
									p = cJSON_PrintUnformatted(item);
									it = cJSON_Parse(p);

									if (!it)
										continue;
									else
									{
										js_name = cJSON_GetObjectItem(it, "n");
										if (Data_paths)
											if (Data_paths[i])
												strcpy(Data_paths[i], js_name->valuestring);
									}

									if (p)
										free(p);

									if (it)
										cJSON_Delete(it);

									if (strcmp(Data_paths[i], strPluginName) == 0)
									{
										Reply_All = true;
										printf("Reply_All=true;\n");
										break;
									}
								}
							}
						}
#pragma endregion js_list
					}

					if (root)
						cJSON_Delete(root);
#pragma endregion root
				}
#pragma endregion Reply_All

				Continue_Upload = time(NULL);

				diff = (unsigned int)difftime(Continue_Upload, Start_Upload);
				printf("timeout: %d\n", AutoUploadParams.continueTimeMs / 1000);
				printf("diff: %d\n", diff);

				if (diff < AutoUploadParams.continueTimeMs / 1000)
				{
					bReport_Upload = false;

					if (bConnectionFlag == true)
					{
						bBlanking = false;
					}

					if (bBlanking != true)
					{
						bBlanking = !bConnectionFlag;
						UploadSensorsDataEx(Data_paths, array_size, Reply_All, bReport_Upload, args);
					}
					sleep(AutoUploadParams.intervalTimeMs / 1000);
				}
				else
				{
					g_bAutoUpload = false;
				}

				for (i = 0; i < array_size; i++)
					if (Data_paths)
						if (Data_paths[i])
							free(Data_paths[i]);

				if (Data_paths)
					free(Data_paths);
			}
			else
				sleep(1);
#pragma endregion g_bAutoUpload
		}
		else
			sleep(1);
	}

	return 0;
}

//Threshold Checker Thread
static void* ThresholdCheckThreadStart(void *args)
{
	handler_context_t *pHandlerContex = (handler_context_t *)args;
	int i = 0;

	char *repMsg = NULL;
	unsigned int bufLen = 4 * 1024;
	bool bRet = false;
	repMsg = (char *)malloc(bufLen);
	memset(repMsg, 0, bufLen);

	while (g_ThresholdCheckContex.isThreadRunning)
	{
		pthread_mutex_lock(&pModbusThresholdMux);
		if (MODBUSthresholdList != NULL && MODBUSthresholdList->next != NULL)
		{
			memset(repMsg, 0, bufLen);
			ModbusCheckThr(MODBUSthresholdList, &repMsg, bufLen);
		}
		pthread_mutex_unlock(&pModbusThresholdMux);

		if (strlen(repMsg))
		{
			bool bRet = false;
			bool isNormal = true;
			//app_os_mutex_lock(&NMThrInfoMutex);
			bRet = ModbusIsThrNormal(MODBUSthresholdList, &isNormal);
			//app_os_mutex_unlock(&NMThrInfoMutex);
			if (bRet)
			{
				char * repJsonStr = NULL;
				int jsonStrlen = 0;
				modbus_thr_rep_info thrRepInfo;
				unsigned int repMsgLen = 0;
				HANDLER_NOTIFY_SEVERITY severity;
				thrRepInfo.isTotalNormal = isNormal;

				repMsgLen = strlen(repMsg) + 1;
				thrRepInfo.repInfo = (char*)malloc(repMsgLen);
				memset(thrRepInfo.repInfo, 0, repMsgLen);
				strcpy(thrRepInfo.repInfo, repMsg);
				jsonStrlen = Parser_PackThrCheckRep(&thrRepInfo, &repJsonStr);
				if (jsonStrlen > 0 && repJsonStr != NULL)
				{
					//g_sendcbf(&g_PluginInfo, modbus_thr_check_rep, repJsonStr, strlen(repJsonStr)+1, NULL, NULL);
					if (thrRepInfo.isTotalNormal)
						severity = Severity_Informational;
					else
						severity = Severity_Error;

					g_sendeventcbf(&g_PluginInfo, severity, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
				}
				if (repJsonStr)free(repJsonStr);
				if (thrRepInfo.repInfo) free(thrRepInfo.repInfo);
			}
		}

		{//app_os_sleep(5000);
			int i = 0;
			for (i = 0; g_ThresholdCheckContex.isThreadRunning && i < 10; i++)
			{
				usleep(100 * 1000);
			}
		}
	}

	if (repMsg)
		free(repMsg);

	return 0;
}

//Threshold Setter Thread
static void* ThresholdSetThreadStart(void *args)
{
	int i = 0;
	char repMsg[1024] = "";

	pthread_mutex_lock(&pModbusThresholdMux);
	modbus_threshold_list Threshold_List = CreateThresholdList();
	bool bRet = ParseThreshold(Threshold_List);

	while (g_ThresholdSetContex.isThreadRunning)
	{
		if (!bRet)
		{
			sprintf(repMsg, "%s", "Threshold apply failed!");
		}
		else
		{
			UpdateThreshold(MODBUSthresholdList, Threshold_List);
			sprintf(repMsg, "Threshold apply OK!");
		}

		if (strlen(repMsg))
		{
			char * repJsonStr = NULL;
			int jsonStrlen = Ack_SetThreshold(repMsg, &repJsonStr);
			if (jsonStrlen > 0 && repJsonStr != NULL)
			{
				g_sendcbf(&g_PluginInfo, modbus_set_thr_rep, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
			}

			if (repJsonStr)
				free(repJsonStr);
		}

		g_ThresholdSetContex.isThreadRunning = false;
	}

	if (Threshold_List)
		DestroyThresholdList(Threshold_List);

	pthread_mutex_unlock(&pModbusThresholdMux);

	return 0;
}

//Threshold Delete Thread
static void* ThresholdDeleteThreadStart(void *args)
{
	modbus_threshold_list curThrItemList = NULL;
	char *tmpMsg;
	unsigned int bufLen = 1024 * 2;
	tmpMsg = (char*)malloc(bufLen);
	memset(tmpMsg, 0, bufLen);

	while (g_ThresholdDeleteContex.isThreadRunning)
	{
		curThrItemList = MODBUSthresholdList;
		if (curThrItemList)
		{
			ModbusWhenDelThrCheckNormal(curThrItemList, &tmpMsg, bufLen);
			DeleteAllModbusThrItemNode(curThrItemList);
			//NetMonWhenDelThrCheckNormal(curThrItemList, &tmpMsg, bufLen);
			//DeleteAllNMThrItemNode(curThrItemList);
		}

		if (strlen(tmpMsg))
		{
			char * repJsonStr = NULL;
			int jsonStrlen = 0;
			modbus_thr_rep_info thrRepInfo;
			unsigned int tmpMsgLen = 0;

			tmpMsgLen = strlen(tmpMsg) + 1;
			thrRepInfo.isTotalNormal = true;
			thrRepInfo.repInfo = (char*)malloc(tmpMsgLen);
			memset(thrRepInfo.repInfo, 0, tmpMsgLen);
			strcpy(thrRepInfo.repInfo, tmpMsg);
			jsonStrlen = Parser_PackThrCheckRep(&thrRepInfo, &repJsonStr);
			if (jsonStrlen > 0 && repJsonStr != NULL)
			{
				HANDLER_NOTIFY_SEVERITY severity = Severity_Informational;
				g_sendeventcbf(&g_PluginInfo, severity, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
			}
			if (repJsonStr)
				free(repJsonStr);

			if (thrRepInfo.repInfo)
				free(thrRepInfo.repInfo);
		}

		if (tmpMsg)
			free(tmpMsg);

		{
			char * repJsonStr = NULL;
			int jsonStrlen = 0;
			char delRepMsg[256] = { 0 };
			snprintf(delRepMsg, sizeof(delRepMsg), "%s", "Delete all threshold successfully!");
			jsonStrlen = Parser_PackDelAllThrRep(delRepMsg, &repJsonStr);
			if (repJsonStr != NULL)
			{
				g_sendcbf(&g_PluginInfo, modbus_del_thr_rep, repJsonStr, strlen(repJsonStr) + 1, NULL, NULL);
			}

			if (repJsonStr)
				free(repJsonStr);
		}

		g_ThresholdDeleteContex.isThreadRunning = false;
	}
	return 0;
}


//--------------------------------------------------------------------------------------------------------------
//--------------------------------------Handler Functions-------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
/* **************************************************************************************
 *  Function Name: Handler_Initialize
 *  Description: Init any objects or variables of this handler
 *  Input :  PLUGIN_INFO *pluginfo
 *  Output: None
 *  Return:  0  : Success Init Handler
 *              -1 : Fail Init Handler
 * ***************************************************************************************/
int HANDLER_API Handler_Initialize(HANDLER_INFO *pluginfo)
{
	char version[INFOSTRLEN];
	char description[INFOSTRLEN];

	if (pluginfo == NULL)
		return handler_fail;

	g_loghandle = pluginfo->loghandle;

	// 1. Topic of this handler
	strPluginName = (char *)calloc(strlen(pluginfo->Name) + 1, sizeof(char));
	strcpy(strPluginName, pluginfo->Name);

	sprintf(version, "%d.%d.%d", VER_MAJOR, VER_MINOR, VER_BUILD);
	strPluginVersion = (char *)calloc(strlen(version) + 1, sizeof(char));
	strcpy(strPluginVersion, version);

	sprintf(description, "%s", SW_DESC);
	strPluginDescription = (char *)calloc(strlen(description) + 1, sizeof(char));
	strcpy(strPluginDescription, description);

	pluginfo->RequestID = iRequestID;
	pluginfo->ActionID = iActionID;
	printf(" >Name: %s\r\n", strPluginName);
	printf(" >Version: %s\r\n", strPluginVersion);
	printf(" >Description: %s\r\n", strPluginDescription);

	// 2. Copy agent info 
	memcpy(&g_PluginInfo, pluginfo, sizeof(HANDLER_INFO));
	g_PluginInfo.agentInfo = pluginfo->agentInfo;

	// 3. Callback function -> Send JSON Data by this callback function
	g_sendcbf = g_PluginInfo.sendcbf = pluginfo->sendcbf;
	g_sendcustcbf = g_PluginInfo.sendcustcbf = pluginfo->sendcustcbf;
	g_subscribecustcbf = g_PluginInfo.subscribecustcbf = pluginfo->subscribecustcbf;
	g_sendreportcbf = g_PluginInfo.sendreportcbf = pluginfo->sendreportcbf;
	g_sendcapabilitycbf = g_PluginInfo.sendcapabilitycbf = pluginfo->sendcapabilitycbf;
	g_sendeventcbf = g_PluginInfo.sendeventcbf = pluginfo->sendeventcbf;

	g_RetrieveContex.threadHandler = NULL;
	g_RetrieveContex.isThreadRunning = false;
	g_AutoReportContex.threadHandler = NULL;
	g_AutoReportContex.isThreadRunning = false;
	g_AutoUploadContex.threadHandler = NULL;
	g_AutoUploadContex.isThreadRunning = false;

	g_status = handler_status_init;

	return handler_success;
}

/* **************************************************************************************
 *  Function Name: Handler_Uninitialize
 *  Description: Release the objects or variables used in this handler
 *  Input :  None
 *  Output: None
 *  Return:  void
 * ***************************************************************************************/
void Handler_Uninitialize()
{
	void *status;

	/*
	if (g_RetrieveContex.isThreadRunning == true)
	{
		g_RetrieveContex.isThreadRunning = false;
		pthread_join((pthread_t)g_RetrieveContex.threadHandler, &status);
		g_RetrieveContex.threadHandler = NULL;
	}

	if (g_AutoReportContex.isThreadRunning == true)
	{
		int i = 0;
		g_AutoReportContex.isThreadRunning = false;
		pthread_join((pthread_t)g_AutoReportContex.threadHandler, &status);
		g_AutoReportContex.threadHandler = NULL;

		for (i = 0; i < Report_array_size; i++)
		{
			if (Report_Data_paths != NULL)
				if (Report_Data_paths[i] != NULL)
					free(Report_Data_paths[i]);
		}
		if (Report_Data_paths != NULL)
		{
			free(Report_Data_paths);
			Report_Data_paths = NULL;
		}
	}

	if (g_AutoUploadContex.isThreadRunning == true)
	{

		g_AutoUploadContex.isThreadRunning = false;
		pthread_join((pthread_t)g_AutoUploadContex.threadHandler, &status);
		g_AutoUploadContex.threadHandler = NULL;
	}


	if (g_ThresholdCheckContex.isThreadRunning == true)
	{
		g_ThresholdCheckContex.isThreadRunning = false;
		pthread_join((pthread_t)g_ThresholdCheckContex.threadHandler, &status);
		g_ThresholdCheckContex.threadHandler = NULL;
	}

	if (g_ThresholdSetContex.isThreadRunning == true)
	{
		g_ThresholdSetContex.isThreadRunning = false;
		pthread_join((pthread_t)g_ThresholdSetContex.threadHandler, &status);
		g_ThresholdSetContex.threadHandler = NULL;
	}

	if (g_ThresholdDeleteContex.isThreadRunning == true)
	{
		g_ThresholdDeleteContex.isThreadRunning = false;
		pthread_join((pthread_t)g_ThresholdDeleteContex.threadHandler, &status);
		g_ThresholdDeleteContex.threadHandler = NULL;
	}
	*/

	g_sendcbf = NULL;
	g_sendcustcbf = NULL;
	g_sendreportcbf = NULL;
	g_sendcapabilitycbf = NULL;
	g_subscribecustcbf = NULL;

	if (strPluginName)
		free(strPluginName);

	if (strPluginVersion)
		free(strPluginVersion);

	if (strPluginDescription)
		free(strPluginDescription);

	if (MRLog_path)
		free(MRLog_path);

	if (MSLog_path)
		free(MSLog_path);

	if (pMRLog)
		fclose(pMRLog);

	if (pMSLog)
		fclose(pMSLog);

	if (g_Capability)
	{
		IoT_ReleaseAll(g_Capability);
		g_Capability = NULL;
	}

	if (Threshold_Data != NULL)
		free(Threshold_Data);

	pthread_mutex_lock(&pModbusThresholdMux);
	if (MODBUSthresholdList)
		DestroyThresholdList(MODBUSthresholdList);
	MODBUSthresholdList = NULL;
	pthread_mutex_unlock(&pModbusThresholdMux);

}

/* **************************************************************************************
 *  Function Name: Handler_Get_Status
 *  Description: Get Handler Threads Status. CAgent will restart current Handler or restart CAgent self if busy.
 *  Input :  None
 *  Output: char * : pOutStatus       // cagent handler status
 *  Return:  handler_success  : Success Init Handler
 *			 handler_fail : Fail Init Handler
 * **************************************************************************************/
int HANDLER_API Handler_Get_Status(HANDLER_THREAD_STATUS * pOutStatus)
{
	int iRet = handler_fail;

	if (!pOutStatus) return iRet;

	switch (g_status)
	{
	default:
	case handler_status_no_init:
	case handler_status_init:
	case handler_status_stop:
		*pOutStatus = g_status;
		break;
	case handler_status_start:
	case handler_status_busy:
	{
		/*time_t tv;
		time(&tv);
		if(difftime(tv, g_monitortime)>5)
			g_status = handler_status_busy;
		else
			g_status = handler_status_start;*/
		*pOutStatus = g_status;
	}
	break;
	}

	iRet = handler_success;
	return iRet;
}


/* **************************************************************************************
 *  Function Name: Handler_OnStatusChange
 *  Description: Agent can notify handler the status is changed.
 *  Input :  PLUGIN_INFO *pluginfo
 *  Output: None
 *  Return:  None
 * ***************************************************************************************/
void HANDLER_API Handler_OnStatusChange(HANDLER_INFO *pluginfo)
{
	printf(" %s> Update Status", strPluginName);
	if (pluginfo)
		memcpy(&g_PluginInfo, pluginfo, sizeof(HANDLER_INFO));
	else
	{
		memset(&g_PluginInfo, 0, sizeof(HANDLER_INFO));
		snprintf(g_PluginInfo.Name, sizeof(g_PluginInfo.Name), "%s", strPluginName);
		g_PluginInfo.RequestID = iRequestID;
		g_PluginInfo.ActionID = iActionID;
	}
}

/* **************************************************************************************
 *  Function Name: Handler_Start
 *  Description: Start Running
 *  Input :  None
 *  Output: None
 *  Return:  0  : Success Init Handler
 *              -1 : Fail Init Handler
 * ***************************************************************************************/
int HANDLER_API Handler_Start(void)
{
	char* result = NULL;
	char modulePath[200] = { 0 };
	char iniPath[200] = { 0 };
	printf("> %s Handler_Start\r\n", strPluginName);

	srand(time(NULL));

	// Load ini file
	bFind = read_INI_Platform(modulePath, iniPath);
	bFind = read_INI_Devices();
	bFind = read_INI_DeviceDetail();
	if (bFind)
	{
		bAllDataAlloc = true;
	}

	if (Modbus_Log)
	{
		MRLog_path = (char *)calloc(1, strlen(modulePath) + strlen("MR.txt") + 1);
		MSLog_path = (char *)calloc(1, strlen(modulePath) + strlen("MS.txt") + 1);
		strcpy(MRLog_path, modulePath);
		strcpy(MSLog_path, modulePath);
		strcat(MRLog_path, "MR.txt");
		strcat(MSLog_path, "MS.txt");
	}

	g_ThresholdSetContex.isThreadRunning = false;
	g_ThresholdDeleteContex.isThreadRunning = false;

	g_RetrieveContex.isThreadRunning = true;
	g_AutoReportContex.isThreadRunning = true;
	g_AutoUploadContex.isThreadRunning = true;
	g_ThresholdCheckContex.isThreadRunning = true;

	if (bFind)
	{
		AI_Regs_temp_Ary = (uint16_t *)calloc(2, sizeof(uint16_t));
		AO_Regs_temp_Ary = (uint16_t *)calloc(2, sizeof(uint16_t));
		AO_Set_Regs_temp_Ary = (uint16_t *)calloc(2, sizeof(uint16_t));
		if (!bIsSimtor)
		{
			if (iTCP_RTU == 0)
			{
				ctx = modbus_new_tcp(Modbus_Clent_IP, Modbus_Client_Port);
				Modbus_Connect();
			}
			else if (iTCP_RTU == 1)
			{
				char com_str[16] = "";
				char parity = 'N';

#if defined(_WIN32) || defined(WIN32)
				sprintf(com_str, "\\\\.\\%s", Modbus_Slave_Port);
#else
				sprintf(com_str, "/dev/%s", Modbus_Slave_Port);
#endif
				printf("com_str : %s\n", com_str);

				if (strcmp(Modbus_Parity, "None") == 0)
					parity = 'N';
				else if (strcmp(Modbus_Parity, "Even") == 0)
					parity = 'E';
				else if (strcmp(Modbus_Parity, "Odd") == 0)
					parity = 'O';
				else
					;	// Not N, E, O

				ctx = modbus_new_rtu(com_str, Modbus_Baud, parity, Modbus_DataBits, Modbus_StopBits);
				Modbus_Connect();
			}
			else
			{
				printf("Protocol error!!\n");
				ctx = NULL;
			}
		}
	}
	else
	{
		printf("INI file not found!!\n");
		ctx = NULL;
	}

	MODBUSthresholdList = CreateThresholdList();

	if (pthread_create(&g_RetrieveContex.threadHandler, NULL, RetrieveThreadStart, &g_RetrieveContex) != 0)
	{
		g_RetrieveContex.isThreadRunning = false;
		printf("> start Retrieve thread failed!\r\n");
		return handler_fail;
	}

	if (pthread_create(&g_AutoReportContex.threadHandler, NULL, AutoReportThreadStart, &g_AutoReportContex) != 0)
	{
		g_AutoReportContex.isThreadRunning = false;
		printf("> start AutoReport thread failed!\r\n");
		return handler_fail;
	}

	if (pthread_create(&g_AutoUploadContex.threadHandler, NULL, AutoUploadThreadStart, &g_AutoUploadContex) != 0)
	{
		g_AutoUploadContex.isThreadRunning = false;
		printf("> start AutoUpload thread failed!\r\n");
		return handler_fail;
	}

	if (pthread_create(&g_ThresholdCheckContex.threadHandler, NULL, ThresholdCheckThreadStart, &g_ThresholdCheckContex) != 0)
	{
		g_ThresholdCheckContex.isThreadRunning = false;
		printf("> start ThresholdCheck thread failed!\r\n");
		return handler_fail;
	}

	g_status = handler_status_start;

	time(&g_monitortime);
	return handler_success;
}

/* **************************************************************************************
 *  Function Name: Handler_Stop
 *  Description: Stop the handler
 *  Input :  None
 *  Output: None
 *  Return:  0  : Success Init Handler
 *              -1 : Fail Init Handler
 * ***************************************************************************************/
int HANDLER_API Handler_Stop(void)
{
	void *status;

	if (g_RetrieveContex.isThreadRunning == true)
	{
		g_RetrieveContex.isThreadRunning = false;
		pthread_join((pthread_t)g_RetrieveContex.threadHandler, &status);
		g_RetrieveContex.threadHandler = NULL;
	}

	if (g_AutoReportContex.isThreadRunning == true)
	{
		int i = 0;
		g_AutoReportContex.isThreadRunning = false;
		pthread_join((pthread_t)g_AutoReportContex.threadHandler, &status);
		g_AutoReportContex.threadHandler = NULL;

		for (i = 0; i < Report_array_size; i++)
		{
			if (Report_Data_paths != NULL)
				if (Report_Data_paths[i] != NULL)
					free(Report_Data_paths[i]);
		}
		if (Report_Data_paths != NULL)
		{
			free(Report_Data_paths);
			Report_Data_paths = NULL;
		}
	}

	if (g_AutoUploadContex.isThreadRunning == true)
	{

		g_AutoUploadContex.isThreadRunning = false;
		pthread_join((pthread_t)g_AutoUploadContex.threadHandler, &status);
		g_AutoUploadContex.threadHandler = NULL;
	}

	if (g_ThresholdCheckContex.isThreadRunning == true)
	{
		g_ThresholdCheckContex.isThreadRunning = false;
		pthread_join((pthread_t)g_ThresholdCheckContex.threadHandler, &status);
		g_ThresholdCheckContex.threadHandler = NULL;
	}

	if (g_ThresholdSetContex.isThreadRunning == true)
	{
		g_ThresholdSetContex.isThreadRunning = false;
		pthread_join((pthread_t)g_ThresholdSetContex.threadHandler, &status);
		g_ThresholdSetContex.threadHandler = NULL;
	}

	if (g_ThresholdDeleteContex.isThreadRunning == true)
	{
		g_ThresholdDeleteContex.isThreadRunning = false;
		pthread_join((pthread_t)g_ThresholdDeleteContex.threadHandler, &status);
		g_ThresholdDeleteContex.threadHandler = NULL;
	}

	for (int devCount = 0; devCount < numberOfWDev; devCount++)
	{
		if (WDev[devCount].DI_Bits)
		{
			free(WDev[devCount].DI_Bits);
			WDev[devCount].DI_Bits = NULL;
		}

		if (WDev[devCount].DO_Bits)
		{
			free(WDev[devCount].DO_Bits);
			WDev[devCount].DO_Bits = NULL;
		}

		if (WDev[devCount].AI_Regs)
		{
			free(WDev[devCount].AI_Regs);
			WDev[devCount].AI_Regs = NULL;
		}

		if (AI_Regs_temp_Ary)
		{
			free(AI_Regs_temp_Ary);
			AI_Regs_temp_Ary = NULL;
		}

		if (WDev[devCount].AO_Regs)
		{
			free(WDev[devCount].AO_Regs);
			WDev[devCount].AO_Regs = NULL;
		}

		if (AO_Regs_temp_Ary)
		{
			free(AO_Regs_temp_Ary);
			AO_Regs_temp_Ary = NULL;
		}

		if (AO_Set_Regs_temp_Ary)
		{
			free(AO_Set_Regs_temp_Ary);
			AO_Set_Regs_temp_Ary = NULL;
		}


		if (WDev[devCount].DI)
		{
			free(WDev[devCount].DI);
			WDev[devCount].DI = NULL;
		}

		if (WDev[devCount].DO)
		{
			free(WDev[devCount].DO);
			WDev[devCount].DO = NULL;
		}

		if (WDev[devCount].AI)
		{
			free(WDev[devCount].AI);
			WDev[devCount].AI = NULL;
		}

		if (WDev[devCount].AO)
		{
			free(WDev[devCount].AO);
			WDev[devCount].AO = NULL;
		}

		if (WDev[devCount].DI_Blocks)
		{
			free(WDev[devCount].DI_Blocks);
			WDev[devCount].DI_Blocks = NULL;
		}

		if (WDev[devCount].DO_Blocks)
		{
			free(WDev[devCount].DO_Blocks);
			WDev[devCount].DO_Blocks = NULL;
		}

		if (WDev[devCount].AI_Blocks)
		{
			free(WDev[devCount].AI_Blocks);
			WDev[devCount].AI_Blocks = NULL;
		}

		if (WDev[devCount].AO_Blocks)
		{
			free(WDev[devCount].AO_Blocks);
			WDev[devCount].AO_Blocks = NULL;
		}

		if (WDev[devCount].DIB)
		{
			//free(WDev[devCount].DIB);
			WDev[devCount].DIB = NULL;
		}

		if (WDev[devCount].DOB)
		{
			//free(WDev[devCount].DOB);
			WDev[devCount].DOB = NULL;
		}

		if (WDev[devCount].AIB)
		{
			free(WDev[devCount].AIB);
			WDev[devCount].AIB = NULL;
		}

		if (WDev[devCount].AOB)
		{
			free(WDev[devCount].AOB);
			WDev[devCount].AOB = NULL;
		}
	}

	if (modbus_get_socket(ctx) > 0)
		Modbus_Disconnect();
	modbus_free(ctx);

	g_status = handler_status_stop;
	return handler_success;
}

/* **************************************************************************************
 *  Function Name: Handler_Recv
 *  Description: Receive Packet from MQTT Server
 *  Input : char * const topic,
 *			void* const data,
 *			const size_t datalen
 *  Output: void *pRev1,
 *			void* pRev2
 *  Return: None
 * ***************************************************************************************/
void HANDLER_API Handler_Recv(char * const topic, void* const data, const size_t datalen, void *pRev1, void* pRev2)
{
	int cmdID = 0;
	char errorStr[128] = { 0 };
	ModbusLog(g_loghandle, Normal, " %s>Recv Topic [%s] Data %s", strPluginName, topic, (char*)data);
	printf(" >Recv Topic [%s] Data %s", topic, (char*)data);

	if (!ParseReceivedData(data, datalen, &cmdID))
		return;

	switch (cmdID)
	{
	case modbus_get_capability_req:
	{
		GetCapability();
		break;
	}

	case modbus_get_sensors_data_req:
	{
		char curSessionID[256] = { 0 };
		sensor_info_list sensorInfoList = CreateSensorInfoList();
		if (Parser_ParseGetSensorDataReqEx(data, sensorInfoList, curSessionID))
		{
			if (strlen(curSessionID))
			{
				if (sensorInfoList != NULL || curSessionID != NULL)
					GetSensorsDataEx(sensorInfoList, curSessionID);
			}
		}
		else
		{
			char * errorRepJsonStr = NULL;
			char errorStr[128];
			sprintf(errorStr, "Command(%d) parse error!", modbus_get_sensors_data_req);
			int jsonStrlen = Parser_PackModbusError(errorStr, &errorRepJsonStr);

			if (jsonStrlen > 0 && errorRepJsonStr != NULL)
			{
				g_sendcbf(&g_PluginInfo, modbus_error_rep, errorRepJsonStr, strlen(errorRepJsonStr) + 1, NULL, NULL);
			}
			if (errorRepJsonStr)
				free(errorRepJsonStr);
		}
		DestroySensorInfoList(sensorInfoList);
		break;
	}

	case modbus_set_sensors_data_req:
	{
		char curSessionID[256] = { 0 };
		sensor_info_list sensorInfoList = CreateSensorInfoList();
		if (Parser_ParseSetSensorDataReqEx(data, sensorInfoList, curSessionID))
		{
			if (strlen(curSessionID))
			{
				if (sensorInfoList != NULL || curSessionID != NULL)
					SetSensorsDataEx(sensorInfoList, curSessionID);
			}
		}
		else
		{
			char * errorRepJsonStr = NULL;
			char errorStr[128];
			sprintf(errorStr, "Command(%d) parse error!", modbus_set_sensors_data_req);
			int jsonStrlen = Parser_PackModbusError(errorStr, &errorRepJsonStr);
			//printf("error : %s\n",errorStr);
			if (jsonStrlen > 0 && errorRepJsonStr != NULL)
			{
				g_sendcbf(&g_PluginInfo, modbus_error_rep, errorRepJsonStr, strlen(errorRepJsonStr) + 1, NULL, NULL);
			}
			if (errorRepJsonStr)
				free(errorRepJsonStr);
		}
		DestroySensorInfoList(sensorInfoList);
		break;
	}

	case modbus_auto_upload_req:
	{
		unsigned int intervalTimeMs = 0; //ms
		unsigned int continueTimeMs = 0;
		char tmpRepFilter[4096] = { 0 };
		bool bRet = Parser_ParseAutoUploadCmd((char *)data, &intervalTimeMs, &continueTimeMs, tmpRepFilter);

		if (bRet)
		{
			AutoUploadParams.intervalTimeMs = intervalTimeMs; //ms
			AutoUploadParams.continueTimeMs = continueTimeMs;
			memset(AutoUploadParams.repFilter, 0, sizeof(AutoUploadParams.repFilter));
			if (strlen(tmpRepFilter)) strcpy(AutoUploadParams.repFilter, tmpRepFilter);
			g_bAutoUpload = true;
			Start_Upload = time(NULL);
		}
		else
		{
			char * errorRepJsonStr = NULL;
			char errorStr[128];
			sprintf(errorStr, "Command(%d) parse error!", modbus_set_sensors_data_req);
			int jsonStrlen = Parser_PackModbusError(errorStr, &errorRepJsonStr);

			if (jsonStrlen > 0 && errorRepJsonStr != NULL)
			{
				g_sendcbf(&g_PluginInfo, modbus_error_rep, errorRepJsonStr, strlen(errorRepJsonStr) + 1, NULL, NULL);
			}
			if (errorRepJsonStr)
				free(errorRepJsonStr);
		}
		break;
	}

	case modbus_set_thr_req:
	{
		Threshold_Data = (char *)calloc(1, strlen((char *)data) + 1);
		strcpy(Threshold_Data, (char *)data);
		SetThreshold();
		break;
	}

	case modbus_del_thr_req:
	{
		DeleteThreshold();
		break;
	}

	}
}

/* **************************************************************************************
 *  Function Name: Handler_AutoReportStart
 *  Description: Start Auto Report
 *  Input : char *pInQuery
 *  Output: None
 *  Return: None
 * ***************************************************************************************/
 //FULL_FUNC : For distinghishing handler's name having Modbus_Handler or not in AutoReport
 //No FULL_FUNC : Not dinsinghish handler's name having Modbus_Handler or not in AutoReport
void HANDLER_API Handler_AutoReportStart(char *pInQuery)
{

#ifdef FULL_FUNC	//Server doesnt support Modbus_Handler now  
	int i = 0;

	if (g_bAutoReport)
	{
		for (i = 0; i < Report_array_size; i++)
		{
			if (Report_Data_paths != NULL)
				if (Report_Data_paths[i] != NULL)
					free(Report_Data_paths[i]);
		}
		if (Report_Data_paths != NULL)
		{
			free(Report_Data_paths);
			Report_Data_paths = NULL;
		}
	}

#pragma region root 
	//printf("\n\npInQuery = %s\n\n",pInQuery);
	Report_item = NULL;
	Report_it = NULL;
	Report_js_name = NULL;
	Report_first = NULL;
	Report_second_interval = NULL;
	Report_second = NULL;
	Report_third = NULL;
	Report_js_list = NULL;

	Report_root = NULL;
	Report_Reply_All = false;
	Report_Data_paths = NULL;
	Report_array_size = 0;

	Report_root = cJSON_Parse(pInQuery);
#pragma region Report_root
	if (!Report_root)
	{
		printf("get root failed !\n");

	}
	else
	{
		Report_first = cJSON_GetObjectItem(Report_root, "susiCommData");
		if (Report_first)
		{
			Report_second_interval = cJSON_GetObjectItem(Report_first, "autoUploadIntervalSec");
			if (Report_second_interval)
			{
				Report_interval = Report_second_interval->valueint;
				//printf("interval : %d\n",Report_interval);
				Report_second = cJSON_GetObjectItem(Report_first, "requestItems");
				if (Report_second)
				{
					Report_third = cJSON_GetObjectItem(Report_second, "All");
					if (Report_third)
						Report_Reply_All = true;
					else
					{
						//Report_third=cJSON_GetObjectItem(Report_second,DEF_HANDLER_NAME);
						Report_third = cJSON_GetObjectItem(Report_second, strPluginName);
						if (Report_third)
							Report_js_list = cJSON_GetObjectItem(Report_third, "e");

					}

				}
			}
		}

#pragma region Report_Reply_All
		if (!Report_Reply_All)
		{
#pragma region Report_js_list
			if (!Report_js_list)
			{
				printf("no list!\n");
				g_bAutoReport = false;
			}
			else
			{
				Report_array_size = cJSON_GetArraySize(Report_js_list);
				Report_Data_paths = (char **)calloc(Report_array_size, sizeof(char *));

				char *p = NULL;

				for (i = 0; i < Report_array_size; i++)
				{

					Report_item = cJSON_GetArrayItem(Report_js_list, i);
					if (Report_Data_paths)
						Report_Data_paths[i] = (char *)calloc(200, sizeof(char));   //set Data_path size as 200*char
					if (!Report_item)
					{
						printf("no item!\n");
					}
					else
					{
						p = cJSON_PrintUnformatted(Report_item);
						Report_it = cJSON_Parse(p);

						if (!Report_it)
							continue;
						else
						{
							Report_js_name = cJSON_GetObjectItem(Report_it, "n");
							if (Report_Data_paths)
								if (Report_Data_paths[i])
									strcpy(Report_Data_paths[i], Report_js_name->valuestring);
							//printf("Data : %s\n",Report_Data_paths[i]);
						}

						if (p)
							free(p);
						if (Report_it)
							cJSON_Delete(Report_it);

						if (strcmp(Report_Data_paths[i], strPluginName) == 0)
						{
							Report_Reply_All = true;
							printf("Report_Reply_All=true;\n");
							break;
						}

					}

				}
				if (Report_third)
					g_bAutoReport = true;
				else
					g_bAutoReport = false;

			}
#pragma endregion Report_js_list
		}
		else
		{
			printf("Report_Reply_All=true;\n");
			if (Report_third)
				g_bAutoReport = true;
			else
				g_bAutoReport = false;
		}
#pragma endregion Report_Reply_All
	}

	if (Report_root)
		cJSON_Delete(Report_root);
#pragma endregion Report_root
#else
	Report_root = NULL;
	Report_root = cJSON_Parse(pInQuery);
	if (!Report_root)
	{
		printf("get root failed !\n");
	}
	else
	{
		Report_first = cJSON_GetObjectItem(Report_root, "susiCommData");
		if (Report_first)
		{
			Report_second_interval = cJSON_GetObjectItem(Report_first, "autoUploadIntervalSec");
			if (Report_second_interval)
			{
				Report_interval = Report_second_interval->valueint;
			}
		}
	}
	if (Report_root)
		cJSON_Delete(Report_root);

	printf("Reply_All=true;\n");
	//Report_interval=1;
	Report_Reply_All = true;
	g_bAutoReport = true;
#endif
}

/* **************************************************************************************
 *  Function Name: Handler_AutoReportStop
 *  Description: Stop Auto Report
 *  Input : None
 *  Output: None
 *  Return: None
 * ***************************************************************************************/
void HANDLER_API Handler_AutoReportStop(char *pInQuery)
{
	/*TODO: Parsing received command*/
#ifdef FULL_FUNC	//Server doesnt support Modbus_Handler now  
	int i = 0;
	for (i = 0; i < Report_array_size; i++)
	{
		if (Report_Data_paths != NULL)
			if (Report_Data_paths[i] != NULL)
				free(Report_Data_paths[i]);
	}
	if (Report_Data_paths != NULL)
	{
		free(Report_Data_paths);
		Report_Data_paths = NULL;
	}
	g_bAutoReport = false;
#else
	g_bAutoReport = false;
#endif
}

/* **************************************************************************************
 *  Function Name: Handler_Get_Capability
 *  Description: Get Handler Information specification.
 *  Input :  None
 *  Output: char ** : pOutReply       // JSON Format
 *  Return:  int  : Length of the status information in JSON format
 *                :  0 : no data need to trans
 * **************************************************************************************/
int HANDLER_API Handler_Get_Capability(char ** pOutReply) // JSON Format
{
	char* result = NULL;
	int len = 0;
	if (!pOutReply) return len;
	//bFind = read_INI();
	if (g_Capability)
	{
		IoT_ReleaseAll(g_Capability);
		g_Capability = NULL;
	}
	//if(bFind)
	//	bAllDataAlloc=true;

	g_Capability = CreateCapability();

	result = IoT_PrintCapability(g_Capability);

	printf("Handler_Get_Capability=%s\n", result);
	printf("---------------------\n");

	len = strlen(result);
	*pOutReply = (char *)malloc(len + 1);
	memset(*pOutReply, 0, len + 1);
	strcpy(*pOutReply, result);
	free(result);
	return len;
}
