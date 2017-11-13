//
// Copyright 2017 InSys
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "bambp_impl.hpp"

#include "ctrladc.h"
#include "ctrlddc.h"
#include "ctrlsdram.h"
#include "ctrlcmpsc.h"
#include "gipcy.h"

#include <string>

#include <uhd/utils/static.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/ranges.hpp>

using namespace std;
using namespace uhd;
using namespace uhd::usrp;

#define DDC_CHAN_NUM 16

device_addrs_t bambp_find(const device_addr_t &hint_)
{
	device_addrs_t	addrs;
	S32 nNum = 0;
	S32 err = 0;
	U32 *pLidList;
	U32 nItemReal;
	U32 i;
	char sVal[256];
	BRD_Info rBrdInfo;
	string sSerial;

	if((hint_.size() > 0) && !hint_.has_key("serial"))
		return addrs;

	sSerial = hint_.get("serial", "");

	err = BRD_initEx(0, 0, 0, &nNum);

	if(!BRD_errcmp(err, BRDerr_OK))
	{
		printf("BRD_initEx() - error!\n");
		BRD_cleanup();
		return addrs;
	}

	pLidList = (U32*)malloc(sizeof(U32) * nNum);

	err = BRD_lidList(pLidList, nNum, &nItemReal);

	if(!BRD_errcmp(err, BRDerr_OK))
	{
		free(pLidList);
		BRD_cleanup();
		return addrs;
	}

	rBrdInfo.size = sizeof(BRD_Info);

	if(hint_.size() == 0)
		nItemReal = 1;

	for(i = 0; i < nItemReal; i++)
	{
		device_addr_t	addr;

		err = BRD_getInfo(pLidList[i], &rBrdInfo);

		if(!BRD_errcmp(err, BRDerr_OK))
			continue;

		addr["serial"] = to_string(rBrdInfo.pid);

		BRDC_bcstombs(sVal, rBrdInfo.name, 256);
		addr["name"] = sVal;
		
		if(!sSerial.empty() && (sSerial != addr["serial"]))
			continue;

		addr["lid"] = to_string(pLidList[i]);;

		addrs.push_back(addr);
	}

	free(pLidList);
	BRD_cleanup();
	return addrs;
}

/***********************************************************************
* Make
**********************************************************************/
static device::sptr bambp_make(const device_addr_t &device_addr)
{
	return device::sptr(new bambp_impl(device_addr));
}

UHD_STATIC_BLOCK(register_bambp_device)
{
	device::register_device(&bambp_find, &bambp_make, device::USRP);
}

bambp_impl::bambp_impl(const uhd::device_addr_t &dev_addr)
{
	S32 nNum = 0;
	S32 err = 0;
	S32 nLid = 0;
	U32 ItemReal;

	strcpy(m_iniFileName, "/exam_ddc.ini");

	strcpy(m_DdcSrvName, "DDC4X160");

	m_handle = 0;
	m_hDDC = 0;
	m_hADC = 0;

	err = BRD_initEx(0, 0, 0, &nNum);

	if(!BRD_errcmp(err, BRDerr_OK))
	{
		BRD_cleanup();
		return;
	}

	nLid = stoi(dev_addr["lid"]);

	m_handle = BRD_open(nLid, BRDopen_SHARED, 0);
	
	if(m_handle < 1)
	{
		BRD_cleanup();
		return;
	}

	BRD_ServList srvList[10];
	err = BRD_serviceList(m_handle, 0, srvList, 10, &ItemReal);

	_tree = uhd::property_tree::make();

	UHD_MSG(status) << "AMBPCX initialization sequence..." << std::endl;
	_tree->create<std::string>("/name").set("AMBPCX Device");

	ReadIniFile();

	SetParamSrv();

	this->setup_mb(0, dev_addr);
}

bambp_impl::~bambp_impl()
{
	if(m_handle)
		BRD_close(m_handle);

	if(m_hADC)
		BRD_release(m_hADC, 0);

	if(m_hDDC)
		BRD_release(m_hDDC, 0);
}

void bambp_impl::GetInifileString(const char *FileName, const char *SectionName, const char *ParamName, const char *defValue, char *strValue, int strSize)
{
	IPC_getPrivateProfileString(SectionName, ParamName, defValue, strValue, strSize, FileName);

	// удалить комментарий из строки 
	char* pChar = strchr(strValue, ';'); // признак комментария или ;
	if(pChar) *pChar = 0;
	pChar = strchr(strValue, '/');			// или //
	if(pChar) if(*(pChar + 1) == '/')	*pChar = 0;

	// Удалить пробелы в конце строки
	int str_size = (int)strlen(strValue);
	for(int i = str_size - 1; i > 1; i--)
		if(strValue[i] != ' ' && strValue[i] != '\t')
		{
			strValue[i + 1] = 0;
			break;
		}
}

void bambp_impl::ReadIniFile()
{
	char Buffer[128];
	char iniFilePath[MAX_PATH];

	IPC_getCurrentDir(iniFilePath, sizeof(iniFilePath));
	strcat(iniFilePath, m_iniFileName);
	char SrvName[32];	// имя службы (без номера)

	sprintf(iniFilePath, "c:\\Program Files\\GNURadio-3.7\\bin\\exam_ddc.ini");

	GetInifileString(iniFilePath, "Option", "AdcServiceName", "ADC4X16", SrvName, sizeof(SrvName));
	sprintf(m_AdcSrvName, "%s%d", SrvName, 0); // имя службы ADC (с номером)

	GetInifileString(iniFilePath, "Option", "DdcServiceName", "DDC4X16", SrvName, sizeof(SrvName));
	sprintf(m_DdcSrvName, "%s%d", SrvName, 0); // имя службы DDC(с номером)

	GetInifileString(iniFilePath, "Option", "PldFileName", "ambpcx_v10_admddc4x16.mcs", m_pldFileName, sizeof(m_pldFileName));
	//GetPrivateProfileString("Option", "PldFileName", "ambpcx_v10_admddc4x16_c.mcs", g_pldFileName, sizeof(g_pldFileName), iniFilePath);
	//BRDCHAR* pChar = _tcschr(g_pldFileName, ';');
	//if(pChar) *pChar = 0; // удалить комментарий из строки
	//pChar = _tcschr(g_pldFileName, ' ');
	//if(pChar) *pChar = 0; // удалить пробелы из строки
	//pChar = strchr(g_pldFileName, '\t');
	//if(pChar) *pChar = 0; // удалить табуляции из строки

	IPC_getPrivateProfileString("Option", "IsPldLoadAlways", "0", Buffer, sizeof(Buffer), iniFilePath);
	m_isPldLoadAlways = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "DMA", "1", Buffer, sizeof(Buffer), iniFilePath);
	m_DmaOn = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "Cycle", "1", Buffer, sizeof(Buffer), iniFilePath);
	m_Cycle = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "DaqIntoMemory", "0", Buffer, sizeof(Buffer), iniFilePath);
	m_MemOn = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "IsWriteFile", "1", Buffer, sizeof(Buffer), iniFilePath);
	m_IsWriteFile = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "SamplesPerChannel", "10240", Buffer, sizeof(Buffer), iniFilePath);
	m_samplesOfChannel = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "MemSamplesPerChan", "10240", Buffer, sizeof(Buffer), iniFilePath);
	m_memorySamplesOfChannel = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "IsSystemMemory", "0", Buffer, sizeof(Buffer), iniFilePath);
	m_IsSysMem = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "EnableDDC", "0", Buffer, sizeof(Buffer), iniFilePath);
	m_DdcOn = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "WorkMode", "0", Buffer, sizeof(Buffer), iniFilePath);
	m_DirWriteFile = atoi(Buffer);
	IPC_getPrivateProfileString("Option", "DirFileBufSize", "64", Buffer, sizeof(Buffer), iniFilePath); // KBytes
	m_FileBufSize = atoi(Buffer) * 1024;
	IPC_getPrivateProfileString("Option", "DirNumBufWrite", "4", Buffer, sizeof(Buffer), iniFilePath); // KBytes
	m_DirWriteFile = m_DirWriteFile ? atoi(Buffer) : 0;

	GetInifileString(iniFilePath, "Option", "DataFileName", "data", m_dataFileName, sizeof(m_dataFileName));

	IPC_getPrivateProfileString("Option", "TimeoutSec", "5", Buffer, sizeof(Buffer), iniFilePath); // sec
	m_MsTimeout = atoi(Buffer) * 1000;
	IPC_getPrivateProfileString("Option", "DirTimeoutSec", "5", Buffer, sizeof(Buffer), iniFilePath); // sec
	m_DirMsTimeout = atoi(Buffer) * 1000;

	IPC_getPrivateProfileString("Option", "DrqFlag", "2", Buffer, sizeof(Buffer), iniFilePath);
	m_DrqFlag = atoi(Buffer);

	if(!m_flDirFileName)
	{
		m_flDirFileName = 1;
		GetInifileString(iniFilePath, "Option", "DirFileName", "ddc_.bin", m_dirFileName, sizeof(m_dirFileName));
	}
}

S32	bambp_impl::SetParamSrv()
{
	S32		status = -1;
	U32 mode = BRDcapt_EXCLUSIVE;
	BRDCHAR adcName[32];
	BRDCHAR ddcName[32];

	BRDC_mbstobcs(adcName, m_AdcSrvName, 32);
	BRDC_mbstobcs(ddcName, m_DdcSrvName, 32);

	m_hADC = BRD_capture(m_handle, 0, &mode, adcName, 10000);
	
	if(mode != BRDcapt_EXCLUSIVE)
		BRDC_printf(_BRDC("%s: Capture mode NON EXCLUSIVE\n"), adcName);

	m_hDDC = BRD_capture(m_handle, 0, &mode, ddcName, 10000);
	
	if(mode != BRDcapt_EXCLUSIVE)
		BRDC_printf(_BRDC("%s: Capture mode NON EXCLUSIVE\n"), ddcName);
	
	if(m_hADC > 0 && m_hDDC > 0)
	{
		BRD_DdcCfg ddcCg;
		status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETCFG, &ddcCg);
		BRDC_printf(_BRDC("ADMFPGA = %0X\n"), ddcCg.AdmConst);
		m_ver = ddcCg.AdmConst >> 12;

		char iniFilePath[MAX_PATH];
		BRDCHAR iniSectionName[MAX_PATH];
		BRDCHAR ddcSrvName[MAX_PATH];
		BRDCHAR filePath[MAX_PATH];

		IPC_getCurrentDir(iniFilePath, sizeof(iniFilePath));
		strcat(iniFilePath, m_iniFileName);
		BRDC_mbstobcs(filePath, iniFilePath, MAX_PATH);
		
		BRDC_mbstobcs(ddcSrvName, m_DdcSrvName, MAX_PATH);
		BRDC_sprintf(iniSectionName, _BRDC("device0_%s"), ddcSrvName);

		BRDC_sprintf(filePath, _BRDC("c:\\Program Files\\GNURadio-3.7\\bin\\exam_ddc.ini"));

		//BRDCHAR Buffer[128];
		//BRDCHAR* endptr;
		//GetPrivateProfileString(iniSectionName, "ChannelMask", "1", Buffer, sizeof(Buffer), iniFilePath);
		//ULONG chan_mask = strtol(Buffer, &endptr, 0);

		ULONG format = 0;
		ULONG sample_size, adc_sample_size;
		int numChan = 0, adc_numChan = 0;

		{
			status = AdcSettings(iniSectionName, filePath); // установить параметры АЦП

			ULONG adc_mask;
			status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETCHANMASK, &adc_mask);
			if(!adc_mask)
				return status;

			for(int iAdc = 0; iAdc < 4; iAdc++)
				adc_numChan += (adc_mask >> iAdc) & 0x1;

			numChan = adc_numChan;

			status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETFORMAT, &format);
			adc_sample_size = format ? format : sizeof(short);
			sample_size = adc_sample_size;
		}

		if(m_DdcOn)
		{
			status = DdcSettings(iniSectionName, filePath); // установить параметры DDC
			if(!BRD_errcmp(status, BRDerr_OK))
				return status;

			ULONG ddc_mask;
			status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETCHANMASK, &ddc_mask);
			if(!ddc_mask)
				return status;
			//			ULONG ddc_mask = 0xffff; // включаем все DDC
			//ULONG ddc_mask = 0x1; // включаем все 4 канала 1-ой микросхемы DDC
			//status = BRD_ctrl(hDDC, 0, BRDctrl_DDC_SETCHANMASK, &ddc_mask);
			//if(BRD_errcmp(status, BRDerr_OK))
			//	printf("BRDctrl_DDC_SETCHANMASK: ddc_mask = %0X\n", ddc_mask);
			//else
			//	DisplayError(status, "BRDctrl_DDC_SETCHANMASK");

			for(int iDdc = 0; iDdc < 16; iDdc++)
				numChan += (ddc_mask >> iDdc) & 0x1;

			status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETFORMAT, &format);
			sample_size = format ? format : sizeof(short);
			sample_size <<= 1;
		}


		if(m_DdcOn == 2)
		{
			m_bMemBufSize = (m_memorySamplesOfChannel * adc_numChan) * adc_sample_size; // получить размер собираемых в память данных в байтах


			m_bBufSize = (m_samplesOfChannel * numChan) * sample_size; // получить размер собираемых данных в байтах

																				   // проверяем наличие динамической памяти
			BRD_SdramCfg SdramConfig;
			ULONG PhysMemSize;
			status = BRD_ctrl(m_hDDC, 0, BRDctrl_SDRAM_GETCFG, &SdramConfig);
			if(status < 0)
			{
				if(m_MemOn)
				{
					BRDC_printf(_BRDC("Get SDRAM Config: Error!!!\n"));
					m_MemOn = 0;
				}
				PhysMemSize = 0;
				return BRDerr_OK;
			}
			else
				PhysMemSize = (1 << SdramConfig.RowAddrBits) *
				(1 << SdramConfig.ColAddrBits) *
				SdramConfig.ModuleBanks *
				SdramConfig.ChipBanks *
				SdramConfig.ModuleCnt * 2; // в 32-битных словах

			if(PhysMemSize && m_MemOn)
			{ // динамическая память присутствует на модуле
				BRDC_printf(_BRDC("SDRAM Config: Memory size = %d MBytes\n"), (PhysMemSize / (1024 * 1024)) * 4);

				status = SdramSettings(m_hDDC, m_bMemBufSize, 0, m_MemOn); // установить параметры SDRAM
				m_memorySamplesOfChannel = (m_bMemBufSize / adc_sample_size) / adc_numChan;
				if(m_MemOn == 2)
				{
					BRDC_printf(_BRDC("SDRAM as a FIFO mode!!!\n"));
					m_MemOn = 0;
					m_MemAsFifo = 1;
				}
				else
					BRDC_printf(_BRDC("Memory samples of channel = %d\n"), m_memorySamplesOfChannel);
			}
			else
			{ // освободить службу SDRAM (она могла быть захвачена командой BRDctrl_SDRAM_GETCFG, если та отработала без ошибки)
				ULONG mem_size = 0;
				status = BRD_ctrl(m_hDDC, 0, BRDctrl_SDRAM_SETMEMSIZE, &mem_size);
				if(m_MemOn)
				{
					BRDC_printf(_BRDC("No SDRAM on board!!!\n"));
					m_MemOn = 0;
				}
			}
			return status;
		}

		m_bBufSize = (m_samplesOfChannel * numChan) * sample_size; // получить размер собираемых данных в байтах
																			   //		printf("Samples of channel = %d\n", g_samplesOfChannel);

		m_bMemBufSize = (m_memorySamplesOfChannel * numChan) * sample_size; // получить размер собираемых в память данных в байтах

																						//ULONG sample_size = format ? format : sizeof(short);
																						//ULONG bBufSize = (g_samplesOfChannel * numChan) * sample_size; // получить размер собираемых данных в байтах

																						// проверяем наличие динамической памяти
		BRD_SdramCfg SdramConfig;
		ULONG PhysMemSize;
		status = BRD_ctrl(m_hDDC, 0, BRDctrl_SDRAM_GETCFG, &SdramConfig);
		if(status < 0)
			PhysMemSize = 0;
		else
			PhysMemSize = (1 << SdramConfig.RowAddrBits) *
			(1 << SdramConfig.ColAddrBits) *
			SdramConfig.ModuleBanks *
			SdramConfig.ChipBanks *
			SdramConfig.ModuleCnt * 2; // в 32-битных словах

		if(PhysMemSize && m_MemOn)
		{ // динамическая память присутствует на модуле и мы хотим ее использовать
			BRDC_printf(_BRDC("SDRAM Config: Memory size = %d MBytes\n"), (PhysMemSize / (1024 * 1024)) * 4);

			status = SdramSettings(m_hDDC, m_bMemBufSize, m_DdcOn, m_MemOn); // установить параметры SDRAM
			m_memorySamplesOfChannel = (m_bMemBufSize / sample_size) / numChan;
			if(m_MemOn == 2)
			{
				BRDC_printf(_BRDC("SDRAM as a FIFO mode!!!\n"));
				m_MemOn = 0;
				m_MemAsFifo = 1;
			}
			else
				BRDC_printf(_BRDC("Memory samples of channel = %d\n"), m_memorySamplesOfChannel);
		}
		else
		{ // освободить службу SDRAM (она могла быть захвачена командой BRDctrl_SDRAM_GETCFG, если та отработала без ошибки)
			ULONG mem_size = 0;
			status = BRD_ctrl(m_hDDC, 0, BRDctrl_SDRAM_SETMEMSIZE, &mem_size);
			status = BRDerr_OK;
		}
		//		status = BRD_release(hDDC, 0);
	}
	return status;
}

//***************************************************************************************
// установить параметры АЦП
S32 bambp_impl::AdcSettings(BRDCHAR *iniSectionName, BRDCHAR *iniFilePath)
{
	S32		status;

	// перед программированием АЦП необходимо установить режим для DDC
	ULONG master = BRDims_SINGLE; // независимый (одиночный) режим
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_SETMASTER, &master);

	// задать параметры из файла
	//TCHAR iniFilePath[MAX_PATH];
	//TCHAR iniSectionName[MAX_PATH];
	//GetCurrentDirectory(sizeof(iniFilePath)/sizeof(TCHAR), iniFilePath);
	//lstrcat(iniFilePath, iniFileName);
	//sBRDC_printf_s(iniSectionName, "device%d_%s%d", idx, srvName, 0);

	BRD_IniFile ini_file;
	BRDC_strcpy(ini_file.fileName, iniFilePath);
	//	lstrcpy(ini_file.sectionName, "device0_ddc4x160");
	BRDC_strcpy(ini_file.sectionName, iniSectionName);
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_READINIFILE, &ini_file);
	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_ADC_READINIFILE: OK\n"));
	else
	{
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_ADC_READINIFILE"));
		return status;
	}

	// получить (и задать) источник и значение тактовой частоты можно отдельной функцией
	BRD_ClkMode clk_mode;
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETCLKMODE, &clk_mode);
	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_ADC_GETCLKMODE: source = %d, value = %f\n"), clk_mode.src, clk_mode.value);
	else
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_ADC_GETCLKMODE"));

	//// получить (и задать) параметры стартовой синхронизации можно отдельной функцией
	//BRD_AdcStartMode start;
	//status = BRD_ctrl(hADC, 0, BRDctrl_ADC_GETSTARTMODE, &start);
	//if(BRD_errcmp(status, BRDerr_OK))
	//	BRDC_printf("BRDctrl_ADC_GETSTARTMODE: start source = %d\n", start.stndStart.startSrc);
	//else
	//	DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_ADC_GETSTARTMODE"));

	// получить параметры стартовой синхронизации
	// команда BRDctrl_ADC_GETSTARTMODE может получать 2 разные структуры 
	// разного размера
	// для определения какую из них использует данная служба применяем 
	// "трюк" с массивом start_struct[40] )))

	U08 start_struct[40]; // наибольшая из структур имеет размер 40 байт
	memset(start_struct, 0x5A, 40);
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETSTARTMODE, &start_struct);
	if(BRD_errcmp(status, BRDerr_OK))
	{
		if(start_struct[39] == 0x5A)
		{
			// стартовая схема на базовом модуле (используется структура по-меньше)
			BRD_StartMode* start = (BRD_StartMode*)start_struct;

			BRDC_printf(_BRDC("BRDctrl_ADC_GETSTARTMODE: start source = %d\n"), start->startSrc);
		}
		else
		{
			// стартовая схема на субмодуле (используется большая структура)
			BRD_AdcStartMode* start = (BRD_AdcStartMode*)start_struct;

			BRDC_printf(_BRDC("BRDctrl_ADC_GETSTARTMODE: start source = %d\n"), start->src);
		}
	}
	else
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_ADC_GETSTARTMODE"));

	//
	// Работа с ключевыси словами "ComparatorThresholdSDX", 
	// "ComparatorThresholdCHAN0", "ComparatorThresholdSUBM" отсутствует
	// 
	// Требуется реализовать как в exam_adc.exe
	//

	//	if(start.stndStart.startSrc == BRDsts_CMP1)
	//	{	// старт от компаратора 1 (от есть от сигнала с разъема SDX)
	//		BRDCHAR Buffer[128];
	//		BRD_CmpSC cmp_sc;
	//		cmp_sc.src = BRDcmps_EXTSTCLK;
	//		cmp_sc.thr[0] = 0.0;
	//#if defined(__IPC_WIN__) || defined(__IPC_LINUX__)
	//        IPC_getPrivateProfileString(iniSectionName, _BRDC("ComparatorThresholdSDX"), _BRDC("0.0"), Buffer, sizeof(Buffer), iniFilePath);
	//#else
	//        GetPrivateProfileString(iniSectionName, _BRDC("ComparatorThresholdSDX"), _BRDC("0.0"), Buffer, sizeof(Buffer), iniFilePath);
	//#endif
	//		cmp_sc.thr[1] = BRDC_atof(Buffer);//0.0;
	//
	//		// задать источник и пороги для компараторов
	//		status = BRD_ctrl(hADC, 0, BRDctrl_CMPSC_SET, &cmp_sc);
	//		if(BRD_errcmp(status, BRDerr_OK))
	//			BRDC_printf("BRDctrl_CMPSC_SET: comparator source = %d, threshold = %.2f\n", cmp_sc.src, cmp_sc.thr[1]);
	//		else
	//			DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_CMPSC_SET"));
	//	}

	ULONG adc_mask;
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETCHANMASK, &adc_mask);
	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_ADC_GETCHANMASK: adc_mask = %0X\n"), adc_mask);
	else
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_ADC_GETCHANMASK"));

	return status;
}

//***************************************************************************************
// установить параметры DDC
S32 bambp_impl::DdcSettings(BRDCHAR *iniSectionName, BRDCHAR *iniFilePath)
{
	S32		status;
	char	filePath[MAX_PATH];
	char	section[MAX_PATH];

	//ULONG master = BRDims_SINGLE; // независимый (одиночный) режим
	//status = BRD_ctrl(hDDC, 0, BRDctrl_DDC_SETMASTER, &master);
	//status = BRD_ctrl(hDDC, 0, BRDctrl_ADC_SETMASTER, &master);

	// задать параметры из файла
	//TCHAR iniFilePath[MAX_PATH];
	//TCHAR iniSectionName[MAX_PATH];
	//GetCurrentDirectory(sizeof(iniFilePath)/sizeof(TCHAR), iniFilePath);
	//lstrcat(iniFilePath, iniFileName);
	//sBRDC_printf_s(iniSectionName, "device%d_%s%d", idx, srvName, 0);

	// получить (и задать) источник и значение тактовой частоты
	BRD_ClkMode clk_mode;
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETCLKMODE, &clk_mode);

	clk_mode.src = BRDclks_EXTCLK;	// FIXME (Polepchuk made for FM416x125M)
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_SETCLKMODE, &clk_mode);

	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_DDC_SETCLKMODE: source = %d, value = %f\n"), clk_mode.src, clk_mode.value);
	else
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETCLKMODE"));

	BRD_IniFile ini_file;
	BRDC_strcpy(ini_file.fileName, iniFilePath);
	//	lstrcpy(ini_file.sectionName, "device0_ddc4x160");
	BRDC_strcpy(ini_file.sectionName, iniSectionName);
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_READINIFILE, &ini_file);
	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_DDC_READINIFILE: OK\n"));
	else
	{
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_READINIFILE"));
		return status;
	}

	BRDC_bcstombs(filePath, iniFilePath, MAX_PATH);
	BRDC_bcstombs(section, iniSectionName, MAX_PATH);

	BRD_StartMode start;
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETSTARTMODE, &start);
	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_DDC_GETSTARTMODE: start source = %d, restart = %d\n"), start.startSrc, start.reStartMode);
	else
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETSTARTMODE"));

	if(start.startSrc == BRDsts_CMP1)
	{	// старт от компаратора 1 (от есть от сигнала с разъема SDX)
		char Buffer[128];
		BRD_CmpSC cmp_sc;
		cmp_sc.src = BRDcmps_EXTSTCLK;
		cmp_sc.thr[0] = 0.0;

		IPC_getPrivateProfileString(section, "ComparatorThresholdSDX", "0.0", Buffer, sizeof(Buffer), filePath);
		cmp_sc.thr[1] = atof(Buffer);//0.0;

										  // задать источникик и пороги для компараторов
		status = BRD_ctrl(m_hDDC, 0, BRDctrl_CMPSC_SET, &cmp_sc);
		if(BRD_errcmp(status, BRDerr_OK))
			BRDC_printf(_BRDC("BRDctrl_CMPSC_SET: comparator source = %d, threshold = %.2f\n"), cmp_sc.src, cmp_sc.thr[1]);
		else
			DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_CMPSC_SET"));
	}
	//// для заказа с синтезатором с VCO 400MHz
	//if(start.startSrc == BRDsts_CMP0 || start.startSrc == BRDsts_CMP1)
	//{	// старт от компаратора 0 (сигнал с разъема ExtSt) или 1 (сигнал с разъема SDX)
	//	TCHAR Buffer[128];
	//	BRD_CmpSC cmp_sc;
	//	cmp_sc.src = BRDcmps_EXTSTCLK;
	//	GetPrivateProfileString(iniSectionName, "ComparatorThresholdExtSt", "0.0", Buffer, sizeof(Buffer), iniFilePath);
	//	cmp_sc.thr[0] = atof(Buffer);//0.0;
	//	GetPrivateProfileString(iniSectionName, "ComparatorThresholdSDX", "0.0", Buffer, sizeof(Buffer), iniFilePath);
	//	cmp_sc.thr[1] = atof(Buffer);//0.0;
	//	
	//	// задать источник и пороги для компараторов
	//	status = BRD_ctrl(hDDC, 0, BRDctrl_CMPSC_SET, &cmp_sc);
	//	if(BRD_errcmp(status, BRDerr_OK))
	//		BRDC_printf("BRDctrl_CMPSC_SET: comparator source = %d, thresholdExtSt = %.2f, thresholdSDX = %.2f\n",
	//													cmp_sc.src, cmp_sc.thr[0], cmp_sc.thr[1]);
	//	else
	//		DisplayError(status, "BRDctrl_CMPSC_SET");
	//}

	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETDDCSYNC, &m_ddc_sync);
	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_DDC_GETDDCSYNC: mode = %d\n"), m_ddc_sync.mode);
	else
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETDDCSYNC"));

	ULONG ddc_mask;
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETCHANMASK, &ddc_mask);
	if(BRD_errcmp(status, BRDerr_OK))
		BRDC_printf(_BRDC("BRDctrl_DDC_GETCHANMASK: ddc_mask = %0X\n"), ddc_mask);
	else
		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETCHANMASK"));

	/*
	if(g_ver != 5)
	{
	ULONG length = FRAME_LENGTH;
	status = BRD_ctrl(hDDC, 0, BRDctrl_DDC_SETFRAME, &length);
	}
	else
	{
	status = BRD_ctrl(hDDC, 0, BRDctrl_DDC_GETTITLEMODE, &g_title_mode);
	if(BRD_errcmp(status, BRDerr_OK))
	BRDC_printf("BRDctrl_DDC_GETTITLEMODE: enable = %d, size = %d\n", g_title_mode.enable, g_title_mode.value);
	else
	DisplayError(status, "BRDctrl_DDC_GETTITLEMODE");
	if(g_title_mode.value > 8)
	{
	BRD_PioDir pio_dir;
	pio_dir.lbDir = BRDpio_READ;
	pio_dir.hbDir = BRDpio_READ;
	status = BRD_ctrl(g_hPio, 0, BRDctrl_PIO_SETDIR, &pio_dir);
	if(BRD_errcmp(status, BRDerr_OK))
	BRDC_printf("BRDctrl_PIO_SETDIR: pio_dir.lbDir = %d, pio_dir.hbDir = %d\n", pio_dir.lbDir, pio_dir.hbDir);
	else
	DisplayError(status, "BRDctrl_PIO_SETDIR");
	}
	}
	*/
	/*	g_fc.chan = 0;
	status = BRD_ctrl(hDDC, 0, BRDctrl_DDC_GETFC, &g_fc);
	BRDC_printf("BRDctrl_DDC_SETFC: channel = %d, FC = %.2f\n", g_fc.chan, g_fc.value);
	g_fc.chan = 1;
	status = BRD_ctrl(hDDC, 0, BRDctrl_DDC_GETFC, &g_fc);
	BRDC_printf("BRDctrl_DDC_SETFC: channel = %d, FC = %.2f\n", g_fc.chan, g_fc.value);
	*/

	return status;
}

//***************************************************************************************
// установить параметры SDRAM
S32 bambp_impl::SdramSettings(BRD_Handle hADC, ULONG &bBuf_size, ULONG ddc_on, ULONG mem_mode)
{
	S32		status;

	ULONG ddc_target = 0; // будем осуществлять сбор данных от DDC в FIFO
	ULONG adc_target = 2; // будем осуществлять сбор данных от АЦП в память
	if(ddc_on)
	{
		ddc_target = 2; // будем осуществлять сбор данных от DDC в память
		adc_target = 0; // будем осуществлять сбор данных от АЦП  в FIFO
	}
	// желательно всегда указывать направления сбора и для АЦП и для DDC
	// иначе, при переключении направления без перезагрузки прошивки может не быть сбора данных
	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_SETTARGET, &ddc_target);
	status = BRD_ctrl(hADC, 0, BRDctrl_ADC_SETTARGET, &adc_target);

	if(mem_mode == 2)
	{	// as a FIFO
		ULONG fifo_mode = 1; // память используется как FIFO
		status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_SETFIFOMODE, &fifo_mode);
	}
	else
	{
		ULONG mode = 0; // применить автоматический режим чтения из памяти
		status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_SETREADMODE, &mode);

		ULONG addr = 0;
		status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_SETSTARTADDR, &addr); // установить адрес записи
		status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_SETREADADDR, &addr); // установить адрес чтения

		ULONG mem_size = bBuf_size >> 2; // получить размер активной зоны в 32-разрядных словах
		status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_SETMEMSIZE, &mem_size);
		bBuf_size = mem_size << 2; // получить фактический размер активной зоны в байтах
		if(BRD_errcmp(status, BRDerr_OK))
			BRDC_printf(_BRDC("BRDctrl_SDRAM_SETMEMSIZE: SDRAM buffer size = %d bytes\n"), bBuf_size);
		else
			DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_SDRAM_SETMEMSIZE"));
	}
	return status;
}

//***************************************************************************************
void bambp_impl::DisplayError(S32 status, const char *funcName, const BRDCHAR *cmd_str)
{
	S32	real_status = BRD_errext(status);
	BRDCHAR func_name[MAX_PATH];
#ifdef _WIN64
	mbstowcs(func_name, funcName, MAX_PATH);
#else
	BRDC_strcpy(func_name, funcName);
#endif 
	BRDCHAR msg[255];
	switch(real_status)
	{
	case BRDerr_OK:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_OK\n"), func_name, cmd_str);
		break;
	case BRDerr_BAD_MODE:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_BAD_MODE\n"), func_name, cmd_str);
		break;
	case BRDerr_INSUFFICIENT_SERVICES:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_INSUFFICIENT_SERVICES\n"), func_name, cmd_str);
		break;
	case BRDerr_BAD_PARAMETER:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_BAD_PARAMETER\n"), func_name, cmd_str);
		break;
	case BRDerr_BUFFER_TOO_SMALL:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_BUFFER_TOO_SMALL\n"), func_name, cmd_str);
		break;
	case BRDerr_WAIT_TIMEOUT:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_WAIT_TIMEOUT\n"), func_name, cmd_str);
		break;
	case BRDerr_DDC_INVALID_PRGFLCLK:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_DDC_INVALID_PRGFLCLK\n"), func_name, cmd_str);
		break;
	case BRDerr_DDC_PRGFILE_NOT:
		BRDC_sprintf(msg, _BRDC("%s - %s: BRDerr_DDC_PRGFILE_NOT\n"), func_name, cmd_str);
		break;
	default:
		BRDC_sprintf(msg, _BRDC("%s - %s: Unknown error, status = %8X\n"), func_name, cmd_str, real_status);
		break;
	}
	BRDC_printf(_BRDC("%s"), msg);
}

void bambp_impl::SetFrequencyNCO(int chan, double dFreqNCO)
{
	BRD_ValChan rValChan;

	rValChan.chan = m_vChannelNumbers[chan];
	rValChan.value = dFreqNCO;

	BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_SETFC, &rValChan);
}

void bambp_impl::SetInputSource(int nADC, int nDDC)
{
	BRD_EnVal rEnVal;

	rEnVal.enable = nADC;
	rEnVal.value = nDDC;

	BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_SETINPSRC, &rEnVal);
}

void bambp_impl::SetChanMask(int nChanMask)
{
	ULONG mask = 1;

	m_vChannelNumbers.clear();

	for(int iChan = 0; iChan < DDC_CHAN_NUM; iChan++)
	{
		if(nChanMask & mask)
			m_vChannelNumbers.push_back(iChan);
				
		mask <<= 1;
	}

	BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_SETCHANMASK, &nChanMask);
}

void bambp_impl::setup_mb(const size_t mb_i, const uhd::device_addr_t &dev_addr)
{
	int i;

	const fs_path mb_path = "/mboards/" + boost::lexical_cast<std::string>(mb_i);
	const fs_path rx_chans_path = mb_path + "/rx_channels";
	std::string product_name = "AMBPCX";

	_tree->create<std::string>(mb_path / "name").set(product_name);

	_tree->create<int>(rx_chans_path / "chan_mask")
		.add_coerced_subscriber(boost::bind(&bambp_impl::SetChanMask, this, _1))
		.set(0x1);

	for(i = 0; i < DDC_CHAN_NUM; i++)
	{
		_tree->create<double>(rx_chans_path / boost::lexical_cast<std::string>(i) / "frequency_nco")
			.add_coerced_subscriber(boost::bind(&bambp_impl::SetFrequencyNCO, this, i, _1));
		_tree->create<int>(rx_chans_path / boost::lexical_cast<std::string>(i) / "input_source")
			.set(0)
			.add_coerced_subscriber(boost::bind(&bambp_impl::SetInputSource, this, _1, i));
	}

	_tree->create<std::string>(mb_path / "clock_source" / "value")
		.set("");
	//	.add_coerced_subscriber(boost::bind(&bambp_impl::update_clock_source, this, _1));

	_tree->create<sensor_value_t>(mb_path / "sensors");

	_tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
	.set(subdev_spec_t("A:0"));
	//	.add_coerced_subscriber(boost::bind(&bambp_impl::update_rx_subdev_spec, this, _1));

	////_tree->create<std::string>(mb_path / "rx_dsps" / "0");

	_tree->create<double>(mb_path / "dboards/A/rx_frontends/0/freq/value").set(0);
	_tree->create<double>(mb_path / "rx_dsps/0/freq/value").set(0);
	_tree->create<double>(mb_path / "tick_rate");
	_tree->create<double>(mb_path / "dboards/A/rx_frontends/0/bandwidth/value");
	_tree->create<meta_range_t>(mb_path / "rx_dsps/0/freq/range");
	_tree->create<meta_range_t>(mb_path / "dboards/A/rx_frontends/0/freq/range");
	
	sensor_value_t val("lo_locked", true, "locked", "unlocked");
	_tree->create<sensor_value_t>(mb_path / "dboards/A/rx_frontends/0/sensors/lo_locked")
		.set(val);
}

rx_streamer::sptr bambp_impl::get_rx_stream(const stream_args_t &args)
{
	U32 nMask = 0;
	U32 i;

	for(i = 0; i < args.channels.size(); i++)
		nMask += 1 << m_vChannelNumbers[i];

	BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_SETCHANMASK, &nMask);

	rx_streamer::sptr sptr(new uhd::rx_streamer_is(args, m_hADC, m_hDDC));

	return sptr;
}

tx_streamer::sptr bambp_impl::get_tx_stream(const stream_args_t &args)
{
	tx_streamer::sptr sptr;

	return sptr;
}

bool bambp_impl::recv_async_msg(
	async_metadata_t &async_metadata, double timeout
)
{
	return 0;
}

void bambp_impl::update_clock_source(const std::string &source)
{

}

void bambp_impl::update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &spec)
{
}

void bambp_impl::update_rx_samp_rate(const double rate)
{

}