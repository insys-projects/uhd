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

#ifndef INCLUDED_BAMBP_IMPL_HPP
#define INCLUDED_BAMBP_IMPL_HPP

#include "../device3/device3_impl.hpp"
#include <uhd/device.hpp>
#include <uhd/stream_is.hpp>
#include <uhd/usrp/subdev_spec.hpp>

#include "brd.h"
#include "ctrlddc.h"

static const std::string AMBPCX_DEFAULT_CLOCK_SOURCE = "internal";

uhd::device_addrs_t bambp_find(const uhd::device_addr_t &hint_);

class bambp_impl: public uhd::device
{

	BRD_Handle	m_handle;

	ULONG		m_MemAsFifo;
	int			m_isChangeParams;

	char		m_iniFileName[256];

	char		m_AdcSrvName[32]; // с номером службы
	char		m_DdcSrvName[32]; // с номером службы

	char		m_pldFileName[MAX_PATH];
	int			m_isPldLoadAlways;

	ULONG		m_DmaOn;
	ULONG		m_Cycle;
	ULONG		m_MemOn;
	ULONG		m_IsWriteFile;
	ULONG		m_IsSysMem;
	ULONG		m_samplesOfChannel;
	ULONG		m_memorySamplesOfChannel;
	ULONG		m_DdcOn;

	BRD_Handle	m_hADC;
	BRD_Handle	m_hDDC;

	int			m_ver;

	ULONG		m_bBufSize;
	ULONG		m_bMemBufSize;

	ULONG		m_DirWriteFile;
	ULONG		m_FileBufSize;
	ULONG		m_DirMsTimeout;

	ULONG		m_MsTimeout;

	ULONG		m_DrqFlag;

	char		m_dirFileName[MAX_PATH];
	ULONG		m_flDirFileName;

	char		m_dataFileName[MAX_PATH];

	BRD_DdcSync m_ddc_sync;

public:

	bambp_impl(const uhd::device_addr_t &);
	~bambp_impl();
	void setup_mb(const size_t which, const uhd::device_addr_t &);

	virtual uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
	virtual uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);
	virtual bool recv_async_msg(uhd::async_metadata_t &async_metadata, double timeout);

private:
	// Запрет/разрешение изменения параметров
	void ChangeParams(int isChange);
	// Установка параметров
	void SetParams();

	void update_clock_source(const std::string &source);
	void update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &spec);
	void update_rx_samp_rate(const double rate);

	static void GetInifileString(const char *FileName, const char *SectionName, const char *ParamName, const char *defValue, char *strValue, int strSize);
	void ReadIniFile();
	S32	SetParamSrv();
	S32 AdcSettings(BRDCHAR *iniSectionName, BRDCHAR *iniFilePath);
	S32 DdcSettings(BRDCHAR *iniSectionName, BRDCHAR *iniFilePath);
	S32 SdramSettings(BRD_Handle hADC, ULONG &bBuf_size, ULONG ddc_on, ULONG mem_mode);

	void DisplayError(S32 status, const char *funcName, const BRDCHAR *cmd_str);

	void SetFrequencyNCO(int nChan, double dFreqNCO);
};

#endif // INCLUDED_BAMBP_IMPL_HPP
