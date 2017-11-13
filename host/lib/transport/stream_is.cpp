#include <uhd/stream_is.hpp>

#include "ctrladc.h"
#include "ctrlddc.h"

using namespace uhd;

rx_streamer_is::rx_streamer_is(const stream_args_t &args, BRD_Handle hADC, BRD_Handle hDDC)
{
	m_hADC = hADC;
	m_hDDC = hDDC;
	
	m_args = args;

	m_isAlloc = 0;
	m_isStart = 0;
}

rx_streamer_is::~rx_streamer_is(void)
{
}

size_t rx_streamer_is::get_num_channels(void) const
{
	return 1;
}
size_t rx_streamer_is::get_max_num_samps(void) const
{
	return 1;
}

size_t rx_streamer_is::recv(
	const buffs_type &buffs,
	const size_t nsamps_per_buff,
	rx_metadata_t &metadata,
	const double timeout,
	const bool one_packet
)
{
	S32		status;
	ULONG	Status = 0;
	ULONG err = 0;

	if(m_isStart == 0)
	{
		metadata.error_code = uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
		return 0;
	}

	if(!m_isAlloc)
	{	// Выделяем память и даем старт
		m_isAlloc = 1;

		AllocBuf(buffs, nsamps_per_buff);
	}

	// для проверки флагов переполнения разрядной сетки АЦП их надо сбрасывать перед каждым стартом
	ULONG clr = 0xF;
	//BRD_ValChan fc_val;

	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_CLRBITSOVERFLOW, &clr);

	ULONG flag = BRDstrm_DRQ_HALF;//BRDstrm_DRQ_HALF; // рекомендуется флаг - FIFO наполовину заполнено
						   //ULONG flag = BRDstrm_DRQ_ALMOST;
	ULONG tetrad;
	BRDctrl_StreamCBufStart start_pars;
	start_pars.isCycle = 0; // без зацикливания 

	// установить источник для работы стрима
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETSRCSTREAM, &tetrad); // стрим будет работать с DDC
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_SETSRC, &tetrad);
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_SETDRQ, &flag);
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_FIFORESET, NULL); // сброс FIFO АЦП
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_RESETFIFO, NULL);
	
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_CBUF_START, &start_pars); // старт ПДП							

	ULONG Enable = 3;

	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_ENABLE, &Enable); // разрешение работы DDC

	Enable = 1;

	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_FIFORESET, NULL); // Сброс ФИФО
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_ENABLE, &Enable); // разрешение работы DDC

	Enable = 2;
	ULONG msTimeout = 5000;
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_CBUF_WAITBUF, &msTimeout);
	
	if(BRD_errcmp(status, BRDerr_WAIT_TIMEOUT))
	{
		// если вышли по тайм-ауту, то остановимся
		status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_FIFOSTATUS, &Status);
		BRDC_printf(_BRDC("DDC FIFO Status = 0x%04X\n"), Status);
		status = BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_CBUF_STOP, NULL);
		//DisplayError(status, __FUNCTION__, _BRDC("TIME-OUT"));
		status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_ENABLE, &Enable); // запрет работы DDC
		err++;
	}
	else
	{
		status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_ENABLE, &Enable); // запрет работы DDC
	}
	
	Enable = 0;
	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_ENABLE, &Enable); // запрет работы ADC
	
	if(!err)
	{
		status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_FIFOSTATUS, &Status);

		// проверка флагов переполнения разрядной сетки АЦП
		status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_ISBITSOVERFLOW, &Status);
			
		//if(Status)
		//	BRDC_printf(_BRDC("ADC Bits OVERFLOW %X  "), Status);
	}

	ConvertData(buffs[0], m_buf_dscr.ppBlk[0], nsamps_per_buff * 2);

	//memcpy(buffs[0], m_buf_dscr.ppBlk[0], nsamps_per_buff * 4);

	return nsamps_per_buff;
}

void rx_streamer_is::issue_stream_cmd(const stream_cmd_t &stream_cmd)
{
	switch(stream_cmd.stream_mode)
	{
		case uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
			m_isStart = 1;
			PrepareStart();
			break;
		case uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS:
			Stop();
			break;
		case uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE:
			break;
	}
}

S32 rx_streamer_is::PrepareStart()
{
	S32	status;

	status = BRD_ctrl(m_hADC, 0, BRDctrl_ADC_PREPARESTART, NULL);
	
	if(status < 0)
		if(!(BRD_errcmp(status, BRDerr_CMD_UNSUPPORTED)
			|| BRD_errcmp(status, BRDerr_INSUFFICIENT_SERVICES)))
		{
			BRDC_printf(_BRDC("Prepare ADC Start: Error = 0x%X!!!\n"), status);
			return status;
		}

	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_PREPARESTART, NULL);
	
	if(status < 0)
		if(!(BRD_errcmp(status, BRDerr_CMD_UNSUPPORTED)
			|| BRD_errcmp(status, BRDerr_INSUFFICIENT_SERVICES)))
		{
			BRDC_printf(_BRDC("Prepare DDC Start: Error = 0x%X!!!\n"), status);
			return status;
		}

	return BRDerr_OK;
}

void rx_streamer_is::Stop()
{
	m_isStart = 0;

	if(m_isAlloc == 0)
		return;

	m_isAlloc = 0;
	BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_CBUF_FREE, 0);
}

S32 rx_streamer_is::AllocBuf(const buffs_type &buffs, size_t nBufSize)
{
	S32		status;
	void*	pBuffer = NULL;
	U32		sample_size = GetSampleSize();

	m_buf_dscr.dir = BRDstrm_DIR_IN;
	m_buf_dscr.isCont = 1; // 0 - буфер размещается в пользовательской памяти ПК, 1 - в системной
	m_buf_dscr.blkNum = 1;
	m_buf_dscr.ppBlk = new PVOID[m_buf_dscr.blkNum];
	m_buf_dscr.blkSize = nBufSize * sample_size * 2 * m_args.channels.size();
	m_buf_dscr.blkSize = m_buf_dscr.blkSize < 0x200000 ? 0x200000 : m_buf_dscr.blkSize;
	
	status = BRD_ctrl(m_hDDC, 0, BRDctrl_STREAM_CBUF_ALLOC, &m_buf_dscr);
	
	if(!BRD_errcmp(status, BRDerr_OK))
		printf("Error alloc memory!\n");
	//	DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_STREAM_CBUF_ALLOC"));

	//*pBufSize = m_buf_dscr.blkSize;

	return status;
}

U32	rx_streamer_is::GetSampleSize()
{
	S32		status;
	U32		sample_size;
	ULONG	format = 0;

	status = BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETFORMAT, &format);
	sample_size = format ? format : 2;

	return sample_size;
}

void rx_streamer_is::ConvertData(void *pDst, void *pSrc, U32 nSize)
{
	volatile U32 sample_size = GetSampleSize();
	
	BRD_AdcCfg adc_cfg;
	BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETCFG, &adc_cfg);

	ULONG format = 0;
	BRD_ctrl(m_hDDC, 0, BRDctrl_DDC_GETFORMAT, &format);

	format = format ? format : 2;

	int nBitsPerSample = (format != 1) ? adc_cfg.Bits : 8;
	nBitsPerSample = (format == 2) ? 16 : 24;

	if((sample_size == 2) && (m_args.cpu_format == "fc32"))
		sc16tofc32(pDst, pSrc, nBitsPerSample, nSize);
	else if((sample_size == 2) && (m_args.cpu_format == "sc16"))
		copy_sc16(pDst, pSrc, nBitsPerSample, nSize);

}

void rx_streamer_is::sc16tofc32(void *pDst, void *pSrc, U32 nBitsPerSample, U32 nSize)
{
	U32    i;
	S16    *psc16 = (S16*)pSrc;
	S16	   val;
	REAL32 *pfc32 = (REAL32*)pDst;

	BRD_AdcCfg adc_cfg;
	BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETCFG, &adc_cfg);

	for(i = 0; i < nSize; i++)
	{
		val = psc16[i];
		val >>= (16 - nBitsPerSample);
		pfc32[i] = val;
		pfc32[i] /= (1 << ((U16)nBitsPerSample - 1)) - 1;
		pfc32[i] *= adc_cfg.InpRange / 1000.;
	}
}

void rx_streamer_is::copy_sc16(void *pDst, void *pSrc, U32 nBitsPerSample, U32 nSize)
{
	volatile U32    i;
	U16    *src16 = (U16*)pSrc;
	U16    *dst16 = (U16*)pDst;
	U16	   val;

	BRD_AdcCfg adc_cfg;
	BRD_ctrl(m_hADC, 0, BRDctrl_ADC_GETCFG, &adc_cfg);

	for(i = 0; i < nSize; i++)
	{
		val = src16[i];
		val >>= (16 - nBitsPerSample);
		dst16[i] = val;
	}
}