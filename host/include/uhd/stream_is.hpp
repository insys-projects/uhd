#ifndef INCLUDED_UHD_STREAM_IS_HPP
#define INCLUDED_UHD_STREAM_IS_HPP

#include "brd.h"
#include "ctrlstrm.h"

#include <uhd/stream.hpp>
namespace uhd {
	class UHD_API rx_streamer_is: public rx_streamer {
		BRD_Handle	m_hADC;
		BRD_Handle	m_hDDC;
		BRDctrl_StreamCBufAlloc	m_buf_dscr;
		volatile U08 m_isAlloc;
		volatile U08 m_isStart;
		stream_args_t m_args;
		volatile U32 m_write_cnt;
		volatile U32 m_max_num_samps;
		volatile U32 m_last_num_samps;
		volatile U32 m_offset;

	public:
		rx_streamer_is(const stream_args_t &args, BRD_Handle hADC, BRD_Handle hDDC);
		
		virtual ~rx_streamer_is(void);
		
		virtual size_t get_num_channels(void) const;
		
		virtual size_t get_max_num_samps(void) const;
					
		virtual size_t recv(
			const buffs_type &buffs,
			const size_t nsamps_per_buff,
			rx_metadata_t &metadata,
			const double timeout = 0.1,
			const bool one_packet = false
		);

		virtual void issue_stream_cmd(const stream_cmd_t &stream_cmd);

	private:
		S32 PrepareStart();
		void Stop();
		S32 AllocBuf(const buffs_type &buffs, size_t nsamps_per_buff);
		U32	GetSampleSize();
		void ConvertData(void *pDst, void *pSrc, U32 nSize);
		void sc16tofc32(void *pDst, void *pSrc, U32 nBitsPerSample, U32 nSize);
		void copy_sc16(void *pDst, void *pSrc, U32 nBitsPerSample, U32 nSize);
	};

	class UHD_API tx_streamer_is: public tx_streamer {		
	public:
		tx_streamer_is(void);

		virtual ~tx_streamer_is(void);

		virtual size_t get_num_channels(void) const;

		virtual size_t get_max_num_samps(void) const;

		virtual size_t send(
			const buffs_type &buffs,
			const size_t nsamps_per_buff,
			const tx_metadata_t &metadata,
			const double timeout = 0.1
		);

		virtual bool recv_async_msg(
			async_metadata_t &async_metadata, double timeout = 0.1
		);
	};

} //namespace uhd

#endif /* INCLUDED_UHD_STREAM_IS_HPP */
