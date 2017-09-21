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

#include <string>

#include <uhd/utils/static.hpp>
#include <uhd/types/sensors.hpp>

using namespace std;
using namespace uhd;
using namespace uhd::usrp;

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

	BRDC_strcpy(m_DdcSrvName, _BRDC("DDC4X160"));

	m_handle = 0;

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
	if(ItemReal <= 10)
	{
		//for(U32 j = 0; j < ItemReal; j++)
		//{
		//	printf("Service %d: %s, Attr = %X.\n",
		//					j, srvList[j].name, srvList[j].attr);
		//}
		U32 iSrv;
		for(iSrv = 0; iSrv < ItemReal; iSrv++)
		{
			if(!BRDC_strcmp(srvList[iSrv].name, m_DdcSrvName))
			{
				err = SetParamSrv(m_handle, &srvList[iSrv]);
				if(!BRD_errcmp(err, BRDerr_OK))
					break;
			}
		}
	}
	else
		BRDC_printf(_BRDC("BRD_serviceList: Real Items = %d (> 10 - ERROR!!!)\n"), ItemReal);

	_tree = uhd::property_tree::make();

	UHD_MSG(status) << "AMBPCX initialization sequence..." << std::endl;
	_tree->create<std::string>("/name").set("AMBPCX Device");

	this->setup_mb(0, dev_addr);
}

void bambp_impl::setup_mb(const size_t mb_i, const uhd::device_addr_t &dev_addr)
{
	const fs_path mb_path = "/mboards/" + boost::lexical_cast<std::string>(mb_i);
	std::string product_name = "AMBPCX";

	_tree->create<std::string>(mb_path / "name").set(product_name);

	// Запрет изменения параметров
	ChangeParams(0);

	_tree->create<std::string>(mb_path / "clock_source" / "value")
		.set("")
		.add_coerced_subscriber(boost::bind(&bambp_impl::update_clock_source, this, _1));

	_tree->create<sensor_value_t>(mb_path / "sensors");

	_tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
		.set(subdev_spec_t("A:0"))
		.add_coerced_subscriber(boost::bind(&bambp_impl::update_rx_subdev_spec, this, _1));

	//_tree->create<std::string>(mb_path / "rx_dsps" / "0");

	_tree->create<double>(mb_path / "rx_dsps" / "0" / "rate" / "value")
		//.set_coercer(boost::bind(&rx_dsp_core_3000::set_host_rate, perif.ddc, _1))
		.add_coerced_subscriber(boost::bind(&bambp_impl::update_rx_samp_rate, this, _1))
		.set(0);

	// Разрешение изменения параметров
	ChangeParams(1);

	// Установка параметров
	SetParams();

	////////////////////////////////////////////////////////////////////
	// setup clock sources and properties
	////////////////////////////////////////////////////////////////////
	//_tree->create<std::string>(mb_path / "clock_source" / "value")
	//	.set(AMBPCX_DEFAULT_CLOCK_SOURCE)
	//	.add_coerced_subscriber(boost::bind(&bambp_impl::update_clock_source, this, boost::ref(mb), _1));

//	mboard_members_t &mb = _mb[mb_i];
//	mb.initialization_done = false;
//
//	std::vector<std::string> eth_addrs;
//	// Not choosing eth0 based on resource might cause user issues
//	std::string eth0_addr = dev_addr.has_key("resource") ? dev_addr["resource"] : dev_addr["addr"];
//	eth_addrs.push_back(eth0_addr);
//
//	mb.next_src_addr = 0;   //Host source address for blocks
//	if(dev_addr.has_key("second_addr")) {
//		std::string eth1_addr = dev_addr["second_addr"];
//
//		// Ensure we do not have duplicate addresses
//		if(eth1_addr != eth0_addr)
//			eth_addrs.push_back(eth1_addr);
//	}
//
//	// Initially store the first address provided to setup communication
//	// Once we read the eeprom, we use it to map IP to its interface
//	x300_eth_conn_t init;
//	init.addr = eth_addrs[0];
//	mb.eth_conns.push_back(init);
//
//	mb.xport_path = dev_addr.has_key("resource") ? "nirio" : "eth";
//	mb.if_pkt_is_big_endian = mb.xport_path != "nirio";
//
//	if(mb.xport_path == "nirio")
//	{
//		nirio_status status = 0;
//
//		std::string rpc_port_name(NIUSRPRIO_DEFAULT_RPC_PORT);
//		if(dev_addr.has_key("niusrpriorpc_port")) {
//			rpc_port_name = dev_addr["niusrpriorpc_port"];
//		}
//		UHD_MSG(status) << boost::format("Connecting to niusrpriorpc at localhost:%s...\n") % rpc_port_name;
//
//		//Instantiate the correct lvbitx object
//		nifpga_lvbitx::sptr lvbitx;
//		switch(get_mb_type_from_pcie(dev_addr["resource"], rpc_port_name)) {
//		case USRP_X300_MB:
//			lvbitx.reset(new x300_lvbitx(dev_addr["fpga"]));
//			break;
//		case USRP_X310_MB:
//			lvbitx.reset(new x310_lvbitx(dev_addr["fpga"]));
//			break;
//		default:
//			nirio_status_to_exception(status, "Motherboard detection error. Please ensure that you \
//                    have a valid USRP X3x0, NI USRP-294xR or NI USRP-295xR device and that all the device \
//                    drivers have loaded successfully.");
//		}
//		//Load the lvbitx onto the device
//		UHD_MSG(status) << boost::format("Using LVBITX bitfile %s...\n") % lvbitx->get_bitfile_path();
//		mb.rio_fpga_interface.reset(new niusrprio_session(dev_addr["resource"], rpc_port_name));
//		nirio_status_chain(mb.rio_fpga_interface->open(lvbitx, dev_addr.has_key("download-fpga")), status);
//		nirio_status_to_exception(status, "x300_impl: Could not initialize RIO session.");
//
//		//Tell the quirks object which FIFOs carry TX stream data
//		const uint32_t tx_data_fifos[2] = {X300_RADIO_DEST_PREFIX_TX, X300_RADIO_DEST_PREFIX_TX + 3};
//		mb.rio_fpga_interface->get_kernel_proxy()->get_rio_quirks().register_tx_streams(tx_data_fifos, 2);
//
//		_tree->create<size_t>(mb_path / "mtu/recv").set(X300_PCIE_RX_DATA_FRAME_SIZE);
//		_tree->create<size_t>(mb_path / "mtu/send").set(X300_PCIE_TX_DATA_FRAME_SIZE);
//		_tree->create<double>(mb_path / "link_max_rate").set(X300_MAX_RATE_PCIE);
//	}
//
//	BOOST_FOREACH(const std::string &key, dev_addr.keys())
//	{
//		if(key.find("recv") != std::string::npos) mb.recv_args[key] = dev_addr[key];
//		if(key.find("send") != std::string::npos) mb.send_args[key] = dev_addr[key];
//	}
//
//	if(mb.xport_path == "eth") {
//		/* This is an ETH connection. Figure out what the maximum supported frame
//		* size is for the transport in the up and down directions. The frame size
//		* depends on the host PIC's NIC's MTU settings. To determine the frame size,
//		* we test for support up to an expected "ceiling". If the user
//		* specified a frame size, we use that frame size as the ceiling. If no
//		* frame size was specified, we use the maximum UHD frame size.
//		*
//		* To optimize performance, the frame size should be greater than or equal
//		* to the frame size that UHD uses so that frames don't get split across
//		* multiple transmission units - this is why the limits passed into the
//		* 'determine_max_frame_size' function are actually frame sizes. */
//		frame_size_t req_max_frame_size;
//		req_max_frame_size.recv_frame_size = (mb.recv_args.has_key("recv_frame_size")) \
//			? boost::lexical_cast<size_t>(mb.recv_args["recv_frame_size"]) \
//			: X300_10GE_DATA_FRAME_MAX_SIZE;
//		req_max_frame_size.send_frame_size = (mb.send_args.has_key("send_frame_size")) \
//			? boost::lexical_cast<size_t>(mb.send_args["send_frame_size"]) \
//			: X300_10GE_DATA_FRAME_MAX_SIZE;
//
//#if defined UHD_PLATFORM_LINUX
//		const std::string mtu_tool("ip link");
//#elif defined UHD_PLATFORM_WIN32
//		const std::string mtu_tool("netsh");
//#else
//		const std::string mtu_tool("ifconfig");
//#endif
//
//		// Detect the frame size on the path to the USRP
//		try {
//			frame_size_t pri_frame_sizes = determine_max_frame_size(
//				eth_addrs.at(0), req_max_frame_size
//			);
//
//			_max_frame_sizes = pri_frame_sizes;
//			if(eth_addrs.size() > 1) {
//				frame_size_t sec_frame_sizes = determine_max_frame_size(
//					eth_addrs.at(1), req_max_frame_size
//				);
//
//				// Choose the minimum of the max frame sizes
//				// to ensure we don't exceed any one of the links' MTU
//				_max_frame_sizes.recv_frame_size = std::min(
//					pri_frame_sizes.recv_frame_size,
//					sec_frame_sizes.recv_frame_size
//				);
//
//				_max_frame_sizes.send_frame_size = std::min(
//					pri_frame_sizes.send_frame_size,
//					sec_frame_sizes.send_frame_size
//				);
//			}
//		}
//		catch(std::exception &e) {
//			UHD_MSG(error) << e.what() << std::endl;
//		}
//
//		if((mb.recv_args.has_key("recv_frame_size"))
//			&& (req_max_frame_size.recv_frame_size > _max_frame_sizes.recv_frame_size)) {
//			UHD_MSG(warning)
//				<< boost::format("You requested a receive frame size of (%lu) but your NIC's max frame size is (%lu).")
//				% req_max_frame_size.recv_frame_size
//				% _max_frame_sizes.recv_frame_size
//				<< std::endl
//				<< boost::format("Please verify your NIC's MTU setting using '%s' or set the recv_frame_size argument appropriately.")
//				% mtu_tool << std::endl
//				<< "UHD will use the auto-detected max frame size for this connection."
//				<< std::endl;
//		}
//
//		if((mb.recv_args.has_key("send_frame_size"))
//			&& (req_max_frame_size.send_frame_size > _max_frame_sizes.send_frame_size)) {
//			UHD_MSG(warning)
//				<< boost::format("You requested a send frame size of (%lu) but your NIC's max frame size is (%lu).")
//				% req_max_frame_size.send_frame_size
//				% _max_frame_sizes.send_frame_size
//				<< std::endl
//				<< boost::format("Please verify your NIC's MTU setting using '%s' or set the send_frame_size argument appropriately.")
//				% mtu_tool << std::endl
//				<< "UHD will use the auto-detected max frame size for this connection."
//				<< std::endl;
//		}
//
//		_tree->create<size_t>(mb_path / "mtu/recv").set(_max_frame_sizes.recv_frame_size);
//		_tree->create<size_t>(mb_path / "mtu/send").set(std::min(_max_frame_sizes.send_frame_size, X300_ETH_DATA_FRAME_MAX_TX_SIZE));
//		_tree->create<double>(mb_path / "link_max_rate").set(X300_MAX_RATE_10GIGE);
//	}
//
//	//create basic communication
//	UHD_MSG(status) << "Setup basic communication..." << std::endl;
//	if(mb.xport_path == "nirio") {
//		boost::mutex::scoped_lock(pcie_zpu_iface_registry_mutex);
//		if(get_pcie_zpu_iface_registry().has_key(mb.get_pri_eth().addr)) {
//			throw uhd::assertion_error("Someone else has a ZPU transport to the device open. Internal error!");
//		}
//		else {
//			mb.zpu_ctrl = x300_make_ctrl_iface_pcie(mb.rio_fpga_interface->get_kernel_proxy());
//			get_pcie_zpu_iface_registry()[mb.get_pri_eth().addr] = boost::weak_ptr<wb_iface>(mb.zpu_ctrl);
//		}
//	}
//	else {
//		mb.zpu_ctrl = x300_make_ctrl_iface_enet(udp_simple::make_connected(
//			mb.get_pri_eth().addr, BOOST_STRINGIZE(X300_FW_COMMS_UDP_PORT)));
//	}
//
//	// Claim device
//	if(not try_to_claim(mb.zpu_ctrl)) {
//		throw uhd::runtime_error("Failed to claim device");
//	}
//	mb.claimer_task = uhd::task::make(boost::bind(&x300_impl::claimer_loop, this, mb.zpu_ctrl));
//
//	//extract the FW path for the X300
//	//and live load fw over ethernet link
//	if(dev_addr.has_key("fw"))
//	{
//		const std::string x300_fw_image = find_image_path(
//			dev_addr.has_key("fw") ? dev_addr["fw"] : X300_FW_FILE_NAME
//		);
//		x300_load_fw(mb.zpu_ctrl, x300_fw_image);
//	}
//
//	//check compat numbers
//	//check fpga compat before fw compat because the fw is a subset of the fpga image
//	this->check_fpga_compat(mb_path, mb);
//	this->check_fw_compat(mb_path, mb.zpu_ctrl);
//
//	mb.fw_regmap = boost::make_shared<fw_regmap_t>();
//	mb.fw_regmap->initialize(*mb.zpu_ctrl.get(), true);
//
//	//store which FPGA image is loaded
//	mb.loaded_fpga_image = get_fpga_option(mb.zpu_ctrl);
//
//	//low speed perif access
//	mb.zpu_spi = spi_core_3000::make(mb.zpu_ctrl, SR_ADDR(SET0_BASE, ZPU_SR_SPI),
//		SR_ADDR(SET0_BASE, ZPU_RB_SPI));
//	mb.zpu_i2c = i2c_core_100_wb32::make(mb.zpu_ctrl, I2C1_BASE);
//	mb.zpu_i2c->set_clock_rate(X300_BUS_CLOCK_RATE / 2);
//
//	////////////////////////////////////////////////////////////////////
//	// print network routes mapping
//	////////////////////////////////////////////////////////////////////
//	/*
//	const uint32_t routes_addr = mb.zpu_ctrl->peek32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_ROUTE_MAP_ADDR));
//	const uint32_t routes_len = mb.zpu_ctrl->peek32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_ROUTE_MAP_LEN));
//	UHD_VAR(routes_len);
//	for (size_t i = 0; i < routes_len; i+=1)
//	{
//	const uint32_t node_addr = mb.zpu_ctrl->peek32(SR_ADDR(routes_addr, i*2+0));
//	const uint32_t nbor_addr = mb.zpu_ctrl->peek32(SR_ADDR(routes_addr, i*2+1));
//	if (node_addr != 0 and nbor_addr != 0)
//	{
//	UHD_MSG(status) << boost::format("%u: %s -> %s")
//	% i
//	% asio::ip::address_v4(node_addr).to_string()
//	% asio::ip::address_v4(nbor_addr).to_string()
//	<< std::endl;
//	}
//	}
//	*/
//
//	////////////////////////////////////////////////////////////////////
//	// setup the mboard eeprom
//	////////////////////////////////////////////////////////////////////
//	UHD_MSG(status) << "Loading values from EEPROM..." << std::endl;
//	x300_mb_eeprom_iface::sptr eeprom16 = x300_mb_eeprom_iface::make(mb.zpu_ctrl, mb.zpu_i2c);
//	if(dev_addr.has_key("blank_eeprom"))
//	{
//		UHD_MSG(warning) << "Obliterating the motherboard EEPROM..." << std::endl;
//		eeprom16->write_eeprom(0x50, 0, byte_vector_t(256, 0xff));
//	}
//	const mboard_eeprom_t mb_eeprom(*eeprom16, "X300");
//	_tree->create<mboard_eeprom_t>(mb_path / "eeprom")
//		.set(mb_eeprom)
//		.add_coerced_subscriber(boost::bind(&x300_impl::set_mb_eeprom, this, mb.zpu_i2c, _1));
//
//	bool recover_mb_eeprom = dev_addr.has_key("recover_mb_eeprom");
//	if(recover_mb_eeprom) {
//		UHD_MSG(warning) << "UHD is operating in EEPROM Recovery Mode which disables hardware version "
//			"checks.\nOperating in this mode may cause hardware damage and unstable "
//			"radio performance!" << std::endl;
//	}
//
//	////////////////////////////////////////////////////////////////////
//	// parse the product number
//	////////////////////////////////////////////////////////////////////
//	std::string product_name = "X300?";
//	switch(get_mb_type_from_eeprom(mb_eeprom)) {
//	case USRP_X300_MB:
//		product_name = "X300";
//		break;
//	case USRP_X310_MB:
//		product_name = "X310";
//		break;
//	default:
//		if(not recover_mb_eeprom)
//			throw uhd::runtime_error("Unrecognized product type.\n"
//				"Either the software does not support this device in which case please update your driver software to the latest version and retry OR\n"
//				"The product code in the EEPROM is corrupt and may require reprogramming.");
//	}
//	_tree->create<std::string>(mb_path / "name").set(product_name);
//	_tree->create<std::string>(mb_path / "codename").set("Yetti");
//
//	////////////////////////////////////////////////////////////////////
//	// determine routing based on address match
//	////////////////////////////////////////////////////////////////////
//	if(mb.xport_path != "nirio") {
//		// Discover ethernet interfaces
//		mb.discover_eth(mb_eeprom, eth_addrs);
//	}
//
//	////////////////////////////////////////////////////////////////////
//	// read hardware revision and compatibility number
//	////////////////////////////////////////////////////////////////////
//	mb.hw_rev = 0;
//	if(mb_eeprom.has_key("revision") and not mb_eeprom["revision"].empty()) {
//		try {
//			mb.hw_rev = boost::lexical_cast<size_t>(mb_eeprom["revision"]);
//		}
//		catch(...) {
//			if(not recover_mb_eeprom)
//				throw uhd::runtime_error("Revision in EEPROM is invalid! Please reprogram your EEPROM.");
//		}
//	}
//	else {
//		if(not recover_mb_eeprom)
//			throw uhd::runtime_error("No revision detected. MB EEPROM must be reprogrammed!");
//	}
//
//	size_t hw_rev_compat = 0;
//	if(mb.hw_rev >= 7) { //Revision compat was added with revision 7
//		if(mb_eeprom.has_key("revision_compat") and not mb_eeprom["revision_compat"].empty()) {
//			try {
//				hw_rev_compat = boost::lexical_cast<size_t>(mb_eeprom["revision_compat"]);
//			}
//			catch(...) {
//				if(not recover_mb_eeprom)
//					throw uhd::runtime_error("Revision compat in EEPROM is invalid! Please reprogram your EEPROM.");
//			}
//		}
//		else {
//			if(not recover_mb_eeprom)
//				throw uhd::runtime_error("No revision compat detected. MB EEPROM must be reprogrammed!");
//		}
//	}
//	else {
//		//For older HW just assume that revision_compat = revision
//		hw_rev_compat = mb.hw_rev;
//	}
//
//	if(hw_rev_compat > X300_REVISION_COMPAT) {
//		if(not recover_mb_eeprom)
//			throw uhd::runtime_error(str(boost::format(
//				"Hardware is too new for this software. Please upgrade to a driver that supports hardware revision %d.")
//				% mb.hw_rev));
//	}
//	else if(mb.hw_rev < X300_REVISION_MIN) { //Compare min against the revision (and not compat) to give us more leeway for partial support for a compat
//		if(not recover_mb_eeprom)
//			throw uhd::runtime_error(str(boost::format(
//				"Software is too new for this hardware. Please downgrade to a driver that supports hardware revision %d.")
//				% mb.hw_rev));
//	}
//
//	////////////////////////////////////////////////////////////////////
//	// create clock control objects
//	////////////////////////////////////////////////////////////////////
//	UHD_MSG(status) << "Setup RF frontend clocking..." << std::endl;
//
//	//Initialize clock control registers. NOTE: This does not configure the LMK yet.
//	mb.clock = x300_clock_ctrl::make(mb.zpu_spi,
//		1 /*slaveno*/,
//		mb.hw_rev,
//		dev_addr.cast<double>("master_clock_rate", X300_DEFAULT_TICK_RATE),
//		dev_addr.cast<double>("dboard_clock_rate", X300_DEFAULT_DBOARD_CLK_RATE),
//		dev_addr.cast<double>("system_ref_rate", X300_DEFAULT_SYSREF_RATE));
//
//	//Initialize clock source to use internal reference and generate
//	//a valid radio clock. This may change after configuration is done.
//	//This will configure the LMK and wait for lock
//	update_clock_source(mb, X300_DEFAULT_CLOCK_SOURCE);
//
//	////////////////////////////////////////////////////////////////////
//	// create clock properties
//	////////////////////////////////////////////////////////////////////
//	_tree->create<double>(mb_path / "master_clock_rate")
//		.set_publisher(boost::bind(&x300_clock_ctrl::get_master_clock_rate, mb.clock))
//		;
//
//	UHD_MSG(status) << "Radio 1x clock:" << (mb.clock->get_master_clock_rate() / 1e6)
//		<< std::endl;
//
//	////////////////////////////////////////////////////////////////////
//	// Create the GPSDO control
//	////////////////////////////////////////////////////////////////////
//	static const uint32_t dont_look_for_gpsdo = 0x1234abcdul;
//
//	//otherwise if not disabled, look for the internal GPSDO
//	if(mb.zpu_ctrl->peek32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_GPSDO_STATUS)) != dont_look_for_gpsdo)
//	{
//		UHD_MSG(status) << "Detecting internal GPSDO.... " << std::flush;
//		try
//		{
//			mb.gps = gps_ctrl::make(x300_make_uart_iface(mb.zpu_ctrl));
//		}
//		catch(std::exception &e)
//		{
//			UHD_MSG(error) << "An error occurred making GPSDO control: " << e.what() << std::endl;
//		}
//		if(mb.gps and mb.gps->gps_detected())
//		{
//			BOOST_FOREACH(const std::string &name, mb.gps->get_sensors())
//			{
//				_tree->create<sensor_value_t>(mb_path / "sensors" / name)
//					.set_publisher(boost::bind(&gps_ctrl::get_sensor, mb.gps, name));
//			}
//		}
//		else
//		{
//			mb.zpu_ctrl->poke32(SR_ADDR(X300_FW_SHMEM_BASE, X300_FW_SHMEM_GPSDO_STATUS), dont_look_for_gpsdo);
//		}
//	}
//
//	////////////////////////////////////////////////////////////////////
//	//clear router?
//	////////////////////////////////////////////////////////////////////
//	for(size_t i = 0; i < 512; i++) {
//		mb.zpu_ctrl->poke32(SR_ADDR(SETXB_BASE, i), 0);
//	}
//
//
//	////////////////////////////////////////////////////////////////////
//	// setup time sources and properties
//	////////////////////////////////////////////////////////////////////
//	_tree->create<std::string>(mb_path / "time_source" / "value")
//		.set("internal")
//		.add_coerced_subscriber(boost::bind(&x300_impl::update_time_source, this, boost::ref(mb), _1));
//	static const std::vector<std::string> time_sources = boost::assign::list_of("internal")("external")("gpsdo");
//	_tree->create<std::vector<std::string> >(mb_path / "time_source" / "options").set(time_sources);
//
//	//setup the time output, default to ON
//	_tree->create<bool>(mb_path / "time_source" / "output")
//		.add_coerced_subscriber(boost::bind(&x300_impl::set_time_source_out, this, boost::ref(mb), _1))
//		.set(true);
//
//	////////////////////////////////////////////////////////////////////
//	// setup clock sources and properties
//	////////////////////////////////////////////////////////////////////
//	_tree->create<std::string>(mb_path / "clock_source" / "value")
//		.set(X300_DEFAULT_CLOCK_SOURCE)
//		.add_coerced_subscriber(boost::bind(&x300_impl::update_clock_source, this, boost::ref(mb), _1));
//
//	static const std::vector<std::string> clock_source_options = boost::assign::list_of("internal")("external")("gpsdo");
//	_tree->create<std::vector<std::string> >(mb_path / "clock_source" / "options").set(clock_source_options);
//
//	//setup external reference options. default to 10 MHz input reference
//	_tree->create<std::string>(mb_path / "clock_source" / "external");
//	static const std::vector<double> external_freq_options = boost::assign::list_of(10e6)(30.72e6)(200e6);
//	_tree->create<std::vector<double> >(mb_path / "clock_source" / "external" / "freq" / "options")
//		.set(external_freq_options);
//	_tree->create<double>(mb_path / "clock_source" / "external" / "value")
//		.set(mb.clock->get_sysref_clock_rate());
//	// FIXME the external clock source settings need to be more robust
//
//	//setup the clock output, default to ON
//	_tree->create<bool>(mb_path / "clock_source" / "output")
//		.add_coerced_subscriber(boost::bind(&x300_clock_ctrl::set_ref_out, mb.clock, _1));
//
//	//initialize tick rate (must be done before setting time)
//	_tree->create<double>(mb_path / "tick_rate")
//		.add_coerced_subscriber(boost::bind(&device3_impl::update_tx_streamers, this, _1))
//		.add_coerced_subscriber(boost::bind(&device3_impl::update_rx_streamers, this, _1))
//		.set(mb.clock->get_master_clock_rate())
//		;
//
//	////////////////////////////////////////////////////////////////////
//	// and do the misc mboard sensors
//	////////////////////////////////////////////////////////////////////
//	_tree->create<sensor_value_t>(mb_path / "sensors" / "ref_locked")
//		.set_publisher(boost::bind(&x300_impl::get_ref_locked, this, mb));
//
//	//////////////// RFNOC /////////////////
//	const size_t n_rfnoc_blocks = mb.zpu_ctrl->peek32(SR_ADDR(SET0_BASE, ZPU_RB_NUM_CE));
//	enumerate_rfnoc_blocks(
//		mb_i,
//		n_rfnoc_blocks,
//		X300_XB_DST_PCI + 1, /* base port */
//		uhd::sid_t(X300_SRC_ADDR0, 0, X300_DST_ADDR + mb_i, 0),
//		dev_addr,
//		mb.if_pkt_is_big_endian ? ENDIANNESS_BIG : ENDIANNESS_LITTLE
//	);
//	//////////////// RFNOC /////////////////
//
//	// If we have a radio, we must configure its codec control:
//	const std::string radio_blockid_hint = str(boost::format("%d/Radio") % mb_i);
//	std::vector<rfnoc::block_id_t> radio_ids =
//		find_blocks<rfnoc::x300_radio_ctrl_impl>(radio_blockid_hint);
//	if(not radio_ids.empty()) {
//		if(radio_ids.size() > 2) {
//			UHD_MSG(warning) << "Too many Radio Blocks found. Using only the first two." << std::endl;
//			radio_ids.resize(2);
//		}
//
//		BOOST_FOREACH(const rfnoc::block_id_t &id, radio_ids) {
//			rfnoc::x300_radio_ctrl_impl::sptr radio(get_block_ctrl<rfnoc::x300_radio_ctrl_impl>(id));
//			mb.radios.push_back(radio);
//			radio->setup_radio(
//				mb.zpu_i2c,
//				mb.clock,
//				dev_addr.has_key("ignore-cal-file"),
//				dev_addr.has_key("self_cal_adc_delay")
//			);
//		}
//
//		////////////////////////////////////////////////////////////////////
//		// ADC test and cal
//		////////////////////////////////////////////////////////////////////
//		if(dev_addr.has_key("self_cal_adc_delay")) {
//			rfnoc::x300_radio_ctrl_impl::self_cal_adc_xfer_delay(
//				mb.radios, mb.clock,
//				boost::bind(&x300_impl::wait_for_clk_locked, this, mb, fw_regmap_t::clk_status_reg_t::LMK_LOCK, _1),
//				true /* Apply ADC delay */);
//		}
//		if(dev_addr.has_key("ext_adc_self_test")) {
//			rfnoc::x300_radio_ctrl_impl::extended_adc_test(
//				mb.radios,
//				dev_addr.cast<double>("ext_adc_self_test", 30));
//		}
//		else if(not dev_addr.has_key("recover_mb_eeprom")) {
//			for(size_t i = 0; i < mb.radios.size(); i++) {
//				mb.radios.at(i)->self_test_adc();
//			}
//		}
//
//		////////////////////////////////////////////////////////////////////
//		// Synchronize times (dboard initialization can desynchronize them)
//		////////////////////////////////////////////////////////////////////
//		if(radio_ids.size() == 2) {
//			this->sync_times(mb, mb.radios[0]->get_time_now());
//		}
//
//	}
//	else {
//		UHD_MSG(status) << "No Radio Block found. Assuming radio-less operation." << std::endl;
//	} /* end of radio block(s) initialization */
//
//	mb.initialization_done = true;
}

rx_streamer::sptr bambp_impl::get_rx_stream(const stream_args_t &args)
{
	rx_streamer::sptr sptr;

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

S32 bambp_impl::SetParamSrv(BRD_Handle handle, BRD_ServList* srv)
{
	S32		status;
	U32 mode = BRDcapt_EXCLUSIVE;
	BRD_Handle hADC = BRD_capture(handle, 0, &mode, srv->name, 10000);
	if(mode == BRDcapt_EXCLUSIVE) BRDC_printf(_BRDC("%s: Capture mode EXCLUSIVE\n"), srv->name);
	if(mode == BRDcapt_SPY)	BRDC_printf(_BRDC("%s: Capture mode SPY\n"), srv->name);

	if(hADC > 0)
	{
//		g_hSRV[g_nSrv++] = hADC;

		BRD_DdcCfg cfg;
		status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETCFG, &cfg);
		BRDC_printf(_BRDC("ADMFPGA = %0X\n"), cfg.AdmConst);
//		g_ver = cfg.AdmConst >> 12;

		//ULONG master = BRDims_SINGLE; // независимый (одиночный) режим
		//status = BRD_ctrl(hADC, 0, BRDctrl_DDC_SETMASTER, &master);
		//status = BRD_ctrl(hADC, 0, BRDctrl_ADC_SETMASTER, &master);

//		BRDCHAR iniFilePath[MAX_PATH];
//		BRDCHAR iniSectionName[MAX_PATH];
//#if defined(__IPC_WIN__) || defined(__IPC_LINUX__)
//		IPC_getCurrentDir(iniFilePath, sizeof(iniFilePath) / sizeof(BRDCHAR));
//#else
//		GetCurrentDirectory(sizeof(iniFilePath) / sizeof(BRDCHAR), iniFilePath);
//#endif
//		BRDC_strcat(iniFilePath, g_iniFileName);
//		BRDC_sprintf(iniSectionName, _BRDC("device%d_%s"), g_nSrv - 1, g_DdcSrvName);

		//BRDCHAR Buffer[128];
		//BRDCHAR* endptr;
		//GetPrivateProfileString(iniSectionName, "ChannelMask", "1", Buffer, sizeof(Buffer), iniFilePath);
		//ULONG chan_mask = strtol(Buffer, &endptr, 0);
//
		ULONG format = 0;
		ULONG sample_size, adc_sample_size;
		int numChan = 0, adc_numChan = 0;
//
//		if(g_DdcOn == 2)
//		{
//			status = AdcSettings(hADC, iniSectionName, iniFilePath); // установить параметры АЦП
//			ULONG adc_mask;
//			status = BRD_ctrl(hADC, 0, BRDctrl_ADC_GETCHANMASK, &adc_mask);
//			if(!adc_mask)
//				return status;
//			for(int iAdc = 0; iAdc < 4; iAdc++)
//				adc_numChan += (adc_mask >> iAdc) & 0x1;
//			status = BRD_ctrl(hADC, 0, BRDctrl_ADC_GETFORMAT, &format);
//			adc_sample_size = format ? format : sizeof(short);
//			g_bMemBufSize[g_nSrv - 1] = (g_memorySamplesOfChannel * adc_numChan) * adc_sample_size; // получить размер собираемых в память данных в байтах
//
//			status = DdcSettings(hADC, iniSectionName, iniFilePath); // установить параметры DDC
//			if(!BRD_errcmp(status, BRDerr_OK))
//				return status;
//
//			ULONG ddc_mask;
//			status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETCHANMASK, &ddc_mask);
//			if(!ddc_mask)
//				return status;
//			for(int iDdc = 0; iDdc < 16; iDdc++)
//				numChan += (ddc_mask >> iDdc) & 0x1;
//			status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETFORMAT, &format);
//			sample_size = format ? format : sizeof(short);
//			sample_size <<= 1;
//			g_bBufSize[g_nSrv - 1] = (g_samplesOfChannel * numChan) * sample_size; // получить размер собираемых данных в байтах
//
//																				   // проверяем наличие динамической памяти
//			BRD_SdramCfg SdramConfig;
//			ULONG PhysMemSize;
//			status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_GETCFG, &SdramConfig);
//			if(status < 0)
//			{
//				if(g_MemOn)
//				{
//					BRDC_printf(_BRDC("Get SDRAM Config: Error!!!\n"));
//					g_MemOn = 0;
//				}
//				PhysMemSize = 0;
//				return BRDerr_OK;
//			}
//			else
//				PhysMemSize = (1 << SdramConfig.RowAddrBits) *
//				(1 << SdramConfig.ColAddrBits) *
//				SdramConfig.ModuleBanks *
//				SdramConfig.ChipBanks *
//				SdramConfig.ModuleCnt * 2; // в 32-битных словах
//
//			if(PhysMemSize && g_MemOn)
//			{ // динамическая память присутствует на модуле
//				BRDC_printf(_BRDC("SDRAM Config: Memory size = %d MBytes\n"), (PhysMemSize / (1024 * 1024)) * 4);
//
//				status = SdramSettings(hADC, g_bMemBufSize[g_nSrv - 1], 0, g_MemOn); // установить параметры SDRAM
//				g_memorySamplesOfChannel = (g_bMemBufSize[g_nSrv - 1] / adc_sample_size) / adc_numChan;
//				if(g_MemOn == 2)
//				{
//					BRDC_printf(_BRDC("SDRAM as a FIFO mode!!!\n"));
//					g_MemOn = 0;
//					g_MemAsFifo = 1;
//				}
//				else
//					BRDC_printf(_BRDC("Memory samples of channel = %d\n"), g_memorySamplesOfChannel);
//			}
//			else
//			{ // освободить службу SDRAM (она могла быть захвачена командой BRDctrl_SDRAM_GETCFG, если та отработала без ошибки)
//				ULONG mem_size = 0;
//				status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_SETMEMSIZE, &mem_size);
//				if(g_MemOn)
//				{
//					BRDC_printf(_BRDC("No SDRAM on board!!!\n"));
//					g_MemOn = 0;
//				}
//			}
//			return status;
//		}
//
//		if(!g_DdcOn)
//		{
//			//status = BRD_ctrl(hADC, 0, BRDctrl_ADC_SETCHANMASK, &chan_mask);
//			//numChan = (chan_mask & 0x1) + ((chan_mask >> 1) & 0x1) + ((chan_mask >> 2) & 0x1) + ((chan_mask >> 3) & 0x1);
//
//			status = AdcSettings(hADC, iniSectionName, iniFilePath); // установить параметры АЦП
//
//			ULONG adc_mask;
//			status = BRD_ctrl(hADC, 0, BRDctrl_ADC_GETCHANMASK, &adc_mask);
//			if(!adc_mask)
//				return status;
//
//			for(int iAdc = 0; iAdc < 4; iAdc++)
//				numChan += (adc_mask >> iAdc) & 0x1;
//
//			status = BRD_ctrl(hADC, 0, BRDctrl_ADC_GETFORMAT, &format);
//			sample_size = format ? format : sizeof(short);
//		}
//		else
		{
			status = DdcSettings(hADC); // установить параметры DDC
			if(!BRD_errcmp(status, BRDerr_OK))
				return status;

			ULONG ddc_mask;
			status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETCHANMASK, &ddc_mask);
			if(!ddc_mask)
				return status;
			//			ULONG ddc_mask = 0xffff; // включаем все DDC
			//ULONG ddc_mask = 0x1; // включаем все 4 канала 1-ой микросхемы DDC
			//status = BRD_ctrl(hADC, 0, BRDctrl_DDC_SETCHANMASK, &ddc_mask);
			//if(BRD_errcmp(status, BRDerr_OK))
			//	printf("BRDctrl_DDC_SETCHANMASK: ddc_mask = %0X\n", ddc_mask);
			//else
			//	DisplayError(status, "BRDctrl_DDC_SETCHANMASK");

			for(int iDdc = 0; iDdc < 16; iDdc++)
				numChan += (ddc_mask >> iDdc) & 0x1;

			status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETFORMAT, &format);
			sample_size = format ? format : sizeof(short);
			sample_size <<= 1;
		}

		m_bBufSize = (m_samplesOfChannel * numChan) * sample_size; // получить размер собираемых данных в байтах
																			   //		printf("Samples of channel = %d\n", g_samplesOfChannel);

		m_bMemBufSize = (m_memorySamplesOfChannel * numChan) * sample_size; // получить размер собираемых в память данных в байтах

																						//ULONG sample_size = format ? format : sizeof(short);
																						//ULONG bBufSize = (g_samplesOfChannel * numChan) * sample_size; // получить размер собираемых данных в байтах

																						// проверяем наличие динамической памяти
		BRD_SdramCfg SdramConfig;
		ULONG PhysMemSize;
		status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_GETCFG, &SdramConfig);
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

			status = SdramSettings(hADC, m_bMemBufSize, m_DdcOn, m_MemOn); // установить параметры SDRAM
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
			status = BRD_ctrl(hADC, 0, BRDctrl_SDRAM_SETMEMSIZE, &mem_size);
			status = BRDerr_OK;
		}
		//		status = BRD_release(hADC, 0);
	}

	return status;
}

S32 bambp_impl::DdcSettings(BRD_Handle hADC)
{
	S32		status = 0;
//
//	//ULONG master = BRDims_SINGLE; // независимый (одиночный) режим
//	//status = BRD_ctrl(hADC, 0, BRDctrl_DDC_SETMASTER, &master);
//	//status = BRD_ctrl(hADC, 0, BRDctrl_ADC_SETMASTER, &master);
//
//	// задать параметры из файла
//	//TCHAR iniFilePath[MAX_PATH];
//	//TCHAR iniSectionName[MAX_PATH];
//	//GetCurrentDirectory(sizeof(iniFilePath)/sizeof(TCHAR), iniFilePath);
//	//lstrcat(iniFilePath, iniFileName);
//	//sprintf_s(iniSectionName, "device%d_%s%d", idx, srvName, 0);
//
//	BRD_IniFile ini_file;
//	BRDC_strcpy(ini_file.fileName, iniFilePath);
//	//	lstrcpy(ini_file.sectionName, "device0_ddc4x160");
//	BRDC_strcpy(ini_file.sectionName, iniSectionName);
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_READINIFILE, &ini_file);
//	if(BRD_errcmp(status, BRDerr_OK))
//		printf("BRDctrl_DDC_READINIFILE: OK\n");
//	else
//	{
//		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_READINIFILE"));
//		return status;
//	}
//
//	// получить (и задать) источник и значение тактовой частоты можно отдельной функцией
//	BRD_ClkMode clk_mode;
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETCLKMODE, &clk_mode);
//	if(BRD_errcmp(status, BRDerr_OK))
//		printf("BRDctrl_DDC_GETCLKMODE: source = %d, value = %f\n", clk_mode.src, clk_mode.value);
//	else
//		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETCLKMODE"));
//
//	BRD_StartMode start;
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETSTARTMODE, &start);
//	if(BRD_errcmp(status, BRDerr_OK))
//		printf("BRDctrl_DDC_GETSTARTMODE: start source = %d, restart = %d\n", start.startSrc, start.reStartMode);
//	else
//		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETSTARTMODE"));
//
//	if(start.startSrc == BRDsts_CMP1)
//	{	// старт от компаратора 1 (от есть от сигнала с разъема SDX)
//		TCHAR Buffer[128];
//		BRD_CmpSC cmp_sc;
//		cmp_sc.src = BRDcmps_EXTSTCLK;
//		cmp_sc.thr[0] = 0.0;
//#if defined(__IPC_WIN__) || defined(__IPC_LINUX__)
//		IPC_getPrivateProfileString(iniSectionName, _BRDC("ComparatorThresholdSDX"), _BRDC("0.0"), Buffer, sizeof(Buffer), iniFilePath);
//#else
//		GetPrivateProfileString(iniSectionName, _BRDC("ComparatorThresholdSDX"), _BRDC("0.0"), Buffer, sizeof(Buffer), iniFilePath);
//#endif
//		cmp_sc.thr[1] = BRDC_atof(Buffer);//0.0;
//
//										  // задать источникик и пороги для компараторов
//		status = BRD_ctrl(hADC, 0, BRDctrl_CMPSC_SET, &cmp_sc);
//		if(BRD_errcmp(status, BRDerr_OK))
//			printf("BRDctrl_CMPSC_SET: comparator source = %d, threshold = %.2f\n", cmp_sc.src, cmp_sc.thr[1]);
//		else
//			DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_CMPSC_SET"));
//	}
//	//// для заказа с синтезатором с VCO 400MHz
//	//if(start.startSrc == BRDsts_CMP0 || start.startSrc == BRDsts_CMP1)
//	//{	// старт от компаратора 0 (сигнал с разъема ExtSt) или 1 (сигнал с разъема SDX)
//	//	TCHAR Buffer[128];
//	//	BRD_CmpSC cmp_sc;
//	//	cmp_sc.src = BRDcmps_EXTSTCLK;
//	//	GetPrivateProfileString(iniSectionName, "ComparatorThresholdExtSt", "0.0", Buffer, sizeof(Buffer), iniFilePath);
//	//	cmp_sc.thr[0] = atof(Buffer);//0.0;
//	//	GetPrivateProfileString(iniSectionName, "ComparatorThresholdSDX", "0.0", Buffer, sizeof(Buffer), iniFilePath);
//	//	cmp_sc.thr[1] = atof(Buffer);//0.0;
//	//	
//	//	// задать источник и пороги для компараторов
//	//	status = BRD_ctrl(hADC, 0, BRDctrl_CMPSC_SET, &cmp_sc);
//	//	if(BRD_errcmp(status, BRDerr_OK))
//	//		printf("BRDctrl_CMPSC_SET: comparator source = %d, thresholdExtSt = %.2f, thresholdSDX = %.2f\n",
//	//													cmp_sc.src, cmp_sc.thr[0], cmp_sc.thr[1]);
//	//	else
//	//		DisplayError(status, "BRDctrl_CMPSC_SET");
//	//}
//
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETDDCSYNC, &g_ddc_sync);
//	if(BRD_errcmp(status, BRDerr_OK))
//		printf("BRDctrl_DDC_GETDDCSYNC: mode = %d\n", g_ddc_sync.mode);
//	else
//		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETDDCSYNC"));
//
//	ULONG ddc_mask;
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETCHANMASK, &ddc_mask);
//	if(BRD_errcmp(status, BRDerr_OK))
//		printf("BRDctrl_DDC_GETCHANMASK: ddc_mask = %0X\n", ddc_mask);
//	else
//		DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_DDC_GETCHANMASK"));
//
//	/*
//	if(g_ver != 5)
//	{
//	ULONG length = FRAME_LENGTH;
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_SETFRAME, &length);
//	}
//	else
//	{
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETTITLEMODE, &g_title_mode);
//	if(BRD_errcmp(status, BRDerr_OK))
//	printf("BRDctrl_DDC_GETTITLEMODE: enable = %d, size = %d\n", g_title_mode.enable, g_title_mode.value);
//	else
//	DisplayError(status, "BRDctrl_DDC_GETTITLEMODE");
//	if(g_title_mode.value > 8)
//	{
//	BRD_PioDir pio_dir;
//	pio_dir.lbDir = BRDpio_READ;
//	pio_dir.hbDir = BRDpio_READ;
//	status = BRD_ctrl(g_hPio, 0, BRDctrl_PIO_SETDIR, &pio_dir);
//	if(BRD_errcmp(status, BRDerr_OK))
//	printf("BRDctrl_PIO_SETDIR: pio_dir.lbDir = %d, pio_dir.hbDir = %d\n", pio_dir.lbDir, pio_dir.hbDir);
//	else
//	DisplayError(status, "BRDctrl_PIO_SETDIR");
//	}
//	}*/
//	/*	g_fc.chan = 0;
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETFC, &g_fc);
//	printf("BRDctrl_DDC_SETFC: channel = %d, FC = %.2f\n", g_fc.chan, g_fc.value);
//	g_fc.chan = 1;
//	status = BRD_ctrl(hADC, 0, BRDctrl_DDC_GETFC, &g_fc);
//	printf("BRDctrl_DDC_SETFC: channel = %d, FC = %.2f\n", g_fc.chan, g_fc.value);*/
	return status;
}

// установить параметры SDRAM
S32 bambp_impl::SdramSettings(BRD_Handle hADC, ULONG& bBuf_size, ULONG ddc_on, ULONG mem_mode)
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
			printf("BRDctrl_SDRAM_SETMEMSIZE: SDRAM buffer size = %d bytes\n", bBuf_size);
		else
			DisplayError(status, __FUNCTION__, _BRDC("BRDctrl_SDRAM_SETMEMSIZE"));
	}
	return status;
}

void bambp_impl::DisplayError(S32 status, const char* funcName, const BRDCHAR* cmd_str)
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

// Запрет/разрешение изменения параметров
void bambp_impl::ChangeParams(int isChange)
{
	m_isChangeParams = isChange;
}

// Установка параметров
void bambp_impl::SetParams()
{

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