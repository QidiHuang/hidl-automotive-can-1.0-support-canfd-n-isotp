/*
 * Copyright (C) 2019 The Android Open Source Project
 * Copyright (C) 2026 Qidi Huang (huang_qi_di@hotmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <libnetdevice/can.h>

#include "common.h"

#include <android-base/logging.h>
#include <android-base/unique_fd.h>
#include <libnl++/MessageFactory.h>
#include <libnl++/Socket.h>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <linux/can/isotp.h>
#include <linux/rtnetlink.h>

// Manually define missing ISO-TP FD constants if not present in UAPI
// #ifndef CAN_ISOTP_FD_FRAME
// #define CAN_ISOTP_FD_FRAME 0x01
// #endif
//
// #ifndef CAN_ISOTP_RECV_FC
// #define CAN_ISOTP_RECV_FC 0x02
// #endif

namespace android::netdevice::can {

static constexpr can_err_mask_t kErrMask = CAN_ERR_MASK;

base::unique_fd socket(const std::string& ifname) {
    sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = nametoindex(ifname);
    if (addr.can_ifindex == 0) {
        LOG(ERROR) << "Interface " << ifname << " doesn't exists";
        return {};
    }

    base::unique_fd sock(::socket(PF_CAN, SOCK_RAW, CAN_RAW));
    if (!sock.ok()) {
        LOG(ERROR) << "Failed to create CAN socket";
        return {};
    }

    if (setsockopt(sock.get(), SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &kErrMask, sizeof(kErrMask)) < 0) {
        PLOG(ERROR) << "Can't receive error frames, CAN setsockpt failed";
        return {};
    }

    int enableCanFd = 1;
    if (setsockopt(sock.get(), SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enableCanFd, sizeof(enableCanFd)) < 0) {
        PLOG(ERROR) << "Failed to enable CAN FD support";
        return {};
    }

    if (0 != fcntl(sock.get(), F_SETFL, O_RDWR | O_NONBLOCK)) {
        LOG(ERROR) << "Couldn't put CAN socket in non-blocking mode";
        return {};
    }

    if (0 != bind(sock.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr))) {
        LOG(ERROR) << "Can't bind to CAN interface " << ifname;
        return {};
    }

    return sock;
}

bool setBitrate(std::string ifname, uint32_t bitrate, uint32_t bitsamplepoint, uint32_t dbitrate, uint32_t dbitsamplepoint) {
    can_bittiming bt = {};
    bt.bitrate = bitrate;
    if (bitrate == 0) {
        bt.bitrate = 500000;  // default 500KHz
    }
    bt.sample_point = bitsamplepoint;
    if (bitsamplepoint == 0) {
        bt.sample_point = 800;  // default 80%
    }

    can_bittiming dbt = {};
    dbt.bitrate = dbitrate;
    if (dbitrate == 0) {
        dbt.bitrate = 2000000;  // default 2MHz
    }
    dbt.sample_point = dbitsamplepoint;
    if (dbitsamplepoint == 0) {
        dbt.sample_point = 800;  // default 80%
    }

    can_ctrlmode cm = {};
    cm.mask = CAN_CTRLMODE_FD;
    cm.flags = CAN_CTRLMODE_FD;

    nl::MessageFactory<ifinfomsg> req(RTM_NEWLINK, NLM_F_REQUEST | NLM_F_ACK);

    req->ifi_index = nametoindex(ifname);
    if (req->ifi_index == 0) {
        LOG(ERROR) << "Can't find interface " << ifname;
        return false;
    }

    {
        auto linkinfo = req.addNested(IFLA_LINKINFO);
        req.add(IFLA_INFO_KIND, "can");
        {
            auto infodata = req.addNested(IFLA_INFO_DATA);
            /* For CAN FD, it would require to add IFLA_CAN_DATA_BITTIMING
             * and IFLA_CAN_CTRLMODE as well. */
            req.add(IFLA_CAN_BITTIMING, bt);
            req.add(IFLA_CAN_DATA_BITTIMING, dbt);
            req.add(IFLA_CAN_CTRLMODE, cm);
            req.add(IFLA_CAN_RESTART_MS, 100);  // restart bus in 100ms on BUS-OFF error
        }
    }
    LOG(INFO) << "try setting CAN bitrate(" << bt.bitrate << "), bitSample(" << bt.sample_point << "), dbitrate(" << dbt.bitrate << "), dbitSample(" << dbt.sample_point << ") for " << ifname;

    nl::Socket sock(NETLINK_ROUTE);
    return sock.send(req) && sock.receiveAck(req);
}

//////////////////// ISO-TP ///////////////////////

base::unique_fd socketIsotp(const std::string& ifname, uint32_t rxid, uint32_t txid, uint32_t blocksize, uint32_t stmin)
{
    /**
     * refer to isotprecv for setting socket opts.
     */
    sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_addr.tp.rx_id = rxid;
    addr.can_addr.tp.tx_id = txid;
    addr.can_ifindex = nametoindex(ifname);
    if (addr.can_ifindex == 0) {
        LOG(ERROR) << "Interface " << ifname << " doesn't exists";
        return {};
    }

    base::unique_fd sock(::socket(PF_CAN, SOCK_DGRAM, CAN_ISOTP));
    if (!sock.ok()) {
        LOG(ERROR) << "Failed to create ISO-TP socket";
        return {};
    }

    struct can_isotp_options opts = {};
    //opts.flags |= CAN_ISOTP_FD_FRAME;
    opts.flags |= (CAN_ISOTP_TX_PADDING | CAN_ISOTP_RX_PADDING);
    opts.txpad_content = 0x55;
    opts.rxpad_content = 0x55;

    if (setsockopt(sock.get(), SOL_CAN_ISOTP, CAN_ISOTP_OPTS, &opts, sizeof(opts)) < 0) {
        PLOG(ERROR) << "Failed to set ISO-TP options";
        return {};
    }

    struct can_isotp_fc_options fc_opts = {};
    fc_opts.bs = blocksize;
    fc_opts.stmin = stmin;
    //fc_opts.wftmax = 10;  // ms
    setsockopt(sock.get(), SOL_CAN_ISOTP, CAN_ISOTP_RECV_FC, &fc_opts, sizeof(fc_opts));

    // Mandatory for FD: sets the maximum frame length for the LL (Link Layer)
    // For CAN FD, this is usually 64.
    struct can_isotp_ll_options ll_opts = {};
    ll_opts.mtu = CANFD_MTU;
    ll_opts.tx_dl = 64;
    //ll_opts.tx_flags = (CANFD_BRS | CANFD_FDF);
    ll_opts.tx_flags = CANFD_BRS;
    setsockopt(sock.get(), SOL_CAN_ISOTP, CAN_ISOTP_LL_OPTS, &ll_opts, sizeof(ll_opts));

    if (0 != fcntl(sock.get(), F_SETFL, O_RDWR | O_NONBLOCK)) {
        LOG(ERROR) << "Couldn't put CAN socket in non-blocking mode";
        return {};
    }

    if (0 != bind(sock.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr))) {
        LOG(ERROR) << "Can't bind ISO-TP socket";
        return {};
    }

    return sock;
}


}  // namespace android::netdevice::can
