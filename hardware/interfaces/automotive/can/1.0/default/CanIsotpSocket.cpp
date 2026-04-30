/*
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

#include "CanIsotpSocket.h"

#include <android-base/logging.h>
#include <libnetdevice/can.h>
#include <libnetdevice/libnetdevice.h>
#include <linux/can.h>
#include <utils/SystemClock.h>

#include <chrono>

namespace android::hardware::automotive::can::V1_0::implementation {

using namespace std::chrono_literals;

/* How frequently the read thread checks whether the interface was asked to be down.
 *
 * Note: This does *not* affect read timing or bandwidth, just CPU load vs time to
 *       down the interface. */
static constexpr auto kReadPooling = 100ms;

std::shared_ptr<CanIsotpSocket> CanIsotpSocket::open(const std::string& ifname,
                            uint32_t rxid, uint32_t txid, ReadCallback rdcb, ErrorCallback errcb) {
    if (rxid == 0) rxid = ISOTP_RX_ID_DEFAULT;
    if (txid == 0) txid = ISOTP_TX_ID_DEFAULT;
    auto sock = netdevice::can::socketIsotp(ifname, rxid, txid, ISOTP_BLOCK_SIZE_DEFAULT, ISOTP_STMIN_DEFAULT);
    if (!sock.ok()) {
        LOG(ERROR) << "Can't open CAN socket on " << ifname << " for ISO-TP";
        return nullptr;
    }

    // Can't use std::make_unique due to private CanIsotpSocket constructor.
    return std::shared_ptr<CanIsotpSocket>(new CanIsotpSocket(std::move(sock), rxid, txid, rdcb, errcb));
}

CanIsotpSocket::CanIsotpSocket(base::unique_fd socket, uint32_t rxId, uint32_t txId, ReadCallback rdcb, ErrorCallback errcb)
    : mRxId(rxId),
      mTxId(txId),
      mReadCallback(rdcb),
      mErrorCallback(errcb),
      mSocket(std::move(socket)),
      mReaderThread(&CanIsotpSocket::readerThread, this) {}

CanIsotpSocket::~CanIsotpSocket() {
    mStopReaderThread = true;

    /* CanIsotpSocket can be brought down as a result of read failure, from the same thread,
     * so let's just detach and let it finish on its own. */
    if (mReaderThreadFinished) {
        mReaderThread.detach();
    } else {
        mReaderThread.join();
    }
}

bool CanIsotpSocket::send(const struct canisotp_frame& frame)
{
    if (frame.can_id != mTxId) {
        LOG(WARNING) << "CanIsotpSocket txId(" << mTxId << ") mismatch with frame txId(" << frame.can_id << ")";
    }

    const auto res = write(mSocket.get(), frame.data, frame.len);
    if (res < 0) {
        LOG(WARNING) << "CanIsotpSocket send failed";
        return false;
    }
    if (res != static_cast<long>(frame.len)) {
        LOG(WARNING) << "CanIsotpSocket only sent " << res << " bytes. expecting " << frame.len;
        return false;
    }
    return true;
}

static struct timeval toTimeval(std::chrono::microseconds t) {
    struct timeval tv;
    tv.tv_sec = t / 1s;
    tv.tv_usec = (t % 1s) / 1us;
    return tv;
}

static int selectRead(const base::unique_fd& fd, std::chrono::microseconds timeout) {
    auto timeouttv = toTimeval(timeout);
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd.get(), &readfds);
    return select(fd.get() + 1, &readfds, nullptr, nullptr, &timeouttv);
}

void CanIsotpSocket::readerThread() {
    LOG(VERBOSE) << "Reader thread started for ISO-TP";
    int errnoCopy = 0;
    std::vector<uint8_t> buffer(ISOTP_DATA_LEN);

    while (!mStopReaderThread) {
        /* The ideal would be to have a blocking read(3) call and interrupt it with shutdown(3).
         * This is unfortunately not supported for SocketCAN, so we need to rely on select(3). */
        const auto sel = selectRead(mSocket, kReadPooling);
        if (sel == 0) continue;  // timeout
        if (sel == -1) {
            PLOG(ERROR) << "Select failed";
            break;
        }

        const auto nbytes = read(mSocket.get(), buffer.data(), buffer.size());

        /* We could use SIOCGSTAMP to get a precise UNIX timestamp for a given packet, but what
         * we really need is a time since boot. There is no direct way to convert between these
         * clocks. We could implement a class to calculate the difference between the clocks
         * (querying both several times and picking the smallest difference); apply the difference
         * to a SIOCGSTAMP returned value; re-synchronize if the elapsed time is too much in the
         * past (indicating the UNIX timestamp might have been adjusted).
         *
         * Apart from the added complexity, it's possible the added calculations and system calls
         * would add so much time to the processing pipeline so the precision of the reported time
         * was buried under the subsystem latency. Let's just use a local time since boot here and
         * leave precise hardware timestamps for custom proprietary implementations (if needed). */
        const std::chrono::nanoseconds ts(elapsedRealtimeNano());

        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;

            errnoCopy = errno;
            PLOG(ERROR) << "Failed to read ISO-TP CANFD packet";
            break;
        }

        mReadCallback(mRxId, buffer.data(), nbytes, ts);
    }

    bool failed = !mStopReaderThread;
    auto errCb = mErrorCallback;
    mReaderThreadFinished = true;

    // Don't access any fields from here, see CanSocket::~CanSocket comment about detached thread
    if (failed) errCb(errnoCopy);

    LOG(VERBOSE) << "Reader thread stopped for ISO-TP";
}

}  // namespace android::hardware::automotive::can::V1_0::implementation