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

#include "CanSocket.h"

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

std::unique_ptr<CanSocket> CanSocket::open(const std::string& ifname, ReadCallback rdcb,
                                           ErrorCallback errcb) {
    auto sock = netdevice::can::socket(ifname);
    if (!sock.ok()) {
        LOG(ERROR) << "Can't open CAN socket on " << ifname;
        return nullptr;
    }

    // Can't use std::make_unique due to private CanSocket constructor.
    return std::unique_ptr<CanSocket>(new CanSocket(std::move(sock), rdcb, errcb));
}

CanSocket::CanSocket(base::unique_fd socket, ReadCallback rdcb, ErrorCallback errcb)
    : mReadCallback(rdcb),
      mErrorCallback(errcb),
      mSocket(std::move(socket)),
      mReaderThread(&CanSocket::readerThread, this) {}

CanSocket::~CanSocket() {
    mStopReaderThread = true;

    /* CanSocket can be brought down as a result of read failure, from the same thread,
     * so let's just detach and let it finish on its own. */
    if (mReaderThreadFinished) {
        mReaderThread.detach();
    } else {
        mReaderThread.join();
    }
}

bool CanSocket::send(const struct canfd_frame& frame) {
    long mtu = (frame.len > 8)? CANFD_MTU: CAN_MTU;
    const auto res = write(mSocket.get(), &frame, mtu);
    if (res < 0) {
        PLOG(DEBUG) << "CanSocket send failed";
        return false;
    }
    if (res != mtu) {
        LOG(DEBUG) << "CanSocket sent wrong number of bytes: " << res;
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

#define CANFD_MTU_NO_PAYLOAD (CANFD_MTU - CANFD_MAX_DLEN)
void CanSocket::readerThread() {
    LOG(VERBOSE) << "Reader thread started";
    int errnoCopy = 0;

    while (!mStopReaderThread) {
        /* The ideal would be to have a blocking read(3) call and interrupt it with shutdown(3).
         * This is unfortunately not supported for SocketCAN, so we need to rely on select(3). */
        const auto sel = selectRead(mSocket, kReadPooling);
        if (sel == 0) continue;  // timeout
        if (sel == -1) {
            PLOG(ERROR) << "Select failed";
            break;
        }

        struct canfd_frame frame;
        const auto nbytes = read(mSocket.get(), &frame, CANFD_MTU);

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

        auto isValidCanLen = [](ssize_t n) {
            // all supported CAN and CANFD frame lengths
            switch (n) {
                case CAN_MTU:
                case CANFD_MTU_NO_PAYLOAD:
                case CANFD_MTU_NO_PAYLOAD+1:
                case CANFD_MTU_NO_PAYLOAD+2:
                case CANFD_MTU_NO_PAYLOAD+3:
                case CANFD_MTU_NO_PAYLOAD+4:
                case CANFD_MTU_NO_PAYLOAD+5:
                case CANFD_MTU_NO_PAYLOAD+6:
                case CANFD_MTU_NO_PAYLOAD+7:
                //case CANFD_MTU_NO_PAYLOAD+8:  // same as CAN_MTU
                case CANFD_MTU_NO_PAYLOAD+12:
                case CANFD_MTU_NO_PAYLOAD+16:
                case CANFD_MTU_NO_PAYLOAD+20:
                case CANFD_MTU_NO_PAYLOAD+24:
                case CANFD_MTU_NO_PAYLOAD+32:
                case CANFD_MTU:
                    return true;
            }
            return false;
        };

        if (isValidCanLen(nbytes)) {
            mReadCallback(frame, ts);
        } else {
            LOG(DEBUG) << "got non-canfd message with " << nbytes << " bytes";
        }
        // never break thread
    }

    bool failed = !mStopReaderThread;
    auto errCb = mErrorCallback;
    mReaderThreadFinished = true;

    // Don't access any fields from here, see CanSocket::~CanSocket comment about detached thread
    if (failed) errCb(errnoCopy);

    LOG(VERBOSE) << "Reader thread stopped";
}

}  // namespace android::hardware::automotive::can::V1_0::implementation
