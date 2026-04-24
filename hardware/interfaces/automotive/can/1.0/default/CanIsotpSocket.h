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

#pragma once

#include <android-base/macros.h>
#include <android-base/unique_fd.h>
#include <linux/can.h>

#include <vector>
#include <atomic>
#include <chrono>
#include <thread>

namespace android::hardware::automotive::can::V1_0::implementation {

#define ISOTP_TX_ID_DEFAULT 0x123
#define ISOTP_RX_ID_DEFAULT 0x124
#define ISOTP_BLOCK_SIZE_DEFAULT 4
#define ISOTP_STMIN_DEFAULT 10
#define ISOTP_DATA_LEN 8192  // 8 KB

struct canisotp_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    len;     /* frame payload length in byte, no more than ISOTP_DATA_LEN */
	__u8    flags;   /* additional flags for CANFD */
	__u8    __res0;  /* reserved / padding */
	__u8    __res1;  /* reserved / padding */
	__u8    data[ISOTP_DATA_LEN] __attribute__((aligned(8)));
};

struct CanIsotpSocket {
    using ReadCallback = std::function<void(const uint8_t *buffer, uint32_t len, std::chrono::nanoseconds)>;
    using ErrorCallback = std::function<void(int errnoVal)>;

    /**
     * Open and bind SocketCAN socket.
     *
     * \param ifname SocketCAN network interface name (such as can0)
     * \param rdcb Callback on received messages
     * \param errcb Callback on socket failure
     * \return Socket instance, or nullptr if it wasn't possible to open one
     */
    static std::unique_ptr<CanIsotpSocket> open(const std::string& ifname,
                                    uint32_t rxid, uint32_t txid, ReadCallback rdcb, ErrorCallback errcb);
    virtual ~CanIsotpSocket();

    /**
     * Send CANFD ISOTP message.
     *
     * \param frame ISOTP frame to send
     * \return true in case of success, false otherwise
     */
    bool send(const struct canisotp_frame& frame);

  private:
    CanIsotpSocket(base::unique_fd socket, ReadCallback rdcb, ErrorCallback errcb);
    void readerThread();

    ReadCallback mReadCallback;
    ErrorCallback mErrorCallback;

    const base::unique_fd mSocket;
    std::thread mReaderThread;
    std::atomic<bool> mStopReaderThread = false;
    std::atomic<bool> mReaderThreadFinished = false;

    DISALLOW_COPY_AND_ASSIGN(CanIsotpSocket);
};

}  // namespace android::hardware::automotive::can::V1_0::implementation