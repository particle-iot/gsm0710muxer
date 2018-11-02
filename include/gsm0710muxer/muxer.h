/*
 * Copyright 2018 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GSM0710_MUXER_H
#define GSM0710_MUXER_H

#include <cstdint>
#include <cstddef>
#include <atomic>
#include <memory>
#include "gsm0710muxer/muxer_def.h"
#include "gsm0710muxer/platform.h"

namespace gsm0710 {

namespace portable {

// Portable functions that need to be implemented
uint64_t getMillis();

} // portable

template <typename StreamT, typename MutexT>
class Muxer {
public:
    Muxer();
    Muxer(StreamT* stream);
    ~Muxer();

    void setStream(StreamT* stream);
    StreamT* getStream();

    int notifyInput(size_t size = 0);

    int start(bool initiator = false);
    int stop();
    bool isRunning();
    bool isStopping();

    void setAckTimeout(unsigned int t1);
    void setControlResponseTimeout(unsigned int t2);
    void setMaxFrameSize(unsigned int n1);
    void setMaxRetransmissions(unsigned int n2);
    void setKeepAlivePeriod(unsigned int p);
    void setKeepAliveMaxMissed(unsigned int m);
    void useMscAsKeepAlive(bool val);

    unsigned int getAckTimeout();
    unsigned int getControlResponseTimeout();
    unsigned int getMaxFrameSize();
    unsigned int getMaxRetransmissions();
    unsigned int getKeepAlivePeriod();
    unsigned int getKeepAliveMaxMissed();

    typedef int (*ChannelDataHandler)(const uint8_t* data, size_t len, void* ctx);

    int openChannel(uint8_t channel, ChannelDataHandler handler = nullptr, void* ctx = nullptr);
    int closeChannel(uint8_t channel);
    int setChannelDataHandler(uint8_t channel, ChannelDataHandler handler, void* ctx);
    int suspendChannel(uint8_t channel);
    int resumeChannel(uint8_t channel);

    int writeChannel(uint8_t channel, const uint8_t* data, size_t size, unsigned int timeout = 0);

    enum class ChannelState {
        Closed,
        Opened,
        Opening,
        Closing,
        Error
    };
    typedef int (*ChannelStateHandler)(uint8_t channel, ChannelState oldState, ChannelState newState, void* ctx);

    int setChannelStateHandler(ChannelStateHandler handler, void* ctx);

private:
    enum class State {
        Stopped = 0,
        Stopping,
        Starting,
        Idle,
        Flag,
        Address,
        Control,
        Length1,
        Length2,
        Data,
        Fcs,
        Error
    };

    struct Channel {
        Channel();

        static const char* stateName(ChannelState state);
        void transition(ChannelState nState);
        void reset();

        uint8_t channel = 0;

        std::atomic<ChannelState> state;
        bool update;
        ChannelDataHandler handler;
        void* handlerCtx;
        uint8_t localModemState;
        volatile uint8_t remoteModemState;
        uint8_t convergence;

        uint64_t timestamp;
        unsigned int retries;
    };

    struct ControlCommand {
        ControlCommand();

        void reset();

        uint8_t command;
        uint8_t data[proto::MAX_CONTROL_REQUEST_DATA];
        size_t len;
        uint64_t timestamp;
        unsigned int retries;

        enum class State {
            None = 0,
            Pending,
            Timeout,
            NonSupported
        };

        static const char* stateName(State s);

        std::atomic<State> state;
    };

    struct CurrentFrameState {
        uint8_t address;
        uint8_t fcs;
        uint8_t control;
        uint8_t hlen;
        size_t length;
        uint8_t* data;
    };

private:
    int run();

    int stopMuxer();
    void forceClose();

    int processInputData();
    int processTimeouts();
    int processChannelData();
    int processControlMessage();
    int processControlCommand(uint8_t oCmd, const uint8_t* data, size_t length);
    int processControlResponse(uint8_t cmd, const uint8_t* data, size_t length);
    int processModemState(const uint8_t* data, size_t length);
    int controlFinished();

    size_t readEaValue(const uint8_t* data, size_t len, unsigned int& value);

    Channel* getChannel(uint8_t channel);
    int waitChannelState(Channel* chan, ChannelState state);
    int waitWritable(Channel* c, unsigned int timeout);
    int closeChannelImpl(uint8_t channel);

    int sendChannel(uint8_t channel, uint8_t control, bool cmd, const uint8_t* data, size_t len);
    int controlSend(proto::ControlChannelCommand cmd, const uint8_t* data, size_t size);
    int controlReply(proto::ControlChannelCommand cmd, const uint8_t* data, size_t size);
    int commandSend(uint8_t channel, uint8_t control);
    int responseSend(uint8_t channel, uint8_t control);
    int modemStatusSend(Channel* c);

    ssize_t findCharacter(uint8_t c);
    ssize_t parsed(size_t size);
    ssize_t consume(size_t size);
    size_t toParse() const;
    uint8_t* curBuf();

    void transition(State state);
    int channelTransition(Channel* channel, ChannelState state);

    static const char* stateName(State state);

private:
    MutexT mutex_;
    TaskHandle_t thread_ = nullptr;
    EventGroupHandle_t events_ = nullptr;
    EventGroupHandle_t channelEvents_ = nullptr;

    enum EventSet {
        EVENT_STATE_CHANGED          = 0x01,
        EVENT_CONTROL_STATE_CHANGED  = 0x02,
        EVENT_INPUT_DATA             = 0x04,
        EVENT_STOP                   = 0x08,
        EVENT_WAKEUP                 = 0x10,
        EVENT_STOPPED                = 0x20,
        EVENT_MAX                    = 0x40
    };

    std::atomic<State> state_;

    bool stopping_ = false;

    StreamT* stream_ = nullptr;
    std::unique_ptr<uint8_t[]> inBuf_ = nullptr;
    size_t inBufSize_ = 0;
    size_t inBufData_ = 0;
    size_t inBufParserPos_ = 0;
    Channel channels_[proto::MAX_CHANNELS];
    ChannelStateHandler channelHandler_ = nullptr;
    void* channelHandlerCtx_ = nullptr;

    bool initiator_ = false;

    unsigned int ackTimeout_ = proto::DEFAULT_T1;
    unsigned int controlResponseTimeout_ = proto::DEFAULT_T2;
    unsigned int maxFrameSize_ = proto::DEFAULT_N1;
    unsigned int maxRetransmissions_ = proto::DEFAULT_N2;
    unsigned int keepAlivePeriod_ = 0;
    bool useMscKeepAlive_ = false;

    uint64_t lastKeepAlive_ = 0;
    unsigned int keepAliveMaxMissed_ = 0;
    unsigned int keepAlivesMissed_ = 0;

    ControlCommand ctrl_ = {};

    CurrentFrameState frame_ = {};

#ifdef LOG_CATEGORY
    LOG_CATEGORY("gsm0710muxer");
#endif // LOG_CATEGORY
};

} // gsm0710

#include "gsm0710muxer/muxer_impl.h"

#endif // GSM0710_MUXER_H
