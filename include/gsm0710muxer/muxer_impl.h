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

#ifndef GSM0710_MUXER_IMPL_H
#define GSM0710_MUXER_IMPL_H

#include <mutex>
#include <type_traits>
#include <cstring>
#include "gsm0710muxer/muxer_def.h"
#include "gsm0710muxer/platform.h"

#ifdef GSM0710_ENABLE_DEBUG_LOGGING
#define GSM0710_LOG_DEBUG(_level, _fmt, ...) LOG_DEBUG(_level, _fmt, ##__VA_ARGS__)
#else
#define GSM0710_LOG_DEBUG(_level, _fmt, ...)
#endif // GSM0710_ENABLE_DEBUG_LOGGING

namespace gsm0710 {

namespace detail {
extern const uint8_t crcTable[256];
} // detail

inline uint8_t crc(const uint8_t* data, size_t len) {
    uint8_t c = 0xff;
    for (size_t i = 0; i < len; i++) {
        c = detail::crcTable[c ^ data[i]];
    }

    return c;
}

inline uint8_t fcs(const uint8_t* data, size_t len) {
    return ~crc(data, len);
}

inline bool validateFcs(uint8_t refFcs, const uint8_t* data, size_t len) {
    uint8_t fcs = detail::crcTable[crc(data, len) ^ refFcs];
    return fcs == 0xcf;
}

template<typename StreamT, typename MutexT>
inline Muxer<StreamT, MutexT>::Muxer()
        : state_(State::Stopped) {
}

template<typename StreamT, typename MutexT>
inline Muxer<StreamT, MutexT>::Muxer(StreamT* stream)
        : Muxer() {
    setStream(stream);
}

template<typename StreamT, typename MutexT>
inline Muxer<StreamT, MutexT>::~Muxer() {
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::setStream(StreamT* stream) {
    std::lock_guard<MutexT> lk(mutex_);
    stream_ = stream;
}

template<typename StreamT, typename MutexT>
inline StreamT* Muxer<StreamT, MutexT>::getStream() {
    return stream_;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::notifyInput(size_t size) {
    if (isRunning()) {
        xEventGroupSetBits(events_, EVENT_INPUT_DATA);
        return 0;
    }

    return GSM0710_ERROR_INVALID_STATE;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::start(bool initiator) {
    stop();

    LOG(INFO, "Starting GSM07.10 muxer");

    {
        std::lock_guard<MutexT> lk(mutex_);

        if (!inBuf_ || inBufSize_ != maxFrameSize_) {
            inBufSize_ = maxFrameSize_;
            inBuf_.reset(new (std::nothrow) uint8_t[inBufSize_]);
            CHECK_TRUE(inBuf_, GSM0710_ERROR_NO_MEMORY);
        }

        initiator_ = initiator;
        inBufData_ = 0;
        inBufParserPos_ = 0;
        ctrl_.reset();
        frame_ = {};
        lastKeepAlive_ = 0;
        keepAlivesMissed_ = 0;
        stopping_ = false;

        for (unsigned c = 0; c < sizeof(channels_) / sizeof(channels_[0]); c++) {
            auto& chan = channels_[c];
            chan.channel = c;
            chan.reset();
        }

        if (!events_) {
            events_ = xEventGroupCreate();
            CHECK_TRUE(events_, GSM0710_ERROR_NO_MEMORY);
        }

        if (!channelEvents_) {
            channelEvents_ = xEventGroupCreate();
            CHECK_TRUE(channelEvents_, GSM0710_ERROR_NO_MEMORY);
        }

        xEventGroupClearBits(events_, EVENT_MAX - 1);
        xEventGroupClearBits(channelEvents_,
                ((EVENT_STATE_CHANGED << ((sizeof(channels_) / sizeof(channels_[0])) + 1)) - 1));

        state_ = State::Stopped;
        transition(State::Starting);

        // Start thread
        if (pdPASS != xTaskCreate([](void* arg) -> void {
                    auto self = (Muxer<StreamT, MutexT>*)arg;
                    self->run();
                    vTaskDelete(nullptr);
                }, "gsm0710", portable::taskStackSize / sizeof(portSTACK_TYPE), this, portable::taskPriority, &thread_)) {
            transition(State::Error);
            return GSM0710_ERROR_UNKNOWN;
        }
    }

    if (initiator) {
        return openChannel(0);
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::stop() {
    if (!isRunning() && !thread_) {
        return GSM0710_ERROR_NONE;
    }

    LOG(INFO, "Stopping GSM07.10 muxer");

    if (isRunning()) {
        xEventGroupSetBits(events_, EVENT_STOP);
    }
    xEventGroupWaitBits(events_, EVENT_STOPPED, pdTRUE, pdFALSE, portMAX_DELAY);

    thread_ = nullptr;

    LOG(INFO, "GSM07.10 muxer stopped");

    return 0;
}

template<typename StreamT, typename MutexT>
inline bool Muxer<StreamT, MutexT>::isRunning() {
    return state_ != State::Stopped && state_ != State::Error;
}

template<typename StreamT, typename MutexT>
inline bool Muxer<StreamT, MutexT>::isStopping() {
    return !isRunning() || stopping_;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::setAckTimeout(unsigned int t1) {
    ackTimeout_ = t1;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::setControlResponseTimeout(unsigned int t2) {
    controlResponseTimeout_ = t2;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::setMaxFrameSize(unsigned int n1) {
    maxFrameSize_ = n1;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::setMaxRetransmissions(unsigned int n2) {
    maxRetransmissions_ = n2;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::setKeepAlivePeriod(unsigned int p) {
    keepAlivePeriod_ = p;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::setKeepAliveMaxMissed(unsigned int m) {
    keepAliveMaxMissed_ = m;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::useMscAsKeepAlive(bool val) {
    useMscKeepAlive_ = val;
}

template<typename StreamT, typename MutexT>
inline unsigned int Muxer<StreamT, MutexT>::getAckTimeout() {
    return ackTimeout_;
}

template<typename StreamT, typename MutexT>
inline unsigned int Muxer<StreamT, MutexT>::getControlResponseTimeout() {
    return controlResponseTimeout_;
}

template<typename StreamT, typename MutexT>
inline unsigned int Muxer<StreamT, MutexT>::getMaxFrameSize() {
    return maxFrameSize_;
}

template<typename StreamT, typename MutexT>
inline unsigned int Muxer<StreamT, MutexT>::getMaxRetransmissions() {
    return maxRetransmissions_;
}

template<typename StreamT, typename MutexT>
inline unsigned int Muxer<StreamT, MutexT>::getKeepAlivePeriod() {
    return keepAlivePeriod_;
}

template<typename StreamT, typename MutexT>
inline unsigned int Muxer<StreamT, MutexT>::getKeepAliveMaxMissed() {
    return keepAliveMaxMissed_;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::openChannel(uint8_t channel, ChannelDataHandler handler, void* ctx) {
    CHECK_TRUE(!isStopping(), GSM0710_ERROR_INVALID_STATE);
    auto c = getChannel(channel);

    CHECK_TRUE(c, GSM0710_ERROR_INVALID_ARGUMENT);

    int r = 0;

    if (c->state == ChannelState::Closed || c->state == ChannelState::Closing || c->state == ChannelState::Error) {
        LOG(INFO, "Openning mux channel %u", channel);
        {
            std::lock_guard<MutexT> lk(mutex_);
            c->reset();
            channelTransition(c, ChannelState::Opening);
            c->timestamp = 0;
            c->retries = 0;
            if (handler != nullptr) {
                c->handler = handler;
                c->handlerCtx = ctx;
            }
        }
        r = waitChannelState(c, ChannelState::Opened);
        if (!r) {
            // Automatically resume channel
            r = resumeChannel(channel);
        }
    } else {
        LOG(INFO, "Mux channel %u already opened", channel);
        std::lock_guard<MutexT> lk(mutex_);
        if (handler != nullptr) {
            c->handler = handler;
            c->handlerCtx = ctx;
        }
    }

    return r;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::closeChannel(uint8_t channel) {
    CHECK_TRUE(!isStopping(), GSM0710_ERROR_INVALID_STATE);
    auto c = getChannel(channel);
    CHECK_TRUE(c, GSM0710_ERROR_INVALID_ARGUMENT);

    CHECK(closeChannelImpl(channel));
    return waitChannelState(c, ChannelState::Closed);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::closeChannelImpl(uint8_t channel) {
    std::lock_guard<MutexT> lk(mutex_);
    auto c = getChannel(channel);
    CHECK_TRUE(c, GSM0710_ERROR_INVALID_ARGUMENT);

    if (c->state == ChannelState::Opened || c->state == ChannelState::Opening) {
        LOG(INFO, "Closing mux channel %u", channel);
        channelTransition(c, ChannelState::Closing);
        c->timestamp = 0;
        c->retries = 0;
    } else {
        LOG(INFO, "Muxed channel %u already closed", channel);
    }
    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::setChannelDataHandler(uint8_t channel, ChannelDataHandler handler, void* ctx) {
    auto c = getChannel(channel);
    CHECK_TRUE(c, GSM0710_ERROR_INVALID_ARGUMENT);

    std::lock_guard<MutexT> lk(mutex_);

    c->handler = handler;
    c->handlerCtx = ctx;

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::suspendChannel(uint8_t channel) {
    CHECK_TRUE(!isStopping(), GSM0710_ERROR_INVALID_STATE);
    auto c = getChannel(channel);
    CHECK_TRUE(c, GSM0710_ERROR_INVALID_ARGUMENT);

    LOG(INFO, "Suspending channel %u", channel);

    std::lock_guard<MutexT> lk(mutex_);

    c->localModemState &= ~(proto::RTR | proto::RTC);
    c->update = true;
    xEventGroupSetBits(events_, EVENT_WAKEUP);

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::resumeChannel(uint8_t channel) {
    CHECK_TRUE(!isStopping(), GSM0710_ERROR_INVALID_STATE);
    auto c = getChannel(channel);
    CHECK_TRUE(c, GSM0710_ERROR_INVALID_ARGUMENT);

    LOG(INFO, "Resuming channel %d", channel);

    std::lock_guard<MutexT> lk(mutex_);

    c->localModemState |= (proto::RTR | proto::RTC);
    c->update = true;
    xEventGroupSetBits(events_, EVENT_WAKEUP);

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::writeChannel(uint8_t channel, const uint8_t* data, size_t size, unsigned int timeout) {
    CHECK_TRUE(!isStopping(), GSM0710_ERROR_INVALID_STATE);
    auto c = getChannel(channel);
    CHECK_TRUE(c, GSM0710_ERROR_INVALID_ARGUMENT);
    CHECK_TRUE(c->state == ChannelState::Opened, GSM0710_ERROR_INVALID_STATE);
    GSM0710_LOG_DEBUG(TRACE, "Writing %u bytes into mux channel %u", size, channel);
    {
        CHECK(waitWritable(c, timeout));
    }
    return sendChannel(channel, proto::UIH, false, data, size);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::waitWritable(Channel* chan, unsigned int timeout) {
    if ((chan->remoteModemState & (proto::RTR))) {
        return 0;
    } else if (timeout > 0) {
        auto t1 = portable::getMillis();
        while ((portable::getMillis() - t1) < timeout && isRunning() &&
                chan->state != ChannelState::Error && chan->state != ChannelState::Closed) {
            int toWait = timeout - (portable::getMillis() - t1);
            if (toWait > 0) {
                xEventGroupWaitBits(channelEvents_, EVENT_STATE_CHANGED << chan->channel, pdTRUE, pdFALSE, toWait / portTICK_RATE_MS);
                if (chan->remoteModemState & proto::RTR) {
                    return 0;
                }
            }
        }
        GSM0710_LOG_DEBUG(ERROR, "Channel %u is still not writable after waiting for %u ms", chan->channel, timeout);
    }
    return GSM0710_ERROR_FLOW_CONTROL;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::setChannelStateHandler(ChannelStateHandler handler, void* ctx) {
    std::lock_guard<MutexT> lk(mutex_);
    channelHandler_ = handler;
    channelHandlerCtx_ = ctx;
    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::run() {
    LOG(INFO, "GSM07.10 muxer thread started");
    transition(State::Idle);

    while (state_ != State::Stopped && state_ != State::Error) {
        auto nextTimeout = processTimeouts();
        if (nextTimeout < 0) {
            transition(State::Error);
            break;
        }

#ifdef GSM0710_PUMP_INPUT_DATA
        nextTimeout = std::min(nextTimeout, GSM0710_PUMP_INPUT_DATA);
#endif

        auto ev = xEventGroupWaitBits(events_, EVENT_INPUT_DATA | EVENT_STOP | EVENT_WAKEUP, pdTRUE,
                pdFALSE, nextTimeout / portTICK_RATE_MS);

        if ((ev & EVENT_STOP) || stopping_) {
            stopMuxer();
        }

        if (ev & EVENT_WAKEUP) {
            GSM0710_LOG_DEBUG(TRACE, "Woken up");
            nextTimeout = processTimeouts();
            if (nextTimeout < 0) {
                transition(State::Error);
                break;
            }
        }

#if !defined(GSM0710_PUMP_INPUT_DATA)
        if (ev & EVENT_INPUT_DATA) {
#else
        {
#endif // !defined(GSM0710_PUMP_INPUT_DATA)
            if (processInputData() < 0) {
                transition(State::Error);
                break;
            }
        }
    }

    LOG(INFO, "GSM07.10 muxer thread exiting");

    xEventGroupSetBits(events_, EVENT_STOPPED);

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::stopMuxer() {
    if (state_ == State::Stopped) {
        // Done
        return 0;
    }

    std::lock_guard<MutexT> lk(mutex_);

    if (!stopping_) {
        LOG(INFO, "Gracefully stopping GSM07.10 muxer");
        LOG(INFO, "Closing all muxed channels");
        for (unsigned c = 1; c < sizeof(channels_) / sizeof(channels_[0]); c++) {
            closeChannelImpl(c);
        }
        stopping_ = true;
    }

    bool readyToCld = true;

    for (unsigned c = 1; c < sizeof(channels_) / sizeof(channels_[0]); c++) {
        if (channels_[c].state != ChannelState::Closed && channels_[c].state != ChannelState::Error) {
            readyToCld = false;
            break;
        }
    }

    if (readyToCld && ctrl_.state != ControlCommand::State::Pending) {
        LOG(INFO, "Sending CLD (multiplexer close down)");
        controlSend(proto::CLD, nullptr, 0);
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::forceClose() {
    for (unsigned c = 1; c < sizeof(channels_) / sizeof(channels_[0]); c++) {
        auto chan = getChannel(c);
        if (chan->state != ChannelState::Closed) {
            channelTransition(getChannel(c), ChannelState::Closed);
        }
    }

    channelTransition(getChannel(0), ChannelState::Closed);
    transition(State::Stopped);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::processInputData() {
    size_t newData = CHECK(stream_->read((char*)inBuf_.get() + inBufData_, inBufSize_ - inBufData_));
    if (newData > 0) {
        GSM0710_LOG_DEBUG(TRACE, "New data to process %u", newData);
        inBufData_ += newData;
        // LOG_DUMP(TRACE, inBuf_.get(), inBufData_);
        // LOG_PRINTF(TRACE, "\r\n");
    }

    bool forceExit = false;

    while (toParse() > 0 && !forceExit && state_ != State::Stopped) {
        GSM0710_LOG_DEBUG(TRACE, "Data in buffer = %u, parser position = %u", inBufData_, inBufParserPos_);
        switch (state_) {
            case State::Idle: {
                auto pos = findCharacter(proto::BASIC_FLAG);
                if (pos >= 0) {
                    // Consume everything up to the flag
                    if (pos > 0) {
                        consume(pos);
                    }
                    transition(State::Flag);
                    frame_ = {};
                } else {
                    // Skip anything we've received so far
                    consume(inBufData_);
                }
                break;
            }

            case State::Flag: {
                GSM0710_LOG_DEBUG(TRACE, "Found flag byte");
                // Skip any additional repeated flags
                auto fpos = findCharacter(proto::BASIC_FLAG);
                if (fpos == 0) {
                    // Mark flag byte as parsed
                    parsed(1);
                } else {
                    // Consume repeated bytes
                    if (inBufParserPos_ > 1) {
                        consume(inBufParserPos_ - 1);
                    }
                    transition(State::Address);
                }
                break;
            }

            case State::Address: {
                frame_.address = *curBuf();
                // Mark address byte as parsed
                parsed(1);
                if (!(frame_.address & proto::EA)) {
                    // EA bit should have been set to 1
                    // Consume erroneous flag and address bytes
                    // Go back to Idle state
                    GSM0710_LOG_DEBUG(ERROR, "Invalid address, EA bit is not set (%02x)", frame_.address);
                    consume(2);
                    transition(State::Idle);
                } else {
                    GSM0710_LOG_DEBUG(TRACE, "Address: %02x (%u)", frame_.address, frame_.address >> 2);
                    transition(State::Control);
                }
                break;
            }

            case State::Control: {
                frame_.control = *curBuf();
                // Mark control byte as parsed
                parsed(1);
                GSM0710_LOG_DEBUG(TRACE, "Control: %02x", frame_.control);
                transition(State::Length1);
                break;
            }

            case State::Length1: {
                frame_.length = *curBuf() >> 1;
                if (!(*curBuf() & proto::EA)) {
                    // Extension bit not present, there is another length byte
                    transition(State::Length2);
                    frame_.hlen = 5;
                } else {
                    frame_.hlen = 4;
                    // Single-byte length field
                    // Validate length
                    GSM0710_LOG_DEBUG(TRACE, "Length: %u, total %u", frame_.length, frame_.hlen + frame_.length + 1);
                    if ((frame_.length + frame_.hlen + 1) > inBufSize_) {
                        // The frame will not fit, consume any read data, go back into Idle state
                        GSM0710_LOG_DEBUG(ERROR, "Received frame will not fit into internal buffer");
                        consume(inBufData_);
                        transition(State::Idle);
                        break;
                    } else {
                        transition(State::Data);
                    }
                }
                // Mark first length byte as parsed
                parsed(1);
                break;
            }

            case State::Length2: {
                frame_.length |= ((size_t)*curBuf()) << 7;
                parsed(1);
                GSM0710_LOG_DEBUG(TRACE, "Length2: %u, total %u", frame_.length, frame_.hlen + frame_.length + 1);
                // Validate length
                if ((frame_.length + frame_.hlen + 1) > inBufSize_) {
                    // The frame will not fit, consume any read data, go back into Idle state
                    GSM0710_LOG_DEBUG(ERROR, "Received frame will not fit into internal buffer");
                    consume(inBufData_);
                    transition(State::Idle);
                    break;
                } else {
                    transition(State::Data);
                }
                break;
            }

            case State::Data: {
                if (toParse() >= frame_.length) {
                    // Mark frame data as parsed
                    frame_.data = curBuf();
                    parsed(frame_.length);
                    transition(State::Fcs);
                } else {
                    forceExit = true;
                }
                break;
            }

            case State::Fcs: {
                frame_.fcs = *curBuf();
                // Mark FCS byte as parsed
                parsed(1);
                if (validateFcs(frame_.fcs, inBuf_.get() + 1, frame_.hlen - 1)) {
                    // Valid frame, process frame data
                    processChannelData();
                } else {
                    GSM0710_LOG_DEBUG(ERROR, "Invalid FCS");
                }

                // Consume all the parsed data
                consume(inBufParserPos_);
                if (isRunning()) {
                    // Go back into Idle state
                    transition(State::Idle);
                }
                break;
            }

            default: {
                return GSM0710_ERROR_INVALID_STATE;
            }
        }
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::processTimeouts() {
    if (!initiator_ && getChannel(0)->state != ChannelState::Opened && !isStopping()) {
        // FIXME: for now returning T1 / 2
        return getAckTimeout() / 2;
    }

    std::lock_guard<MutexT> lk(mutex_);

    if (ctrl_.state == ControlCommand::State::Pending) {
        if ((portable::getMillis() - ctrl_.timestamp) >= getControlResponseTimeout()) {
            // Time to retry
            if (ctrl_.retries++ < getMaxRetransmissions()) {
                GSM0710_LOG_DEBUG(TRACE, "(%d/%d) Retrying control command: %02x, len = %u",
                        ctrl_.retries, getMaxRetransmissions(), ctrl_.command, ctrl_.len);
                CHECK(sendChannel(0, proto::UIH, true, ctrl_.data, ctrl_.len));
                ctrl_.timestamp = portable::getMillis();
            } else {
                ctrl_.state = ControlCommand::State::Timeout;
                controlFinished();
            }
        }
    }

    for (unsigned c = 0; c < sizeof(channels_) / sizeof(channels_[0]); c++) {
        auto chan = &channels_[c];
        if (chan->state == ChannelState::Closing || chan->state == ChannelState::Opening) {
            // Transitive state
            if ((portable::getMillis() - chan->timestamp) >= getAckTimeout()) {
                if (chan->retries++ < getMaxRetransmissions()) {
                    if (chan->state == ChannelState::Closing) {
                        GSM0710_LOG_DEBUG(TRACE, "(%d/%d) Trying to close channel %u",
                                chan->retries, getMaxRetransmissions(), chan->channel);
                        CHECK(commandSend(c, proto::DISC | proto::PF));
                    } else {
                        GSM0710_LOG_DEBUG(TRACE, "(%d/%d) Trying to open channel %u",
                                chan->retries, getMaxRetransmissions(), chan->channel);
                        CHECK(commandSend(c, proto::SABM | proto::PF));
                    }
                    chan->timestamp = portable::getMillis();
                } else {
                    if (chan->state == ChannelState::Closing) {
                        GSM0710_LOG_DEBUG(ERROR, "Failed to close channel %u", chan->channel);
                    } else {
                        GSM0710_LOG_DEBUG(ERROR, "Failed to open channel %u", chan->channel);
                    }
                    channelTransition(chan, ChannelState::Error);
                }
            }
        } else if (c > 0 && chan->update == true && chan->state == ChannelState::Opened) {
            if (ctrl_.state != ControlCommand::State::Pending) {
                chan->update = false;
                CHECK(modemStatusSend(chan));
            }
        }
    }

    if (initiator_ && getChannel(0)->state == ChannelState::Opened && keepAlivePeriod_ && !stopping_ && ctrl_.state != ControlCommand::State::Pending &&
            (portable::getMillis() - lastKeepAlive_) >= keepAlivePeriod_) {
        if (!useMscKeepAlive_) {
            CHECK(controlSend(proto::TEST, (const uint8_t*)"abc", 3));
        } else {
            CHECK(modemStatusSend(getChannel(1)));
        }
        lastKeepAlive_ = portable::getMillis();
    }

    // FIXME: for now returning T1 / 2
    return getAckTimeout() / 2;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::processChannelData() {
    std::unique_lock<MutexT> lk(mutex_);

    uint8_t channel = frame_.address >> 2;
    GSM0710_LOG_DEBUG(TRACE, "New frame on channel %u, control = %02x, length = %u, fcs = %02x",
            channel, frame_.control, frame_.length, frame_.fcs);;
    auto c = getChannel(channel);
    switch (frame_.control) {
        case proto::SABM | proto::PF: {
            GSM0710_LOG_DEBUG(TRACE, "SABM");
            if (c) {
                if (!channelTransition(c, ChannelState::Opened)) {
                    c->retries = 0;
                    c->timestamp = 0;
                    c->update = true;
                    GSM0710_LOG_DEBUG(TRACE, "Channel %u sucessfully opened, replying with UA", channel);
                    responseSend(channel, proto::UA | proto::PF);
                } else {
                    // Denied by channel handler
                    GSM0710_LOG_DEBUG(TRACE, "Channel %u failed to open due denial in channel handler. Replying with DM", channel);
                    responseSend(channel, proto::DM | proto::PF);
                }
            } else {
                GSM0710_LOG_DEBUG(WARN, "Received SABM on an unknown channel %u, replying with DM", channel);
                responseSend(channel, proto::DM | proto::PF);
            }
            break;
        }
        case proto::UA | proto::PF: {
            GSM0710_LOG_DEBUG(TRACE, "UA");
            if (c) {
                switch (c->state) {
                    case ChannelState::Closing: {
                        channelTransition(c, ChannelState::Closed);
                        break;
                    }
                    case ChannelState::Opening: {
                        channelTransition(c, ChannelState::Opened);
                        break;
                    }
                    default: {
                        GSM0710_LOG_DEBUG(WARN, "Received UA for a channel (%u) in %s state",
                                c->channel, c->stateName(c->state));
                        break;
                    }
                }
            } else {
                GSM0710_LOG_DEBUG(WARN, "Received UA on an unknown channel %u, ignoring", channel);
            }
            break;
        }
        case proto::DM:
        case proto::DM | proto::PF: {
            GSM0710_LOG_DEBUG(TRACE, "DM");
            if (c) {
                channelTransition(c, ChannelState::Closed);
            } else {
                GSM0710_LOG_DEBUG(WARN, "Received DM on an unknown channel %u, ignoring", channel);
            }
            break;
        }
        case proto::DISC | proto::PF: {
            GSM0710_LOG_DEBUG(TRACE, "DISC");
            if (!c || c->state == ChannelState::Closed) {
                if (!c) {
                    GSM0710_LOG_DEBUG(WARN, "Received DISC on an unknown channel %u, replying with DM", channel);
                } else {
                    GSM0710_LOG_DEBUG(WARN, "Received DISC for a closed channel %u, replying with DM", channel);
                }
                responseSend(channel, proto::DM | proto::PF);
            } else {
                channelTransition(c, ChannelState::Closed);
                GSM0710_LOG_DEBUG(TRACE, "Replying with UA");
                responseSend(channel, proto::UA | proto::PF);
                if (channel == 0) {
                    LOG(INFO, "Muxer channel 0 closed by the other side, exiting multiplexed mode");
                    forceClose();
                }
            }
            break;
        }

        case proto::UIH:
        case proto::UI:
        case proto::UIH | proto::PF:
        case proto::UI | proto::PF: {
            GSM0710_LOG_DEBUG(TRACE, "UI/UIH");
            if (!c || c->state != ChannelState::Opened) {
                if (!c) {
                    GSM0710_LOG_DEBUG(WARN, "Received UI/UIH on an unknown channel %u, replying with DM", channel);
                } else {
                    GSM0710_LOG_DEBUG(WARN, "Received UI/UIH for a non-open channel %u, replying with DM", channel);
                }
                commandSend(channel, proto::DM | proto::PF);
            } else {
                if (channel > 0) {
                    if (c->handler) {
                        auto h = c->handler;
                        auto ctx = c->handlerCtx;
                        lk.unlock();
                        h(frame_.data, frame_.length, ctx);
                    } else {
                        GSM0710_LOG_DEBUG(TRACE, "Frame ignored, no channel data handler set");
                    }
                } else {
                    if (frame_.length > 1) {
                        processControlMessage();
                    }
                }
            }
            break;
        }
    }
    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::processControlMessage() {
    size_t toProcess = frame_.length;
    size_t pos = 0;
    while ((toProcess - pos) > 0 && state_ != State::Stopped) {
        // Get type
        unsigned int cmd = 0;
        size_t cmdLength = readEaValue(frame_.data + pos, toProcess - pos, cmd);
        CHECK_TRUE(cmdLength == 1, GSM0710_ERROR_BAD_DATA);
        pos += cmdLength;

        // Get length
        unsigned int length = 0;
        size_t lengthLength = readEaValue(frame_.data + pos, toProcess - pos, length);
        CHECK_TRUE((pos + lengthLength + length) == toProcess, GSM0710_ERROR_BAD_DATA);
        pos += lengthLength;

        cmd <<= 1;
        if (cmd & proto::CR) {
            processControlCommand(cmd, frame_.data + pos, length);
        } else {
            processControlResponse(cmd, frame_.data + pos, length);
        }

        pos += length;
    }
    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::processControlCommand(uint8_t oCmd, const uint8_t* data, size_t length) {
    GSM0710_LOG_DEBUG(TRACE, "Control command %02x, length = %u", oCmd, length);
    auto cmd = oCmd & ~(proto::CR);
    switch (cmd) {
        case proto::PN: {
            GSM0710_LOG_DEBUG(TRACE, "PN");
            break;
        }

        case proto::PSC: {
            GSM0710_LOG_DEBUG(TRACE, "PSC, not implemented, still replying");
            CHECK_TRUE(length == 0, GSM0710_ERROR_BAD_DATA);
            // Not implemented, but we'll reply just in case
            controlReply(proto::PSC, nullptr, 0);
            break;
        }

        case proto::CLD: {
            LOG(INFO, "Received CLD, exiting multiplexed mode");
            CHECK_TRUE(length == 0, GSM0710_ERROR_BAD_DATA);
            // Ack
            controlReply(proto::CLD, nullptr, 0);
            forceClose();
            break;
        }

        case proto::TEST: {
            GSM0710_LOG_DEBUG(TRACE, "TEST, replying with the same data (length = %u) back", length);
            // Reply with the same data
            controlReply(proto::TEST, data, length);
            break;
        }

        case proto::MSC: {
            GSM0710_LOG_DEBUG(TRACE, "MSC");
            // Modem state
            processModemState(data, length);
            break;
        }

        case proto::RLS: {
            GSM0710_LOG_DEBUG(TRACE, "RLS, replying with the same data (length = %u) that we've received", length);
            // When the remote line status command is received,
            // the remote device must respond with a Remote Line Status Response
            // containing the values that it received
            controlReply(proto::RLS, data, length);
            break;
        }

        case proto::FCON:
        case proto::FCOFF:
        case proto::RPN:
        case proto::SNC:
        default: {
            controlReply(proto::NSC, &oCmd, 1);
            break;
        }
    }
    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::processControlResponse(uint8_t cmd, const uint8_t* data, size_t length) {
    GSM0710_LOG_DEBUG(TRACE, "Control response %02x, length = %u", cmd, length);
    CHECK_TRUE(ctrl_.state == ControlCommand::State::Pending, GSM0710_ERROR_INVALID_STATE);
    if (ctrl_.command == (cmd | proto::CR)) {
        // Matches
        ctrl_.state = ControlCommand::State::None;
        controlFinished();
    } else if (cmd == proto::NSC) {
        ctrl_.state = ControlCommand::State::NonSupported;
        controlFinished();
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::controlFinished() {
    GSM0710_LOG_DEBUG(TRACE, "Control command %02x finished, result = %s", ctrl_.command, ctrl_.stateName(ctrl_.state));
    if (ctrl_.command == (proto::CLD | proto::CR) && stopping_) {
        LOG(INFO, "Received response to CLD or timed out, exiting multiplexed mode");
        forceClose();
    } else if ((ctrl_.command == (proto::TEST | proto::CR)) ||
            (useMscKeepAlive_ && ctrl_.command == (proto::MSC | proto::CR) && (ctrl_.data[2] >> 2) == 1)) {
        if (ctrl_.state != ControlCommand::State::Timeout) {
            keepAlivesMissed_ = 0;
        } else {
            keepAlivesMissed_++;
            if (keepAliveMaxMissed_ && keepAlivesMissed_ >= keepAliveMaxMissed_) {
                LOG(ERROR, "The other end has not replied to keep alives (TESTs) %u times, considering muxed connection dead", keepAlivesMissed_);
                forceClose();
            }
        }
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::processModemState(const uint8_t* data, size_t length) {
    CHECK_TRUE(length >= 2, GSM0710_ERROR_BAD_DATA);
    uint8_t channel = data[0] >> 2;
    unsigned int v24;
    size_t l = readEaValue(data + 1, length - 1, v24);
    // Ignore anything else

    auto c = getChannel(channel);
    if (c && l >= 1) {
        GSM0710_LOG_DEBUG(INFO, "Updating channel %u remote modem state. Old = %02x, new = %02x", channel, c->remoteModemState, v24);
        c->remoteModemState = v24;
        xEventGroupSetBits(channelEvents_, EVENT_STATE_CHANGED << channel);
    }

    controlReply(proto::MSC, data, length);
    return 0;
}

template<typename StreamT, typename MutexT>
inline size_t Muxer<StreamT, MutexT>::readEaValue(const uint8_t* data, size_t len, unsigned int& value) {
    value = 0;
    for (size_t i = 0; i < len; i++) {
        // Remove EA bit
        uint8_t v = data[i] >> 1;
        value |= v << (7 * i);
        if (data[i] & proto::EA) {
            return i + 1;
        }
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline typename Muxer<StreamT, MutexT>::Channel* Muxer<StreamT, MutexT>::getChannel(uint8_t channel) {
    std::lock_guard<MutexT> lk(mutex_);
    if (channel < sizeof(channels_) / sizeof(channels_[0])) {
        return &channels_[channel];
    }

    return nullptr;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::waitChannelState(Channel* chan, ChannelState state) {
    GSM0710_LOG_DEBUG(TRACE, "Waiting for channel %u in state %s to transition to %s",
            chan->channel, chan->stateName(chan->state), chan->stateName(state));
    while (isRunning() && chan->state != state && chan->state != ChannelState::Error) {
        xEventGroupWaitBits(channelEvents_, EVENT_STATE_CHANGED << chan->channel, pdTRUE, pdFALSE, 100 / portTICK_RATE_MS);
    }

    return !(chan->state == state);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::sendChannel(uint8_t channel, uint8_t control, bool cmd, const uint8_t* data, size_t len) {
    std::lock_guard<MutexT> lk(mutex_);

    uint8_t header[5];
    uint8_t footer[2];
    const size_t hlen = len <= 0x7f ? 4 : 5;

    header[0] = proto::BASIC_FLAG;
    header[1] = (channel << 2) | proto::EA;
    if ((initiator_ && cmd) || (!initiator_ && !cmd)) {
        header[1] |= proto::CR;
    }
    header[2] = control;
    header[3] = (len & 0x7f) << 1;
    if (hlen == 5) {
        header[4] = len >> 7;
    } else {
        header[3] |= proto::EA;
    }

    footer[0] = fcs(header + 1, hlen - 1);
    footer[1] = proto::BASIC_FLAG;

    // LOG_DUMP(TRACE, header, hlen);
    // if (len) {
    //     LOG_DUMP(TRACE, data, len);
    // }
    // LOG_DUMP(TRACE, footer, sizeof(footer));

    auto t1 = portable::getMillis();

    size_t sent = 0;
    while (sent < hlen) {
        sent += CHECK(stream_->write((const char*)header + sent, hlen - sent));
        if ((portable::getMillis() - t1) > getControlResponseTimeout() * 2) {
            return GSM0710_ERROR_FLOW_CONTROL;
        }
    }

    t1 = portable::getMillis();
    if (len) {
        sent = 0;
        while (sent < len) {
            sent += CHECK(stream_->write((const char*)data + sent, len - sent));
            if ((portable::getMillis() - t1) > getControlResponseTimeout() * 2) {
                return GSM0710_ERROR_FLOW_CONTROL;
            }
        }
    }

    sent = 0;
    t1 = portable::getMillis();
    while (sent < sizeof(footer)) {
        sent += CHECK(stream_->write((const char*)footer + sent, sizeof(footer) - sent));
        if ((portable::getMillis() - t1) > getControlResponseTimeout() * 2) {
            return GSM0710_ERROR_FLOW_CONTROL;
        }
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::controlSend(proto::ControlChannelCommand cmd, const uint8_t* data, size_t size) {
    GSM0710_LOG_DEBUG(TRACE, "Sending control command %02x, length = %u", cmd, size);
    std::unique_lock<MutexT> lk(mutex_);

    while (true) {
        if (ctrl_.state == ControlCommand::State::Pending) {
            GSM0710_LOG_DEBUG(TRACE, "Waiting for current pending control command (%02x) to get acked or timeout",
                    ctrl_.command);
            lk.unlock();
            xEventGroupWaitBits(events_, EVENT_CONTROL_STATE_CHANGED, pdTRUE, pdFALSE, 100 / portTICK_RATE_MS);
            lk.lock();
        } else {
            break;
        }
    }

    ctrl_.state = ControlCommand::State::Pending;
    ctrl_.len = 2 + size;
    ctrl_.command = cmd | proto::CR;
    ctrl_.data[0] = cmd | proto::EA | proto::CR;
    ctrl_.data[1] = ((uint8_t)size << 1) | proto::EA;
    memcpy(ctrl_.data + 2, data, size);
    ctrl_.timestamp = 0;
    ctrl_.retries = 0;
    lk.unlock();
    xEventGroupSetBits(events_, EVENT_WAKEUP);
    return 0;
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::controlReply(proto::ControlChannelCommand cmd, const uint8_t* data, size_t size) {
    GSM0710_LOG_DEBUG(TRACE, "Replying to control command %02x, length = %u", cmd, size);
    uint8_t buf[2 + size] = {};
    buf[0] = (cmd | proto::EA) & ~(proto::CR);
    buf[1] = ((uint8_t)size << 1) | proto::EA;
    memcpy(buf + 2, data, size);
    return sendChannel(0, proto::UIH, false, buf, size + 2);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::commandSend(uint8_t channel, uint8_t control) {
    GSM0710_LOG_DEBUG(TRACE, "Sending command %02x on channel %u", control, channel);
    return sendChannel(channel, control, true, nullptr, 0);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::responseSend(uint8_t channel, uint8_t control) {
    GSM0710_LOG_DEBUG(TRACE, "Sending response %02x on channel %u", control, channel);
    return sendChannel(channel, control, false, nullptr, 0);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::modemStatusSend(Channel* c) {
    GSM0710_LOG_DEBUG(TRACE, "Sending local modem status on channel %u", c->channel);
#if !defined(GSM0710_MODEM_STATUS_SEND_EMPTY_BREAK)
    uint8_t buf[2] = {};
    buf[0] = (c->channel << 2) | proto::EA | proto::CR;
    buf[1] = (c->localModemState << 1) | proto::EA;
#else
    uint8_t buf[3] = {};
    buf[0] = (c->channel << 2) | proto::EA | proto::CR;
    buf[1] = (c->localModemState << 1);
    buf[2] = proto::EA;
#endif // !defined(GSM0710_MODEM_STATUS_SEND_EMPTY_BREAK)
    return controlSend(proto::MSC, buf, sizeof(buf));
}

template<typename StreamT, typename MutexT>
inline ssize_t Muxer<StreamT, MutexT>::findCharacter(uint8_t c) {
    auto p = std::memchr(curBuf(), c, toParse());
    if (!p) {
        return GSM0710_ERROR_NOT_FOUND;
    }

    return (uint8_t*)p - (uint8_t*)curBuf();
}

template<typename StreamT, typename MutexT>
inline ssize_t Muxer<StreamT, MutexT>::parsed(size_t size) {
    GSM0710_LOG_DEBUG(TRACE, "Parsed %u bytes", size);
    inBufParserPos_ += size;
    return 0;
}

template<typename StreamT, typename MutexT>
inline size_t Muxer<StreamT, MutexT>::toParse() const {
    return inBufData_ - inBufParserPos_;
}

template<typename StreamT, typename MutexT>
inline uint8_t* Muxer<StreamT, MutexT>::curBuf() {
    return inBuf_.get() + inBufParserPos_;
}

template<typename StreamT, typename MutexT>
inline ssize_t Muxer<StreamT, MutexT>::consume(size_t size) {
    GSM0710_LOG_DEBUG(TRACE, "Consuming %u bytes", size);
    CHECK_TRUE(size > 0, 0);
    CHECK_TRUE(inBufData_ >= size, GSM0710_ERROR_INVALID_ARGUMENT);
    inBufData_ -= size;
    if (inBufData_ > 0) {
        ::memmove(inBuf_.get(), inBuf_.get() + size, inBufData_);
    }

    inBufParserPos_ -= std::min(inBufParserPos_, size);

    return inBufData_;
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::transition(State state) {
    GSM0710_LOG_DEBUG(TRACE, "Transitioning from state %s to state %s",
            stateName(state_), stateName(state));
    state_ = state;
    xEventGroupSetBits(events_, EVENT_STATE_CHANGED);
}

template<typename StreamT, typename MutexT>
inline int Muxer<StreamT, MutexT>::channelTransition(Channel* channel, ChannelState state) {
    if (channelHandler_) {
        int r = channelHandler_(channel->channel, channel->state, state, channelHandlerCtx_);
        if (!initiator_ && channel->channel != 0 && r) {
            GSM0710_LOG_DEBUG(TRACE, "Channel %u state change from %s to %s denied by channel handler",
                    channel->channel, channel->stateName(channel->state), channel->stateName(state));
            return r;
        }
    }

    channel->transition(state);
    xEventGroupSetBits(channelEvents_, EVENT_STATE_CHANGED << channel->channel);

    if (channel->channel == 0 && state == ChannelState::Error) {
        forceClose();
    }

    return 0;
}

template<typename StreamT, typename MutexT>
inline const char* Muxer<StreamT, MutexT>::stateName(State state) {
    static const char* stateNames[] = {
        "Stopped",
        "Stopping",
        "Starting",
        "Idle",
        "Flag",
        "Address",
        "Control",
        "Length1",
        "Length2",
        "Data",
        "Fcs",
        "Error"
    };
    return stateNames[static_cast<typename std::underlying_type<State>::type>(state)];
}

// Muxer::Channel
template<typename StreamT, typename MutexT>
Muxer<StreamT, MutexT>::Channel::Channel()
        : state(ChannelState::Closed) {
    reset();
}

template<typename StreamT, typename MutexT>
inline const char* Muxer<StreamT, MutexT>::Channel::stateName(ChannelState state) {
    static const char* stateNames[] = {
        "Closed",
        "Opened",
        "Opening",
        "Closing",
        "Error"
    };
    return stateNames[static_cast<typename std::underlying_type<ChannelState>::type>(state)];
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::Channel::transition(ChannelState nState) {
    GSM0710_LOG_DEBUG(TRACE, "Channel %u transitioning from %s to %s",
            channel, stateName(state), stateName(nState));
    state = nState;
    if (nState == ChannelState::Closed) {
        reset();
    }
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::Channel::reset() {
    state = ChannelState::Closed;
    handler = nullptr;
    localModemState = (proto::RTR | proto::RTC);
    remoteModemState = (proto::RTR | proto::RTC);
    update = false;
    convergence = proto::UNSTRUCTURED;
    timestamp = 0;
    retries = 0;
}

template<typename StreamT, typename MutexT>
inline Muxer<StreamT, MutexT>::ControlCommand::ControlCommand() {
    reset();
}

template<typename StreamT, typename MutexT>
inline void Muxer<StreamT, MutexT>::ControlCommand::reset() {
    command = 0;
    len = 0;
    timestamp = 0;
    retries = 0;
    state = State::None;
}

template<typename StreamT, typename MutexT>
inline const char* Muxer<StreamT, MutexT>::ControlCommand::stateName(typename Muxer<StreamT, MutexT>::ControlCommand::State state) {
    static const char* stateNames[] = {
        "None",
        "Pending",
        "Timeout",
        "NonSupported"
    };
    return stateNames[static_cast<typename std::underlying_type<Muxer<StreamT, MutexT>::ControlCommand::State>::type>(state)];
}

} // gsm0710

#endif // GSM0710_MUXER_IMPL_H
