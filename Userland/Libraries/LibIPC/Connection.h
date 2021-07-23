/*
 * Copyright (c) 2018-2020, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/ByteBuffer.h>
#include <AK/NonnullOwnPtrVector.h>
#include <LibCore/Event.h>
#include <LibCore/EventLoop.h>
#include <LibCore/LocalSocket.h>
#include <LibCore/Notifier.h>
#include <LibCore/TCPSocket.h>
#include <LibCore/Timer.h>
#include <LibIPC/Message.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace IPC {

template<typename SocketType>
struct SocketConnectionSendFd {
};

template<>
struct SocketConnectionSendFd<Core::TCPSocket> {
    static constexpr bool can_send_or_receive_fds = false;
    static bool send_fd(int, int)
    {
        VERIFY_NOT_REACHED(); // Can't send FDs over TCP!
    }
};

template<>
struct SocketConnectionSendFd<Core::LocalSocket> {
    static constexpr bool can_send_or_receive_fds = true;
    static bool send_fd([[maybe_unused]] int socket_fd, [[maybe_unused]] int fd)
    {
#ifdef __serenity__
        auto rc = sendfd(socket_fd, fd);
        if (rc < 0) {
            perror("sendfd");
            return false;
        }
        return true;
#else
        warnln("fd passing is not supported on this platform, sorry :(");
        return false;
#endif
    }
};

template<typename Handler, typename LocalEndpoint, typename PeerEndpoint>
class RawConnection {
public:
    RawConnection(Handler& handler)
        : m_handler(handler)
    {
    }

    bool received_bytes(ReadonlyBytes const& bytes, int socket_fd)
    {
        m_unprocessed_bytes.append(bytes);

        size_t index = 0;
        u32 message_size = 0;
        for (; index + sizeof(message_size) < m_unprocessed_bytes.size(); index += message_size) {
            memcpy(&message_size, m_unprocessed_bytes.data() + index, sizeof(message_size));
            if (message_size == 0 || m_unprocessed_bytes.size() - index - sizeof(uint32_t) < message_size)
                break;
            index += sizeof(message_size);
            auto remaining_bytes = ReadonlyBytes { m_unprocessed_bytes.data() + index, message_size };
            if (auto message = LocalEndpoint::decode_message(remaining_bytes, socket_fd)) {
                m_handler.handle_raw_message(message.release_nonnull(), remaining_bytes, false);
            } else if (auto message = PeerEndpoint::decode_message(remaining_bytes, socket_fd)) {
                m_handler.handle_raw_message(message.release_nonnull(), remaining_bytes, true);
            } else {
                dbgln("RawConnection: Failed to parse a message, size: {}", message_size);
                break;
            }
        }

        if (index < m_unprocessed_bytes.size()) {
            // Sometimes we might receive a partial message. That's okay, just stash away
            // the unprocessed bytes and we'll prepend them to the next incoming message
            // in the next run of this function.
            m_unprocessed_bytes = ByteBuffer::copy(m_unprocessed_bytes.data() + index, m_unprocessed_bytes.size() - index);
        } else {
            m_unprocessed_bytes.clear();
        }

        return true;
    }

    bool is_empty() const { return m_unprocessed_bytes.is_empty(); }

private:
    Handler& m_handler;
    ByteBuffer m_unprocessed_bytes;
};

template<typename LocalEndpoint, typename PeerEndpoint, typename LocalStub = typename LocalEndpoint::Stub, typename SocketType = Core::LocalSocket>
class Connection : public Core::Object {
public:
    using RawConnectionType = typename IPC::RawConnection<Connection, LocalEndpoint, PeerEndpoint>;
    template<typename RawHandler, typename RawLocalEndpoint, typename RawPeerEndpoint>
    friend class IPC::RawConnection;

    Connection(LocalStub& local_stub, NonnullRefPtr<SocketType> socket)
        : m_local_stub(local_stub)
        , m_raw_connection(*this)
        , m_socket(move(socket))
        , m_notifier(Core::Notifier::construct(m_socket->fd(), Core::Notifier::Read, this))
    {
        m_responsiveness_timer = Core::Timer::create_single_shot(3000, [this] { may_have_become_unresponsive(); });
        m_notifier->on_ready_to_read = [this] {
            NonnullRefPtr protect = *this;
            drain_messages_from_peer();
            handle_messages();
        };
    }

    template<typename MessageType>
    OwnPtr<MessageType> wait_for_specific_message()
    {
        return wait_for_specific_endpoint_message<MessageType, LocalEndpoint>();
    }

    void post_message(const Message& message)
    {
        post_message(message.encode());
    }

    // FIXME: unnecessary copy
    void post_message(MessageBuffer buffer)
    {
        // NOTE: If this connection is being shut down, but has not yet been destroyed,
        //       the socket will be closed. Don't try to send more messages.
        if (!m_socket->is_open())
            return;

        // Prepend the message size.
        uint32_t message_size = buffer.data.size();
        buffer.data.prepend(reinterpret_cast<const u8*>(&message_size), sizeof(message_size));

        for (auto& fd : buffer.fds) {
            if (!SocketConnectionSendFd<SocketType>::send_fd(m_socket->fd(), fd->value())) {
                shutdown();
                break;
            }
        }

        size_t total_nwritten = 0;
        while (total_nwritten < buffer.data.size()) {
            if (m_buffer_outgoing) {
                auto available = m_send_buffer.capacity() - m_send_buffer.size();
                if (available == 0) {
                    if (!flush_send_buffer())
                        return;
                    continue;
                }
                auto need = buffer.data.size() - total_nwritten;
                auto write_bytes = min(need, available);
                m_send_buffer.append(buffer.data.data() + total_nwritten, write_bytes);
                total_nwritten += write_bytes;
                if (need >= available) {
                    if (!flush_send_buffer())
                        return;
                    continue;
                } else if (!buffer.fds.is_empty()) {
                    if (!flush_send_buffer())
                        return;
                }
                break;
            } else if (!m_send_buffer.is_empty()) {
                if (!flush_send_buffer())
                    return;
            }

            auto nwritten = write(m_socket->fd(), buffer.data.data() + total_nwritten, buffer.data.size() - total_nwritten);
            if (nwritten < 0) {
                switch (errno) {
                case EPIPE:
                    dbgln("RawConnection {:p} post_message: Disconnected from peer", this);
                    shutdown();
                    return;
                case EAGAIN:
                    dbgln("RawConnection {:p} post_message: Peer buffer overflowed", this);
                    shutdown();
                    return;
                default:
                    perror("RawConnection::post_message write");
                    shutdown();
                    return;
                }
            }
            m_bytes_sent += (size_t)nwritten;
            total_nwritten += nwritten;
        }

        m_responsiveness_timer->start();
    }

    template<typename RequestType, typename... Args>
    NonnullOwnPtr<typename RequestType::ResponseType> send_sync(Args&&... args)
    {
        post_message(RequestType(forward<Args>(args)...));
        auto response = wait_for_specific_endpoint_message<typename RequestType::ResponseType, PeerEndpoint>();
        VERIFY(response);
        return response.release_nonnull();
    }

    template<typename RequestType, typename... Args>
    OwnPtr<typename RequestType::ResponseType> send_sync_but_allow_failure(Args&&... args)
    {
        post_message(RequestType(forward<Args>(args)...));
        return wait_for_specific_endpoint_message<typename RequestType::ResponseType, PeerEndpoint>();
    }

    virtual void may_have_become_unresponsive() { }
    virtual void did_become_responsive() { }

    void shutdown()
    {
        m_notifier->close();
        m_socket->close();
        if (on_disconnect)
            on_disconnect();
        die();
    }

    virtual void die() { }

    bool is_open() const { return m_socket->is_open(); }

    u64 bytes_sent() const { return m_bytes_sent; }
    u64 bytes_received() const { return m_bytes_received; }

    Function<void()> on_disconnect;
    Function<void()> on_idle;
    Function<bool(bool, ReadonlyBytes const&)> on_handle_raw_message;

    SocketType& socket() { return *m_socket; }
    SocketType const& socket() const { return *m_socket; }
    bool is_connected() const { return m_socket->is_connected(); }

protected:
    virtual void handle_raw_message(NonnullOwnPtr<IPC::Message>&& message, ReadonlyBytes const& bytes, bool is_peer)
    {
        if (on_handle_raw_message && !on_handle_raw_message(is_peer, bytes))
            return;
        bool was_empty = m_unprocessed_messages.is_empty();
        m_unprocessed_messages.append(move(message));
        if (was_empty) {
            deferred_invoke([this](auto&) {
                handle_messages();
            });
        }
    }

    template<typename MessageType, typename Endpoint>
    OwnPtr<MessageType> wait_for_specific_endpoint_message()
    {
        for (;;) {
            if (!m_send_buffer.is_empty()) {
                if (!flush_send_buffer())
                    break;
            }

            // Double check we don't already have the event waiting for us.
            // Otherwise we might end up blocked for a while for no reason.
            for (size_t i = 0; i < m_unprocessed_messages.size(); ++i) {
                auto& message = m_unprocessed_messages[i];
                if (message.endpoint_magic() != Endpoint::static_magic())
                    continue;
                if (message.message_id() == MessageType::static_message_id())
                    return m_unprocessed_messages.take(i).template release_nonnull<MessageType>();
            }

            if (!m_socket->is_open())
                break;
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(m_socket->fd(), &rfds);
            for (;;) {
                if (auto rc = select(m_socket->fd() + 1, &rfds, nullptr, nullptr, nullptr); rc < 0) {
                    if (errno == EINTR)
                        continue;
                    perror("wait_for_specific_endpoint_message: select");
                    VERIFY_NOT_REACHED();
                } else {
                    VERIFY(rc > 0);
                    VERIFY(FD_ISSET(m_socket->fd(), &rfds));
                    break;
                }
            }

            if (!drain_messages_from_peer())
                break;
        }
        return {};
    }

    bool drain_messages_from_peer()
    {
        while (m_socket->is_open()) {
            u8 buffer[4096];
            ssize_t nread = recv(m_socket->fd(), buffer, sizeof(buffer), MSG_DONTWAIT);
            if (nread < 0) {
                if (errno == EAGAIN)
                    break;
                perror("recv");
                exit(1);
                return false;
            }
            if (nread == 0) {
                if (m_raw_connection.is_empty()) {
                    deferred_invoke([this](auto&) { shutdown(); });
                    return false;
                }
                break;
            }
            m_bytes_received += (size_t)nread;
            m_raw_connection.received_bytes(ReadonlyBytes { buffer, (size_t)nread }, SocketConnectionSendFd<SocketType>::can_send_or_receive_fds ? m_socket->fd() : -1);
        }

        if (!m_raw_connection.is_empty()) {
            m_responsiveness_timer->stop();
            did_become_responsive();
        } else {
            notify_if_idle();
        }
        return true;
    }

    void handle_messages()
    {
        auto messages = move(m_unprocessed_messages);
        for (auto& message : messages) {
            if (message.endpoint_magic() == LocalEndpoint::static_magic())
                if (auto response = m_local_stub.handle(message))
                    post_message(*response);
        }
        notify_if_idle();
    }

    void notify_if_idle()
    {
        if (!m_unprocessed_messages.is_empty() || !m_unprocessed_bytes.is_empty())
            return;
        if (on_idle)
            on_idle();
    }

    bool flush_send_buffer()
    {
        if (m_send_buffer.is_empty())
            return true;

        bool was_blocking = socket().set_blocking(true);
        size_t total_nwritten = 0;
        while (total_nwritten < m_send_buffer.size()) {
            auto nwritten = write(m_socket->fd(), m_send_buffer.data() + total_nwritten, m_send_buffer.size() - total_nwritten);
            if (nwritten < 0) {
                switch (errno) {
                case EPIPE:
                    dbgln("Connection {:p} flush_send_buffer: Disconnected from peer", this);
                    shutdown();
                    return false;
                case EAGAIN:
                    dbgln("Connection {:p} flush_send_buffer: Peer buffer overflowed", this);
                    shutdown();
                    return false;
                default:
                    perror("Connection::flush_send_buffer write");
                    shutdown();
                    return false;
                }
            }
            m_bytes_sent += (size_t)nwritten;
            total_nwritten += (size_t)nwritten;
        }
        if (!was_blocking)
            socket().set_blocking(false);

        if (m_buffer_outgoing)
            m_send_buffer.clear_with_capacity();
        else
            m_send_buffer.clear();
        return true;
    }

    void enable_send_buffer(size_t size)
    {
        VERIFY(size > 0);
        m_buffer_outgoing = true;
        m_send_buffer.ensure_capacity(size);
    }

    void disable_send_buffer()
    {
        m_buffer_outgoing = false;
        flush_send_buffer();
    }

protected:
    LocalStub& m_local_stub;
    RawConnectionType m_raw_connection;
    NonnullRefPtr<SocketType> m_socket;
    RefPtr<Core::Timer> m_responsiveness_timer;

    RefPtr<Core::Notifier> m_notifier;
    NonnullOwnPtrVector<Message> m_unprocessed_messages;
    ByteBuffer m_unprocessed_bytes;

    u64 m_bytes_received { 0 };
    u64 m_bytes_sent { 0 };

    Vector<u8> m_send_buffer;
    bool m_buffer_outgoing { false };
};

}

template<typename LocalEndpoint, typename PeerEndpoint>
struct AK::Formatter<IPC::Connection<LocalEndpoint, PeerEndpoint>> : Formatter<Core::Object> {
};
