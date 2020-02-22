/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
#include "server.h"
#include "client.h"

#include <memory>
#include <functional>
#include <cstdint>
#include <signal.h>
#include <gflags/gflags.h>
#include <boost/bind.hpp>
#include "cyber/common/log.h"

DEFINE_int32(port, 9090, "tcp listen port");

Server::Server(Node& node)
    : node(node)
    , signals(io)
    , endpoint(boost::asio::ip::tcp::v4(), (uint16_t)FLAGS_port)
    , acceptor(io, endpoint)
    , socket(io)
{
AINFO<<"(DMCZP) EnteringMethod: DEFINE_int32";
AINFO<<"(DMCZP) EnteringMethod: Server::Server";
AINFO<<"(DMCZP) EnteringMethod: Server::run";
AINFO<<"(DMCZP) EnteringMethod: Server::stop";
AINFO<<"(DMCZP) EnteringMethod: Server::begin_accept";
AINFO<<"(DMCZP) EnteringMethod: Server::end_accept";
    signals.add(SIGTERM);
    signals.add(SIGINT);
}

Server::~Server()
{
}

void Server::run()
{
    signals.async_wait(boost::bind(
        &Server::stop,
        shared_from_this(),
        boost::asio::placeholders::error,
        boost::asio::placeholders::signal_number));

    begin_accept();

    io.run();
}

void Server::stop(const boost::system::error_code& ec, int signal_number)
{
    if (ec)
    {
        AERROR << "Error waiting on signals: " << ec.message();
    }
    else
    {
        AINFO << "Signal " << signal_number << " received, stopping server";
    }
    acceptor.close();
    clients.stop_all();
}

void Server::begin_accept()
{
    acceptor.async_accept(
        socket,
        boost::bind(
            &Server::end_accept,
            shared_from_this(),
            boost::asio::placeholders::error));
}

void Server::end_accept(const boost::system::error_code& ec)
{
    if (!acceptor.is_open())
    {
        return;
    }

    if (ec)
    {
        AERROR << "Error accepting connection: " << ec.message();
    }
    else
    {
        auto client = std::make_shared<Client>(node, clients, std::move(socket));
        clients.start(client);
    }

    begin_accept();
}
