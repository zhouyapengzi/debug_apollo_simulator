#include "cyber/common/log.h"
/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */
#include "clients.h"
#include "client.h"

Clients::Clients()
{
AINFO<<"(DMCZP) EnteringMethod: Clients::Clients";
AINFO<<"(DMCZP) EnteringMethod: Clients::start";
AINFO<<"(DMCZP) EnteringMethod: Clients::stop";
AINFO<<"(DMCZP) EnteringMethod: Clients::stop_all";
AINFO<<"(DMCZP) EnteringMethod: Clients::Clients";
AINFO<<"(DMCZP) EnteringMethod: Clients::start";
AINFO<<"(DMCZP) EnteringMethod: Clients::stop";
AINFO<<"(DMCZP) EnteringMethod: Clients::stop_all";
}

Clients::~Clients()
{
}

void Clients::start(std::shared_ptr<Client> client)
{
    clients.insert(client);
    client->start();
}

void Clients::stop(std::shared_ptr<Client> client)
{
    clients.erase(client);
    client->stop();
}

void Clients::stop_all()
{
    for (auto& client : clients)
    {
        client->stop();
    }
    clients.clear();
}
