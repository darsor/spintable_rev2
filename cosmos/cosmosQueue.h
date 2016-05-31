#ifndef COSMOS_QUEUE_H
#define COSMOS_QUEUE_H

#include "cosmos.h"
#include "packets.h"
#include <queue>

class CosmosQueue {
    public:
        CosmosQueue(int port, const unsigned int size);
        ~CosmosQueue();
        void push(Packet* item);
        bool pop();
        void deleteFront();
        unsigned int size();
        void connect();
        void disconnect();
        bool isConnected() { return connected; }
        int recv(unsigned char* buffer, int size);
    private:
        Cosmos cosmos;
        bool connected;

        std::queue<Packet*> queue;
        unsigned int capacity;
};
#endif
