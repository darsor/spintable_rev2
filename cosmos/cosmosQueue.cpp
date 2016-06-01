#include "cosmosQueue.h"
#include <mutex>

std::mutex push_mutex;
std::mutex pop_mutex;

CosmosQueue::CosmosQueue(int port, const unsigned int size):
    cosmos(port), queue() {
        connected = false;
        capacity = size;
    }

CosmosQueue::~CosmosQueue() {
    while (!queue.empty()) {
        deleteFront();
    }
    capacity = 0;
}

void CosmosQueue::deleteFront() {
    delete queue.front();
    queue.pop();
}

void CosmosQueue::push(Packet* item) {
    push_mutex.lock();
    if (queue.size() >= capacity) {
        deleteFront();
    }
    queue.push(item);
    push_mutex.unlock();
}

bool CosmosQueue::pop() {
    pop_mutex.lock();
    push_mutex.lock();
    if (queue.empty()) {
        pop_mutex.unlock();
        push_mutex.unlock();
        return false;
    }
    queue.front()->convert();
    if (cosmos.sendPacket(queue.front()->buffer, queue.front()->length) != 0) {
        connected = false;
        push_mutex.unlock();
        pop_mutex.unlock();
        return false;
    }
    deleteFront();
    push_mutex.unlock();
    pop_mutex.unlock();
    return true;
}

unsigned int CosmosQueue::size() {
    return queue.size();
}

void CosmosQueue::connect() {
    cosmos.acceptConnection();
    connected = true;
}

void CosmosQueue::disconnect() {
    cosmos.cosmosDisconnect();
}

int CosmosQueue::recv(unsigned char* buffer, int size) {
    if (cosmos.recvPacket(buffer, size) < 0) {
        connected = false;
        return -1;
    } else return 0;
}
