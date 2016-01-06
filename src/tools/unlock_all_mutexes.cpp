#include "util/version.hpp"
#include "util/simple_logger.hpp"
#include "engine/datafacade/shared_barriers.hpp"

#include <iostream>

namespace osrm
{
namespace tools
{

int main()
{
    util::LogPolicy::GetInstance().Unmute();
    try
    {
        util::SimpleLogger().Write() << "starting up engines, " << OSRM_VERSION;
        util::SimpleLogger().Write() << "Releasing all locks";
        SharedBarriers barrier;
        barrier.pending_update_mutex.unlock();
        barrier.query_mutex.unlock();
        barrier.update_mutex.unlock();
    }
    catch (const std::exception &e)
    {
        util::SimpleLogger().Write(logWARNING) << "[excpetion] " << e.what();
    }
    return 0;
}
}
}
