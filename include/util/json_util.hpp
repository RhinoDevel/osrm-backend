#ifndef JSON_UTIL_HPP
#define JSON_UTIL_HPP

#include "osrm/json_container.hpp"

#include <cmath>
#include <limits>

namespace osrm
{
namespace json
{

// Make sure we don't have inf and NaN values
template <typename T> T clamp_float(T d)
{
    if (std::isnan(d) || std::numeric_limits<T>::infinity() == d)
    {
        return std::numeric_limits<T>::max();
    }
    if (-std::numeric_limits<T>::infinity() == d)
    {
        return std::numeric_limits<T>::lowest();
    }

    return d;
}

template <typename... Args> osrm::json::Array make_array(Args... args)
{
    osrm::json::Array a;
    append_to_container(a.values, args...);
    return a;
}

template <typename T> osrm::json::Array make_array(const std::vector<T> &vector)
{
    osrm::json::Array a;
    for (const auto &v : vector)
    {
        a.values.emplace_back(v);
    }
    return a;
}

// template specialization needed as clang does not play nice
template <> osrm::json::Array make_array(const std::vector<bool> &vector)
{
    osrm::json::Array a;
    for (const bool v : vector)
    {
        a.values.emplace_back(v);
    }
    return a;
}

// Easy acces to object hierachies
osrm::json::Value &get(osrm::json::Value &value) { return value; }

template <typename... Keys>
osrm::json::Value &get(osrm::json::Value &value, const char *key, Keys... keys)
{
    using recursive_object_t = mapbox::util::recursive_wrapper<osrm::json::Object>;
    return get(value.get<recursive_object_t>().get().values[key], keys...);
}

template <typename... Keys>
osrm::json::Value &get(osrm::json::Value &value, unsigned key, Keys... keys)
{
    using recursive_array_t = mapbox::util::recursive_wrapper<osrm::json::Array>;
    return get(value.get<recursive_array_t>().get().values[key], keys...);
}

} // namespace json
} // namespace osrm
#endif // JSON_UTIL_HPP
