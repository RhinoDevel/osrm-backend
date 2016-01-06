#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#include "server/http/compression_type.hpp"
#include "server/http/reply.hpp"
#include "server/http/request.hpp"
#include "server/request_parser.hpp"

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/config.hpp>
#include <boost/version.hpp>

#include <memory>
#include <vector>

namespace osrm
{
namespace server
{

// workaround for incomplete std::shared_ptr compatibility in old boost versions
#if BOOST_VERSION < 105300 || defined BOOST_NO_CXX11_SMART_PTR

namespace boost
{
template <class T> const T *get_pointer(std::shared_ptr<T> const &p) { return p.get(); }

template <class T> T *get_pointer(std::shared_ptr<T> &p) { return p.get(); }
} // namespace boost

#endif

class RequestHandler;

namespace http
{

/// Represents a single connection from a client.
class Connection : public std::enable_shared_from_this<Connection>
{
  public:
    explicit Connection(boost::asio::io_service &io_service, RequestHandler &handler);
    Connection(const Connection &) = delete;
    Connection() = delete;

    boost::asio::ip::tcp::socket &socket();

    /// Start the first asynchronous operation for the connection.
    void start();

  private:
    void handle_read(const boost::system::error_code &e, std::size_t bytes_transferred);

    /// Handle completion of a write operation.
    void handle_write(const boost::system::error_code &e);

    std::vector<char> compress_buffers(const std::vector<char> &uncompressed_data,
                                       const compression_type compression_type);

    boost::asio::io_service::strand strand;
    boost::asio::ip::tcp::socket TCP_socket;
    RequestHandler &request_handler;
    RequestParser request_parser;
    boost::array<char, 8192> incoming_data_buffer;
    http::request current_request;
    reply current_reply;
    std::vector<char> compressed_output;
};

} // namespace http

}
}

#endif // CONNECTION_HPP
