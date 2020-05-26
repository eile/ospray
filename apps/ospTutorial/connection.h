#pragma once
#include <http-parser/http_parser.h>
#include <boost/asio.hpp>

#include <iostream>
#include <unordered_map>

namespace {
typedef std::unordered_map<std::string, std::string> HeadersMap;

/**
 * Incoming and outgoing async http connection.
 * Can only be used once to read() or write(). Self-destructs after onData()
 * return.
 */
class Connection
{
 public:
  Connection(boost::asio::io_service &ioService)
      : _ioService(ioService), _socket(ioService), _resolver(ioService)
  {
    http_parser_settings_init(&_httpSettings);

    _httpParser.data = this;
    _httpSettings.on_url = [](http_parser *parser, const char *at, size_t len) {
      reinterpret_cast<Connection *>(parser->data)->_onURL(at, len);
      return 0;
    };
    _httpSettings.on_header_field =
        [](http_parser *parser, const char *at, size_t len) {
          reinterpret_cast<Connection *>(parser->data)->_onHeaderField(at, len);
          return 0;
        };

    _httpSettings.on_header_value =
        [](http_parser *parser, const char *at, size_t len) {
          reinterpret_cast<Connection *>(parser->data)->_onHeaderValue(at, len);
          return 0;
        };

    _httpSettings.on_body =
        [](http_parser *parser, const char *at, size_t len) {
          reinterpret_cast<Connection *>(parser->data)->_onBody(at, len);
          return 0;
        };

    _httpSettings.on_message_complete = [](http_parser *parser) {
      reinterpret_cast<Connection *>(parser->data)->_onData();
      return 0;
    };
  }

  virtual ~Connection()
  {
    shutdown();
  }

  boost::asio::ip::tcp::socket &socket()
  {
    return _socket;
  }

  void read()
  {
    if (!_live) {
      std::cerr << "Deleted connection" << std::endl;
      return;
    }

    http_parser_init(&_httpParser, HTTP_REQUEST);
    _read(boost::system::error_code(), 0);
  }

  void write(const std::string &server, const int port)
  {
    if (!_live) {
      std::cerr << "Deleted connection" << std::endl;
      return;
    }

    http_parser_init(&_httpParser, HTTP_RESPONSE);
    _write(server, port);
  }

  std::string &url()
  {
    return _url;
  }

  const std::string &url() const
  {
    return _url;
  }

  const std::string &body() const
  {
    return _body;
  }

  const HeadersMap &headers() const
  {
    return _headers;
  }

 protected:
  virtual void onData() = 0;

 private:
  void _read(const boost::system::error_code &error, const size_t len)
  {
    if (!_request.empty() && error == boost::asio::error::eof) {
      // completed write without content-length
      _onData();
      delete this;
      return;
    }

    if (_handleError(error))
      return;

    const auto parsed =
        http_parser_execute(&_httpParser, &_httpSettings, _buffer.data(), len);

    if (parsed != len || !_live) {
      delete this;
      return;
    }

    const auto i = _headers.find("Content-Length");
    if (!_request.empty() && i != _headers.end()
        && _body.length() >= size_t(std::stoi(i->second))) {
      // completed write with content-length
      _onData();
      delete this;
      return;
    }

    async_read(_socket,
        boost::asio::buffer(_buffer.data(), _buffer.size()),
        boost::asio::transfer_at_least(1),
        [this](const boost::system::error_code &error, const size_t len) {
          this->_read(error, len);
        });
  }

  void _write(const std::string &server, const int port)
  {
    boost::asio::ip::tcp::resolver::query query(server, std::to_string(port));
    _resolver.async_resolve(query,
        [this](const boost::system::error_code &error,
            boost::asio::ip::tcp::resolver::iterator endpoint) {
          if (_handleError(error))
            return;

          _socket.async_connect(
              *endpoint, [this](const boost::system::error_code &error) {
                if (_handleError(error))
                  return;

                _request = std::string("GET ") + _url + " HTTP/1.0\r\n\r\n";
                async_write(_socket,
                    boost::asio::buffer(_request),
                    [this](const boost::system::error_code &error,
                        const size_t len) {
                      if (_handleError(error))
                        return;

                      _read(boost::system::error_code(), 0); // response
                    });
              });
        });
  }

  bool _handleError(const boost::system::error_code &error)
  {
    if (!error)
      return false;

    std::cerr << "Network error: " << error.message() << std::endl;
    delete this;
    return true;
  }

  void _onURL(const char *at, size_t len)
  {
    _url = std::string(at, len);
  }

  void _onHeaderField(const char *at, size_t len)
  {
    _currentHeader = std::string(at, len);
  }

  void _onHeaderValue(const char *at, size_t len)
  {
    _headers[_currentHeader] = std::string(at, len);
    _currentHeader.clear();
  }

  void _onBody(const char *at, size_t len)
  {
    _body += std::string(at, len);
  }

  void _onData()
  {
    onData();
    _live = false;
  }

  void shutdown()
  {
    if (_socket.is_open()) {
      _socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
      _socket.close();
    }
  }

 private:
  boost::asio::io_service &_ioService;
  boost::asio::ip::tcp::socket _socket;
  boost::asio::ip::tcp::resolver _resolver;
  bool _live = true;

  std::string _request;
  std::array<char, 65536> _buffer;

  std::string _currentHeader;

  std::string _url;
  HeadersMap _headers;
  std::string _body;

  http_parser _httpParser;
  http_parser_settings _httpSettings;
};
} // namespace