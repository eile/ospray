#pragma once
#include <http-parser/http_parser.h>
#include <boost/asio.hpp>

#include <iostream>
#include <unordered_map>

namespace {
typedef std::unordered_map<std::string, std::string> HeadersMap;

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
      reinterpret_cast<Connection *>(parser->data)->_onRequest();
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

  void readRequest()
  {
    if (!_live) {
      std::cerr << "Deleted connection" << std::endl;
      return;
    }

    http_parser_init(&_httpParser, HTTP_REQUEST);
    _readRequest(boost::system::error_code(), 0);
  }

  void writeRequest(const std::string &uri)
  { // parse uri into host, url, parameters
    // send request
    // collect response
    // call onResponse()
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
  virtual void onRequest() = 0;

 private:
  void _readRequest(const boost::system::error_code &err, const size_t len)
  {
    if (!!err) {
      shutdown();
      return;
    }

    const auto parsed =
        http_parser_execute(&_httpParser, &_httpSettings, _buffer.data(), len);

    if (parsed != len || !_live) {
      delete this;
      return;
    }

    async_read(_socket,
        boost::asio::buffer(_buffer.data(), _buffer.size()),
        boost::asio::transfer_at_least(1),
        [this](const boost::system::error_code &err, const size_t len) {
          this->_readRequest(err, len);
        });
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
    _body = std::string(at, len);
  }

  void _onRequest()
  {
    onRequest();
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

  std::array<char, 8192> _buffer;

  std::string _currentHeader;

  std::string _url;
  HeadersMap _headers;
  std::string _body;

  http_parser _httpParser;
  http_parser_settings _httpSettings;
};
} // namespace