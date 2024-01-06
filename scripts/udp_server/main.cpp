#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <ctime>
#include <iostream>
#include <memory>

using boost::asio::ip::udp;

std::string make_daytime_string()
{
    using namespace std;
    time_t now = time(0);
    return ctime(&now);
}

class UDPServer {
public:
    UDPServer(boost::asio::io_context& io_context,
        const char* local_ip, const char* local_port,
        const char* remote_ip, const char* remote_port)
    {
        udp::resolver resolver(io_context);
        local_endpoint_ = std::make_shared<udp::endpoint>(*resolver.resolve(
            udp::resolver::query(udp::v4(), local_ip, local_port)));
        std::cout << "local endpoint: "
                  << local_endpoint_->address().to_string()
                  << ": " << local_endpoint_->port()
                  << std::endl;
        remote_endpoint_ = std::make_shared<udp::endpoint>(*resolver.resolve(
            udp::resolver::query(udp::v4(), remote_ip, remote_port)));
        std::cout << "remote endpoint: "
                  << remote_endpoint_->address().to_string()
                  << ": " << remote_endpoint_->port()
                  << std::endl;
        socket_ = std::make_shared<udp::socket>(io_context, *local_endpoint_);
        do_receive();
    }

private:
    void handle_receive(boost::system::error_code ec, std::size_t bytes_received)
    {
        std::cout << local_endpoint_->address().to_string()
                  << ":" << local_endpoint_->port() << ": received: " << bytes_received << std::endl;
        if (!ec && bytes_received > 0) {
            do_send(bytes_received);
        } else {
            do_receive();
        }
    };

    void handle_send(boost::system::error_code ec, std::size_t bytes_sent)
    {
        std::cout << remote_endpoint_->address().to_string()
                  << ":" << remote_endpoint_->port() << ": sent: " << bytes_sent << std::endl;
        (void)ec;
        (void)bytes_sent;
        do_receive();
    };

    void do_receive()
    {
        socket_->async_receive_from(
            boost::asio::buffer(data_, max_length), *local_endpoint_,
            std::bind(&UDPServer::handle_receive, this, std::placeholders::_1, std::placeholders::_2));
    }

    void do_send(size_t bytes_received)
    {
        socket_->async_send_to(
            boost::asio::buffer(data_, bytes_received), *remote_endpoint_,
            std::bind(&UDPServer::handle_send, this, std::placeholders::_1, std::placeholders::_2));
    }

    std::shared_ptr<udp::socket> socket_;
    std::shared_ptr<udp::endpoint> local_endpoint_;
    std::shared_ptr<udp::endpoint> remote_endpoint_;
    enum { max_length = 4096 };
    char data_[max_length];
};

int main()
{
    try {
        boost::asio::io_context io_context;
        const char* local_port = "4242";
        const char* local_ip = "192.0.2.2";
        const char* remote_port = "4242";
        const char* remote_ip = "192.0.2.1";
        UDPServer server(io_context, local_ip, local_port, remote_ip, remote_port);
        std::cout << "running" << std::endl;
        io_context.run();
        std::cout << "finished" << std::endl;
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
};

// vi: ts=4 sw=4 et
