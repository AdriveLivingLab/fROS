#include "rclcpp/rclcpp.hpp"

class FRosNode : public rclcpp::Node
{
public:
    FRosNode()
        : Node("f_ros_node"), count_(0)
    {
        // Port Number of gateway device
        this->declare_parameter("port_num_gw", rclcpp::ParameterType::PARAMETER_STRING);
        const char *port_num_gw = this->get_parameter("port_num_gw").get_parameter_value().get<std::string>().c_str();
        const unsigned short port_gw = static_cast<unsigned short>(std::strtoul(port_num_gw, NULL, 0));
        // IP Address of gateway device
        this->declare_parameter("ip_address_gw", rclcpp::ParameterType::PARAMETER_STRING);
        const std::string ip_adress_gw = this->get_parameter("ip_adress_gw").get_parameter_value().get<std::string>();
        const boost::asio::ip::address address_gw = boost::asio::ip::address::from_string(ip_adress_gw);

        std::cout << "Creating service and opening socket" << std::endl;
        // service creation
        boost::asio::io_service io_service;
        // socket creation
        socket = new tcp::socket(io_service);
        // connection
        socket->connect(tcp::endpoint(address_gw, port_gw));

        std::cout << "Trying to initiate connection" << std::endl;

        boost::system::error_code error_init;
        boost::asio::write(socket, boost::asio::buffer(send_init), error_init);
        !error_init ? std::cout << "Client sent init command!" << std::endl : std::cout << "Init sent failed: " << error_init.message() << std::endl;

        boost::system::error_code error_start;
        boost::asio::write(socket, boost::asio::buffer(send_start), error_start);
        !error_start ? std::cout << "Client sent start command!" << std::endl : std::cout << "Start sent failed: " << error_start.message() << std::endl;

        std::cout << "Starting publishers" << std::endl;

        // TODO: needs to be adapted to ROS 2
        // Publisher inits. Paste publishers.txt from c-coderdbc here!
        // example:
        // ::FROS:: PASTE YOUR PUBLISHERS HERE
        // publisher_ = this->create_publisher<f_ros_msgs::msg::Frame1>("~/frame_1", 1);

        /***************************************
         * Connection
         ***************************************/
        /* Get the first packets and throw them away, as the first messages received contain help text*/
        boost::asio::streambuf receive_buffer;
        boost::system::error_code error_receive;
        for (int i = 0; i <= 4; i++)
        {
            boost::asio::read(socket, receive_buffer, boost::asio::transfer_at_least(1), error_receive);
        }

        /***************************************
         * Allocation for message decoding
         ***************************************/
        int iSockRLen = 0;
        IXXAT_BIN_PKT *pPkt;

        // App structure buffer
        pApp = (APP_IXX_PP *)calloc(sizeof(APP_IXX_PP), 1);
        if (pApp == NULL)
        {
            perror("APP Buffer allocation");
            exit;
        }

        // Socket packets buffer
        pApp->pSBuf = (char *)malloc(512 * 3);
        if (pApp->pSBuf == NULL)
        {
            perror("Buffer allocation");
            exit;
        }
        std::cout << "--" << std::endl;
        std::cout << "Message decoding active" << std::endl;

        // convert from Hz to ms
        const std::chrono::milliseconds interval = std::chrono::milliseconds(static_cast<int>(1.0 / static_cast<double>(loopSpeed)));
        timer_ = this->create_wall_timer(interval, std::bind(&FRosNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        boost::asio::streambuf receive_buffer;
        boost::system::error_code error_receive;
        // Read packet
        pApp->iSLen = boost::asio::read(socket, receive_buffer, boost::asio::transfer_at_least(1), error_receive);

        if (error_receive && error_receive != boost::asio::error::eof)
        {
            std::cout << "receive failed: " << error_receive.message() << std::endl;
        }
        else
        {
            // Receive packet
            const char *data = boost::asio::buffer_cast<const char *>(receive_buffer.data());

            // Append packet to buffer
            memcpy(pApp->pSBuf + pApp->iRByt, data, pApp->iSLen);
            // start of analysis from index 0
            pApp->iSIdx = 0;
            pApp->iSLen += pApp->iRByt;

            // loop until we parsed the whole buffer or we reach an incomplete message
            while (pApp->iSIdx < (pApp->iSLen))
            {
                // try to extract message
                int retVal = exIxxPkt(pApp);

                if (retVal == 0) // message found and message is complete
                {
                    // search index for next search to next position
                    pApp->iSIdx = pApp->iNIdx;

                    // Call the right function according to the ID to decode the binary string into signals
                    // Paste evaluation.txt from c-coderdbc here!
                    // example:

                    // needs to be adapted to ROS 2
                    // ::FROS:: PASTE YOUR MSG HERE
                    // if (pApp->pPkt->dw_id == 1)
                    // {
                    //   f_ros_msgs::msg::Frame1 *decodedMsg;
                    //   Frame_1_raw *rawMsg = (FRAME_1_raw *)(pApp->pPkt->ab_data);
                    //   decodedMsg = Frame_1_APP(rawMsg);
                    //   publisher_->publish(*decodedMsg);
                    //   free(pApp->pPkt);
                    //   free(decodedMsg);
                    // }
                }
                else // end of buffer reached, last message is incomplete
                {
                    // copy the packet reminder into buffer
                    memcpy(pApp->pSBuf, (pApp->pSBuf + pApp->iNIdx), pApp->iRByt);
                    break;
                }
            }
        }

        if (!rclcpp::ok())
        {
            boost::system::error_code error_stop;
            boost::asio::write(socket, boost::asio::buffer(send_stop), error_stop);
            !error_stop ? std::cout << "Client sent stop command!" << std::endl : std::cout << "Stop sent failed: " << error_stop.message() << std::endl;
            // Close the TCP socket, so the gateway can accept a new connection in the future
            socket->close();
            exit(0);
        }
    }

    const std::string send_init = "c init 500\n\r";
    const std::string send_start = "c start\n\r";
    const std::string send_stop = "c stop\n\r";
    APP_IXX_PP *pApp;
    tcp::socket *socket;
    rclcpp::TimerBase::SharedPtr timer_;
    // TODO: needs to be adapted to ROS 2
    // Publisher inits. Paste publishers.txt from c-coderdbc here!
    // example:
    // ::FROS:: PASTE YOUR PUBLISHERS HERE
    // rclcpp::Publisher<f_ros_msgs::msg::Frame1>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FRosNode>());
    rclcpp::shutdown();
    return 0;
}
