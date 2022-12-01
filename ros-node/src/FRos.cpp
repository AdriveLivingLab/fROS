/**
 * @file
 * @brief This is a example node to get FlexRay messages into ROS1 environment using a Ixxat FRC-EP170
 * @authors ludwig.kastner@hs-kempten.de, daniel.schneider@hs-kempten.de, krishnakumar.mayannavar@hs-kempten.de
 * @date 01.12.2022
 * */

#include "ros/ros.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../include/FRos/convert.h"
#include "../include/FRos/decode.h"

#define DEBUG false
#define PRINT false

/** \namespace BOOST TCP link*/
using boost::asio::ip::tcp;

/** \brief Length of the stream */
size_t len = 0;
/** \brief Check the timings */
bool performance_check = 0;
/** \brief speed of accessing the data */
#define loopSpeed 1000

/***************************************
 * Macros
 ***************************************/
#define PKT_MSGLEN_OFFSET (4u)
#define PKT_HEADER_LEN (21u) // with SIGN
#define PKT_SIGN_START "B "
#define PKT_SIGN_END "\r\n"
#define ASCII_SPACE (0x10)
#define ASCII_CR (0x0D)
#define ASCII_LF (0x0A)
#define ASCII_B (0x42)

/***************************************
 * Typedef
 ***************************************/
typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef unsigned long UINT64;

/***************************************
 * Structures
 ***************************************/
// IXXAT Packet header structure
typedef struct __attribute__((__packed__)) pkt
{
  UINT16 w_msgSign;     // B space
  UINT16 w_msglen;      // message complete length (incl. msglen)
  UINT32 dw_id;         // message ID
  UINT8 b_format;       // message frame format information
  UINT64 qw_timestamp;  // message timestamp
  UINT16 w_reserved;    // reserved data for future use
  UINT16 w_datalen;     // num of data bytes (length of ab_data)
  UINT8 ab_data[];      // data bytes
} IXXAT_BIN_PKT;

// App structure
typedef struct app_pkt_parse
{
  char *pSBuf;          // Socket Received data buffer
  UINT32 iSLen;         // Socket Received data length
  UINT32 iSIdx;         // Start index
  UINT32 iNIdx;         // Next: End + 1 index
  int iRByt;            // Remaining bytes
  IXXAT_BIN_PKT *pPkt;  // Packet buffer pointer
} APP_IXX_PP;

/***************************************
 * Functions
 ***************************************/
// Swap 2 bytes
UINT16 swapOrderUINT16(UINT16 us)
{
  us = (us >> 8) |
       (us << 8);

  return us;
}

// Swap 4 bytes
UINT32 swapOrderUINT32(UINT32 ui)
{
  ui = (ui >> 24) |
       ((ui << 8) & 0x00FF0000) |
       ((ui >> 8) & 0x0000FF00) |
       (ui << 24);

  return ui;
}

// Swap 8 bytes
UINT64 swapOrderUINT64(UINT64 ull)
{
  ull = (ull >> 56) |
        ((ull << 40) & 0x00FF000000000000) |
        ((ull << 24) & 0x0000FF0000000000) |
        ((ull << 8) & 0x000000FF00000000) |
        ((ull >> 8) & 0x00000000FF000000) |
        ((ull >> 24) & 0x0000000000FF0000) |
        ((ull >> 40) & 0x000000000000FF00) |
        (ull << 56);

  return ull;
}

// Function to get message from buffer
int exIxxPkt(APP_IXX_PP *pApp)
{
  char *pBuf = pApp->pSBuf;
  IXXAT_BIN_PKT *pRawPkt;
  IXXAT_BIN_PKT *pIxPkt;
  int retVal = 0;

  // look for start string "B " (see user manual)
  while (pApp->iSIdx < pApp->iSLen)
  {
    // compare start signature
    if (strcmp((pBuf + pApp->iSIdx), PKT_SIGN_START) != 0)
    {
      pApp->iSIdx++;
      continue;
    }
    // "B " is found
    // check how many bytes in the buffer are remaining (since splitted messages can occur)
    pApp->iRByt = pApp->iSLen - pApp->iSIdx;
    if (pApp->iRByt < PKT_MSGLEN_OFFSET)
    {
      return 1;
    }

    // RAW message structure mapping
    pRawPkt = (IXXAT_BIN_PKT *)(pBuf + pApp->iSIdx);

    // extract message length
    int pktLen = swapOrderUINT16(pRawPkt->w_msglen);

    // check if we have full message length
    // TODO: Optional <space> or other char handling, i.e +1 (see manual)
    if (pApp->iRByt < (pktLen + 2 + 2))
    {
      return 2;
    }

    // buffer for bytes-in-order message
    pIxPkt = (IXXAT_BIN_PKT *)malloc(pktLen + 2);

    // store the address
    pApp->pPkt = pIxPkt;

    // parse message in swapped order since we have little endian
    pIxPkt->w_msgSign = swapOrderUINT16(pRawPkt->w_msgSign);
    pIxPkt->w_msglen = swapOrderUINT16(pRawPkt->w_msglen);
    pIxPkt->dw_id = swapOrderUINT32(pRawPkt->dw_id);
    pIxPkt->b_format = pRawPkt->b_format;
    pIxPkt->qw_timestamp = swapOrderUINT64(pRawPkt->qw_timestamp);
    pIxPkt->w_reserved = swapOrderUINT16(pRawPkt->w_reserved);
    pIxPkt->w_datalen = swapOrderUINT16(pRawPkt->w_datalen);

    // copy the databytes
    memcpy(pIxPkt->ab_data, pRawPkt->ab_data, pIxPkt->w_datalen);

    // check if we have space (optional) + PKT_END signature
    // +1 => 2 for start sign and '- 1' for last index
    // TBD: to check incremental value +2 or +1
    // pApp->iNIdx = pApp->iSIdx + pIxPkt->w_msglen + 2;
    pApp->iNIdx = pApp->iSIdx + pIxPkt->w_msglen + 1;

    // TODO: nice to have iNIdx check that it does not exceed packet length
    // Check if optional SPACE is available
    // TBD: Observed other chars?? e.g. 0x0C, 0x30, ...
    // if (pBuf[pApp->iNIdx] == ASCII_SPACE)
    if (pBuf[pApp->iNIdx] != ASCII_CR)
    {
      pApp->iNIdx++;
    }

    // Check end signature
    if (pBuf[pApp->iNIdx] == ASCII_B)
    {
      printf("*Warn. Direct Start sign: %x, %x, %x\n", pBuf[pApp->iNIdx - 1], pBuf[pApp->iNIdx], pBuf[pApp->iNIdx + 1]);
    }
    else if ((pBuf[pApp->iNIdx] != ASCII_CR) || (pBuf[pApp->iNIdx + 1] != ASCII_LF))
    {
      printf("*Warn. End sign: %x, %x, %x\n", pBuf[pApp->iNIdx - 1], pBuf[pApp->iNIdx], pBuf[pApp->iNIdx + 1]);
    }
    else
    {
      pApp->iNIdx += 2;
    }

    // reset remaining bytes
    pApp->iRByt = 0;

    // return packet found
    return 0;
  }

  // return packet not found
  return -1;
}

/***************************************
 * Main
 ***************************************/
int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "ixxat_gw_pkg");
  ros::NodeHandle nh("~");
  // Port number to which the gateway broadcasts
  std::string port_num_gw;
  std::string ip_address_gw;
  if (!nh.getParam("port_num_gw", port_num_gw))
  {
    ROS_INFO("Missing Portnumber to Gateway");
  }
  if (!nh.getParam("ip_address_gw", ip_address_gw))
  {
    ROS_INFO("Missing IP Adress of Gateway");
  }
  // Port Number of gateway device
  const unsigned short port_gw = static_cast<unsigned short>(std::strtoul(port_num_gw.c_str(), NULL, 0));
  // Port Number of gateway device
  const boost::asio::ip::address address_gw = boost::asio::ip::address::from_string(ip_address_gw);

  std::cout << "Creating service and opening socket" << std::endl;
  // service creation
  boost::asio::io_service io_service;
  // socket creation
  tcp::socket socket(io_service);
  // connection
  socket.connect(tcp::endpoint(address_gw, port_gw));

  std::string send_init = "c init 500\n\r";
  std::string send_start = "c start\n\r";
  std::string send_stop = "c stop\n\r";

  std::cout << "Trying to initiate connection" << std::endl;

  boost::system::error_code error_init;
  boost::asio::write(socket, boost::asio::buffer(send_init), error_init);
  !error_init ? std::cout << "Client sent init command!" << std::endl : std::cout << "Init sent failed: " << error_init.message() << std::endl;

  boost::system::error_code error_start;
  boost::asio::write(socket, boost::asio::buffer(send_start), error_start);
  !error_start ? std::cout << "Client sent start command!" << std::endl : std::cout << "Start sent failed: " << error_start.message() << std::endl;

  std::cout << "Starting publishers" << std::endl;
  
  // Publisher inits. Paste publishers.txt from c-coderdbc here!
  // example: 
  ros::Publisher publish_Frame_1 = nh.advertise<ixxat_gw::Frame_1>("Frame_1", 1);

  /* Initilaize loop rate  */
  ros::Rate loop_rate(loopSpeed);

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
  APP_IXX_PP *pApp;
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
  while (ros::ok())
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

        if (retVal == 0) //message found and message is complete
        {
          // search index for next search to next position
          pApp->iSIdx = pApp->iNIdx;
          
          // Call the right function according to the ID to decode the binary string into signals
          // Paste evaluation.txt from c-coderdbc here!
          // example: 
          if (pApp->pPkt->dw_id == 1) {
            ixxat_gw::Frame_1 *decodedMsg;
            Frame_1_raw *rawMsg = (FRAME_1_raw *)(pApp->pPkt->ab_data);
            decodedMsg = Frame_1_APP(rawMsg);
            publish_Frame_1_.publish(*decodedMsg);
            free(pApp->pPkt);
            free(decodedMsg);
          }
          
        }
        else //end of buffer reached, last message is incomplete
        {
          //copy the packet reminder into buffer
          memcpy(pApp->pSBuf, (pApp->pSBuf + pApp->iNIdx), pApp->iRByt);
          break;
        }
      }
    }
    //ROS spin
    ros::spinOnce();
    loop_rate.sleep();
  }
  /***************************************
  * Node closing
  ***************************************/
  boost::system::error_code error_stop;
  boost::asio::write(socket, boost::asio::buffer(send_stop), error_stop);
  !error_stop ? std::cout << "Client sent stop command!" << std::endl : std::cout << "Stop sent failed: " << error_stop.message() << std::endl;
  //Close the TCP socket, so the gateway can accept a new connection in the future
  socket.close();
  return 0;
}
