/*
 * Copyright (c) 2019, Markus Bader
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author    Markus Bader
 * @date      2019-05-15
 * @brief     Demo program which uses the EasyProfile lib and the boost::asio:serial to read gyro values. 
 * The demo program initializes first a calibration and plots afterward measurements.
 **/


#include <iostream>
#include <string>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <EasyProfile/EasyProfile.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define SERIAL_PORT "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0"

bool loop = true;

void my_handler(int s) {
    loop = false;
}
using namespace boost;



/**
 * @brief     Decodes byte streams
 * @notes     Most parts of this functions are taken from HelloMotionModule.cpp of the orignial TransducerM_Example_CPP_QT_V124R2
 **/
std::string Serial_ISR(char* rxData, int rxSize, EasyObjectDictionary &eOD, EasyProfile &eP) {
    std::stringstream msg;
    Ep_Header header;
    if(EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header)) {   // Step 2: Tell the CAL that new data has arrived.
        //         It does not matter if the new data only contains a fraction
        //         of a complete package, nor does it matter if the data is broken
        //         during the transmission. On_RecvPkg() will only return EP_SUCC_
        //         when a complete and correct package has arrived.

        // Example Reading of the Short ID of the device who send the data:
        uint32 fromId = header.fromId;                         // Step 3.1:  Now we are able to read the received payload data.
        //            header.fromId tells us from which Motion Module the data comes.

        (void)fromId;                                          //            Supress "parameter unused" complier warning


        switch (header.cmd) {                                  // Step 3.2: header.cmd tells what kind of data is inside the payload.
        case EP_CMD_ACK_: {                                    //           We can use a switch() as demonstrated here to do different
            Ep_Ack ep_Ack;                                     //           tasks for different types of data.
            if(EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack)) {

            }
        } break;
        case EP_CMD_STATUS_: {
            Ep_Status ep_Status;
            if(EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status)) {
                int qos = ep_Status.sysState.bits.qos;        //  This is how you read the Quality-of-Service
                //  For definition of the digits, please refer to the
                //  defintion of 'Ep_Status_SysState' in EasyObjectDictionary.h
                (void)qos;
            }
        }
        break;
        case EP_CMD_Raw_GYRO_ACC_MAG_: {
            Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
            if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag)) {

            }
        } break;
        case EP_CMD_Q_S1_S_: {
            Ep_Q_s1_s ep_Q_s1_s;
            if(EP_SUCC_ == eOD.Read_Ep_Q_s1_s(&ep_Q_s1_s)) {

            }
        } break;
        case EP_CMD_Q_S1_E_: {
            Ep_Q_s1_e ep_Q_s1_e;
            if(EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e)) { // Step 3.3: If we decided that the received Quaternion should be used,
                //           Here is an example of how to access the Quaternion data.
                float q1 = ep_Q_s1_e.q[0];
                float q2 = ep_Q_s1_e.q[1];
                float q3 = ep_Q_s1_e.q[2];
                float q4 = ep_Q_s1_e.q[3];
                uint32 timeStamp = ep_Q_s1_e.timeStamp;     //           TimeStamp indicates the time point (since the Module has been powered on),
                //           when this particular set of Quaternion was calculated. (Unit: uS)
                //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId  = ep_Q_s1_e.header.fromId; //           The ID indicates the device Short ID telling which Motion Module the data comes from.

                // Display the data on GUI:
                msg << "Q1=" << q1 << "  Q2=" << q2 << "  Q3=" << q3 << "  Q4=" << q4 << "  TimeStamp=" << timeStamp << " Device Short ID"<< deviceId << std::endl;
            }
        }
        break;
        case EP_CMD_EULER_S1_S_: {
            Ep_Euler_s1_s ep_Euler_s1_s;
            if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_s(&ep_Euler_s1_s)) {

            }
        } break;
        case EP_CMD_EULER_S1_E_: {
            Ep_Euler_s1_e ep_Euler_s1_e;
            if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_e(&ep_Euler_s1_e)) {

            }
        } break;
        case EP_CMD_RPY_: {
            Ep_RPY ep_RPY;
            if(EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY)) {    //           Another Example reading of the received Roll Pitch and Yaw
                float roll  = ep_RPY.roll;
                float pitch = ep_RPY.pitch;
                float yaw   = ep_RPY.yaw;
                uint32 timeStamp = ep_RPY.timeStamp;      //           TimeStamp indicates the time point (since the Module has been powered on),
                //           when this particular set of Roll-Pitch-Yaw was calculated. (Unit: uS)
                //           Note that overflow will occure when the uint32 type reaches its maximum value.
                uint32 deviceId  = ep_RPY.header.fromId;  //           The ID indicates from wich IMU Module the data comes from.


                // Display the data on GUI:
                msg << "Roll=" << roll << "  Pitch=" << pitch << "  Yaw=" << yaw << "  TimeStamp=" << timeStamp << " Device Short ID"<< deviceId << std::endl;
            }
        }
        break;
        case EP_CMD_GRAVITY_: {
            Ep_Gravity ep_Gravity;
            if(EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity)) {

            }
        } break;
        case EP_CMD_CALIB_: {
            Ep_Calib ep_Calib;
            if(EP_SUCC_ == eOD.Read_Ep_Calib(&ep_Calib)) {
                uint32 deviceId  = ep_Calib.header.fromId;
                uint8  calibType = ep_Calib.calibType;
                uint16 status    = ep_Calib.ctrlVal;

                (void)deviceId;                      // Remove 'variable unused' warning from compiler

                //--------------------------------------------
                // CalibB status report
                if(calibType == (4 + 0x10)) {        // 4 means calibration type is CalibB.  +0x10 means it is its status report.
                    if(status == 1) {                // calibration successful.
                        msg << "CalibB Successful!" << std::endl;
                    }
                    else {
                        msg << "CalibB Failed!" << std::endl;
                    }
                }
                // CalibB status report
                //--------------------------------------------

                //--------------------------------------------
                // Forced calibration status report
                else if(calibType == (5 + 0x10)) {   // 5 means calibration type is Force Calibration,  +0x10 means it is its status report.
                    if(status == 0) {                // calibration successful.
                        msg << "Forced Calibration Processing.." << std::endl;
                    }
                    else if(status == 1) {
                        msg << "Forced Calibration Successful!" << std::endl;
                    }
                    else if(status == 5) {
                        msg << "Forced Calibration successfully revoked!" << std::endl;
                    }
                    else {
                        msg << "Forced Calibration Failed!" << std::endl;
                    }
                }
                // Forced calibration status report
                //--------------------------------------------
            }
        }
        break;
        }

    }
    return msg.str();
}


void read_char()
{


    EasyObjectDictionary eOD;
    EasyProfile          eP(&eOD);

    asio::io_service io;
    asio::serial_port port ( io );

    port.open ( SERIAL_PORT );
    port.set_option ( asio::serial_port::baud_rate ( 115200 ) );
    port.set_option ( asio::serial_port::character_size ( asio::serial_port::character_size ( 8 ) ) );
    port.set_option ( asio::serial_port::parity ( asio::serial_port::parity::none ) );
    port.set_option ( asio::serial_port::stop_bits ( asio::serial_port::stop_bits::one ) );
    port.set_option ( asio::serial_port::flow_control ( asio::serial_port::flow_control::none ) );



    uint16 toId = EP_ID_BROADCAST_;                                              // Step 1: Get the destination device ID to which the request is sent
                                                                                 //         You can also choose to boardcast the request without specifying the target device ID
    if(EP_SUCC_ == eOD.Write_Ep_Calib(toId, 4, 2, 0xaa35, 0, 0)) {               // Step 2: Write the device ID and calibration cmd parameters into the data structure,
                                                                                 //         the parameter value "4, 1, 0xaa35, 0, 0" are fixed and do not change them.
        EP_ID_TYPE_  txToId;
        char*        txPkgData;
        int          txPkgSize;
        EP_CMD_TYPE_ txCmd = EP_CMD_CALIB_;                                      // Step 3: Define what type of data we are sending (Here we must declare that it is calibration command)
        if(EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txPkgData, &txPkgSize)) {  // Step 4: create a package out of the data structure (i.e. the payload) to be sent

            boost::asio::write(port, boost::asio::buffer(txPkgData, txPkgSize));
            std::cout << "Calibration result pending..." << std::endl;
        }
    }

    boost::array<char, 0x1> buf;
    boost::system::error_code ec;

    for ( size_t i = 0; i < 10000; i++) {
        usleep(10);
        std::size_t n = asio::read ( port, asio::buffer ( buf ),  asio::transfer_all(), ec );
        if ( ec ) {
            throw std::runtime_error ( "error!" );
        } else {

            std::string message = Serial_ISR(&buf[0], buf.size(), eOD, eP );
            if(message.empty() == false) {
                std::cout << message;
            }
        }

    }
    std::cout << "Closing port!" << std::endl;
    port.close();

}

int main ( int argc, char **argv )
{
    std::cout.setf(std::ios::unitbuf);
    std::cout << "Hello, TransducerM!" << std::endl;


    signal (SIGINT,my_handler);

    read_char();
    return 0;
}
