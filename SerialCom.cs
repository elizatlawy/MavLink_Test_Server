/*******************************************************************************
* Copyright (c) 2018 Elhay Rauper
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*     * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*     * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
* THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Management;

namespace MavlinkTestServer
{
    public class SerialCom : SerialPort
    {
        private bool connected_ = false;
        readonly List<Action<byte[]>> subsList_ = new List<Action<byte[]>>();
        readonly MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();

        // locking to prevent multiple reads on serial port
        readonly object readlock = new object();
        // our target sysid
        byte sysid;
        // our target compid
        byte compid;

        public bool Connect(string port, int baudRate)
        {
            return Connect(port, baudRate, Parity.None, 8, StopBits.One);
        }

        public bool Connect(string port,
                            int baudRate,
                            Parity parity,
                            int dataBits,
                            StopBits stopBits)
        {
            BaudRate = baudRate;
            PortName = port;
            Parity = parity;
            DataBits = dataBits;
            StopBits = stopBits;
            ReadTimeout = 2000;


            try
            {
                Open();
                connected_ = true;
            }
            catch (Exception e)
            {
                string err = e.ToString();
                connected_ = false;
            }
            BackgroundWorker bgw = new BackgroundWorker();

            bgw.DoWork += Bgw_DoWork;

            bgw.RunWorkerAsync();

            return connected_;
        }

        public void SendHB()
        {
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,
                    new MAVLink.mavlink_heartbeat_t()
                    {

                        autopilot = (byte)MAVLink.MAV_AUTOPILOT.INVALID,
                        base_mode = 0,
                        system_status = 0,
                        type = (byte)MAVLink.MAV_TYPE.GCS,
                        custom_mode = 2218
                    }, false, 1, 155, 0);

                Write(packet, 0, packet.Length);

            }
        }

        public void SendEsc()
        {
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
                    new MAVLink.mavlink_command_long_t()
                    {
                        command = 5000,
                        param1 = 1,
                        param2 = 11.1F,
                        param3 = 22.2F,
                        param4 = 33.3F,
                        param5 = 44.4F,
                        param6 = 31,
                        param7 = 3,

            }, false, 1, 155, 0);
                Write(packet, 0, packet.Length);
            }
        }

        public void SendFuel()
        {
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
                    new MAVLink.mavlink_command_long_t()
                    {
                        command = 5011,
                        param1 = 11.1F,
                        param2 = 1F,
                        param3 = 2.2F,
                        param4 = 150F,
                        param5 = 300F,
                        param6 = 1,
                        param7 = 3,

                    }, false, 1, 155, 0);
                Write(packet, 0, packet.Length);
            }
        }
        public void SendAll()
        {
            for (float i = 1; i <= 6; i++)
            {
                Send(5000, i, 11.0F + i / 10, 22.0F + i / 10, 33.0F + i / 10, 44.0F + i / 10, 2340, 1170);
            }
            Send(5001, 55.77F, 66.77F, 77.77F, 0, 0, 2340, 1170);
            Send(5002, 88.8F, 99.8F, 111.8F, 222.8F, 0, 2340, 1170);
            Send(5003, 12.34F, 23.45F, 34.56F, 45.67F, 56.78F, 2340, 1170);
            Send(5004, 0F, 1F, 0F, 1F, 0F, 2340, 1170);
            Send(5005, 1.1F, 2.2F, 3.3F, 4.4F, 5.5F, 6.6F, 1170);
            Send(5006, 6.6F, 21.12F, 31.13F, 4.4F, 5.5F, 2340, 1170);
            Send(5007, 98.7F, 87.6F, 76.5F, 65.4F, 54.3F, 43.2F, 1170);
            Send(5008, 111.1F, 87.6F, 76.5F, 65.4F, 54.3F, 2340, 1170);
            //Send(5009, 1F, 0F, 1F, 0F, 1F, 0F, 1170);
            Send(5010, 2730, 333.33F, 1F, 46F, 55F, 2340, 1170);
            Send(5011, 100F, 333.33F, 1F, 46F, 55F, 2340, 1170);
            Send(5012, 19F, 28F, 37F, 46F, 55F, 2340, 1170);
            Send(5020, 999F, 888F, 777F, 666F, 555F, 4444, 1170);
        }
        public void Send(ushort pcommand, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        {
            lock (readlock)
            {
                var packet = mavlink.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
                    new MAVLink.mavlink_command_long_t()
                    {
                        confirmation = 1,
                        command = pcommand,
                        param1 = p1,
                        param2 = p2,
                        param3 = p3,
                        param4 = p4,
                        param5 = p5,
                        param6 = p6,
                        param7 = p7,

                    }, false, 1, 155, 0);
                Write(packet, 0, packet.Length);
            }
        }
        void Bgw_DoWork(object sender, DoWorkEventArgs e)
        {
            while (IsOpen)
            {
                try
                {
                    MAVLink.MAVLinkMessage packet;
                    lock (readlock)
                    {
                        // read any valid packet from the port
                        packet = mavlink.ReadPacket(BaseStream);

                        // check its valid
                        if (packet == null || packet.data == null)
                            continue;
                    }

                    // check to see if its a hb packet from the comport
                    if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                    {
                        Console.WriteLine(DateTime.Now.ToString("hh:mm:ss.fff") + " Got HB");
                        var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                        // save the sysid and compid of the seen MAV
                        sysid = packet.sysid;
                        compid = packet.compid;

                        // request streams at 2 hz
                        var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                            new MAVLink.mavlink_request_data_stream_t()
                            {
                                req_message_rate = 2,
                                req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                start_stop = 1,
                                target_component = compid,
                                target_system = sysid
                            });

                        Write(buffer, 0, buffer.Length);

                        buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb, sysid: 255, compid: 2);

                        Write(buffer, 0, buffer.Length);
                    }

                    // from here we should check the the message is addressed to us
                    if (sysid != packet.sysid || compid != packet.compid)
                        continue;

                    Console.WriteLine(DateTime.Now.ToString("hh:mm:ss.fff") + " " + packet.msgtypename);

                    if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
                    //or
                    //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                    {
                        var esc = (MAVLink.mavlink_command_long_t)packet.data;
                        if (esc.command == 5000)
                        {
                            Console.WriteLine(DateTime.Now.ToString("hh:mm:ss.fff") + " GOT ESC: " + esc.param1);

                        }

                    }
                }
                catch
                {
                }

                System.Threading.Thread.Sleep(1);
            }
        }
        public bool Send(byte[] bytes)
        {
            if (connected_)
            {
                Write(bytes, 0, bytes.Length);
                return true;
            }
            return false;
        }

        public void CloseMe()
        {
            if (connected_)
            {
                connected_ = false;
                Close();
            }
        }

        public bool IsConnected()
        {
            return connected_;
        }

        public void Subscribe(Action<byte[]> func)
        {
            subsList_.Add(func);
        }

        public void Unsubscribe(Action<byte[]> func)
        {
            subsList_.Remove(func);
        }

        private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (connected_)
            {
                int numBytes = BytesToRead;
                var buff = new byte[numBytes];
                int bytesRead = Read(buff, 0, numBytes);
                if (bytesRead != numBytes)
                    return;
                try
                {
                    foreach (var func in subsList_)
                    {
                        func(buff);
                    }
                }
                catch (InvalidOperationException) // connection closed
                {

                }

            }
        }


        /// <summary>
        /// Get connected COM port names
        /// </summary>
        /// <returns>array of connected COM ports</returns>
        public static string[] GetConnectedPorts()
        {
            return GetPortNames().Distinct().ToArray();
        }

        /// <summary>
        /// Get detailed collection of connected COM port objects
        /// </summary>
        /// <returns>collection of connected COM ports objects</returns>
        public static List<ManagementBaseObject> GetConnectedPortsDetailedList()
        {
            ManagementObjectCollection portsCollection;
            List<ManagementBaseObject> comPortsList;

            using (var searcher = new ManagementObjectSearcher(@"Select * From Win32_PnPEntity"))
            {
                portsCollection = searcher.Get();
                comPortsList = new List<ManagementBaseObject>();

                foreach (var device in portsCollection)
                {
                    // verify port is COM port
                    string name = (String)device.GetPropertyValue("Name");
                    if (name != null && name.Contains("(COM"))
                    {
                        comPortsList.Add(device);
                    }
                }
            }

            return comPortsList;
        }

        /// <summary>
        /// Get port simplefied name. i.e. 'COM3'
        /// </summary>
        /// <param name="detailedPort">string of port details</param>
        /// <returns>port simplefied name</returns>
        public static string DetailedToSimplefiedPortName(string detailedPort)
        {
            int startIndx = detailedPort.IndexOf('(');
            int endIndx = detailedPort.IndexOf(')');
            int portNameLength = endIndx - startIndx - 1;
            string simplefiedPortName = detailedPort.Substring(startIndx + 1, portNameLength);

            if (startIndx == -1 ||
               endIndx == -1 ||
               startIndx >= endIndx ||
               portNameLength < 4 ||
               !simplefiedPortName.Contains("COM"))
            {
                return null;
            }

            return simplefiedPortName;
        }

        /// <summary>
        /// Get detailed list of connected COM ports strings
        /// </summary>
        /// <returns>List of connected COM ports strings</returns>
        public static List<string> GetConnectedPortsDetailedStrings()
        {
            List<string> portsStrings = new List<string>();
            List<ManagementBaseObject> comPorts = GetConnectedPortsDetailedList();

            foreach (var comDevice in comPorts)
            {
                string detailedPort = (String)comDevice.GetPropertyValue("Name");

                portsStrings.Add(detailedPort);
            }

            return portsStrings;
        }

        public static Parity StringToParity(string parity)
        {
            return parity switch
            {
                "None" => Parity.None,
                "Odd" => Parity.Odd,
                "Even" => Parity.Even,
                "Mark" => Parity.Mark,
                "Space" => Parity.Space,
                _ => Parity.None,
            };
        }

        public static StopBits StringToStopBits(string stopBits)
        {
            return stopBits switch
            {
                "0" => StopBits.None,
                "1" => StopBits.One,
                "1.5" => StopBits.OnePointFive,
                "2" => StopBits.Two,
                _ => StopBits.None,
            };
        }

    }
}
