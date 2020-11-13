using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace MavlinkTestServer
{
    class TCPServer
    {
        public const int BUFF_SIZE = 65536;
        byte[] bytesIn_;
        readonly NetworkStream nwStream;
        bool isDead = true;
        public TCPServer()
        {
            //---listen at the specified IP and port no.---
            bytesIn_ = new byte[BUFF_SIZE];
            IPAddress localAdd = IPAddress.Parse("127.0.0.1");
            TcpListener listener = new TcpListener(localAdd, 5000);
            Console.WriteLine(" TCP Server start Listening...");
            listener.Start();

            //---incoming client connected---
            TcpClient client = listener.AcceptTcpClient();
            isDead = false;

            //---get the incoming data through a network stream---
            nwStream = client.GetStream();
            AsyncListen();

        }

        readonly MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
        private void AsyncListen()
        {
            nwStream.BeginRead(bytesIn_, 0, bytesIn_.Length, new AsyncCallback(OnReadCB), null);
        }

        private void OnReadCB(IAsyncResult ar)
        {
            try
            {
                if (isDead)
                    return;
                int numberOfBytesRead = nwStream.EndRead(ar);
                if (numberOfBytesRead <= 0)
                    return;
                byte[] truncArray = new byte[numberOfBytesRead];
                Array.Copy(bytesIn_, truncArray, truncArray.Length);
                MAVLink.MAVLinkMessage message = new MAVLink.MAVLinkMessage(truncArray);
                Console.WriteLine("TCP Server GOT: " + message.ToString());
                bytesIn_ = new byte[BUFF_SIZE];
                AsyncListen();
            }
            catch
            {
                //TODO: popup error in the GUI "error receiving TCP Data" 
            }
        }


        public void SendAll()
        {
            // for (float i = 1; i <= 6; i++)
            // {
            //     Send(5000, i, 11.0F + i / 10, 22.0F + i / 10, 33.0F + i / 10, 44.0F + i / 10, 2340, 1170);
            // }
            // Send(5001, 55.77F, 66.77F, 77.77F, 0, 0, 2340, 1170);
            // Send(5002, 88.8F, 99.8F, 111.8F, 222.8F, 0, 2340, 1170);
            // Send(5003, 12.34F, 23.45F, 34.56F, 45.67F, 56.78F, 2340, 1170);
            // Send(5004, 0F, 1F, 0F, 1F, 0F, 2340, 1170);
            // Send(5005, 1.1F, 2.2F, 3.3F, 4.4F, 5.5F, 6.6F, 1170);
            // Send(5006, 6.6F, 21.12F, 31.13F, 4.4F, 5.5F, 2340, 1170);
            // Send(5007, 98.7F, 87.6F, 76.5F, 65.4F, 54.3F, 43.2F, 1170);
            // Send(5008, 111.1F, 87.6F, 76.5F, 65.4F, 54.3F, 2340, 1170);
            // //Send(5009, 1F, 0F, 1F, 0F, 1F, 0F, 1170);
            // Send(5010, 2730, 333.33F, 1F, 46F, 55F, 2340, 1170);
            // Send(5011, 100F, 333.33F, 1F, 46F, 55F, 2340, 1170);
            // Send(5012, 19F, 28F, 37F, 46F, 55F, 2340, 1170);
            // float initial = 5000;
            // for (float i = 500; i  >= 0; i = i - 50)
            // {
            //     Thread.Sleep(50);
            //     Send(5011, 100F, 0, 1F, i, initial, 0, 0);
            //     Console.WriteLine("UDPServer sent 5011 message, curr Fuel is: " + i);
            // }
        }
        // TODO: turn to send function to be async 
        readonly object readlock = new object();
        public void Send(ushort pcommand, float p1, float p2, float p3, float p4,
            float p5, float p6, float p7)
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
                nwStream.Write(packet, 0, packet.Length);
            }
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
                        custom_mode = 2730
                    }, false, 1, 155, 0);
                nwStream.Write(packet, 0, packet.Length);
            }
        }
    }
}

